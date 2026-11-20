/****************************************************************************
 *
 *   Copyright (c) 2026 RM2027 Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file flowtest.cpp
 * @brief PMW3901 光流单独测试工具 —— 绕开 PX4 的 sensors / EKF 链路
 *
 * ★ 为什么要有这个工具
 *
 *   正式固件里，光流数据要经过四层才到 EKF：
 *
 *       pmw3901 驱动  →  sensor_optical_flow
 *       sensors 模块  →  vehicle_optical_flow     ← 这一层要陀螺 + 朝下测距
 *       EKF2          →  只用 vehicle_optical_flow
 *
 *   其中 VehicleOpticalFlow 有两处硬依赖（源码核实）：
 *     · VehicleOpticalFlow.cpp:363  只接受 orientation == ROTATION_DOWNWARD_FACING 的测距
 *     · VehicleOpticalFlow.cpp:407  用 sensor_gyro 积分补 delta_angle
 *   本板（测试板）只焊了主控及其外围，**没有 IMU、没有 VL53** ⇒ 中间层拿不到输入
 *   ⇒ 光流数据会被**静默丢弃**，看起来就像"光流坏了"。
 *
 *   所以本工具**不碰 PX4 任何上层**：直接操作 SPI3 读芯片原始寄存器，
 *   用**固定高度**把像素流换算成 m/s，从指定串口打印。链路短到只有：
 *
 *       PMW3901 芯片  --SPI3-->  本工具  --串口-->  你的电脑
 *
 * ★ 为什么必须自己开串口（而不是 PX4_INFO）
 *
 *   本板 NuttX 配置是 **CONFIG_NO_SERIAL_CONSOLE=y**（8 个串口的 *_SERIAL_CONSOLE
 *   全部 not set）⇒ syslog / printf **没有任何串口出口**，PX4_INFO 打得再多也看不见。
 *   所以输出必须显式 open 一个 /dev/ttySx 再 write。
 *
 * ★ 默认输出口 = /dev/ttyS1 = USART2 = 板上 U21 座
 *   U21 是 WAFER-GH1.25-4PWB（4 线 GH1.25）：
 *       1 = 5V   2 = MCU RX(U1.25)   3 = MCU TX(U1.86)   4/5/6 = GND
 *   接线（对 USB-TTL）：模块 5V↔飞控 5V、模块 GND↔飞控 GND、
 *                      模块 RX ↔ U21.3(TX)、模块 TX ↔ U21.2(RX)   ← 收发交叉
 *
 * ★ 单位换算（这是"固定高度"的全部意义）
 *
 *   芯片给的是**像素**，不是速度。换算两步：
 *     像素 → 弧度：  rad     = px / 385        （385 来自 PX4 驱动 PMW3901.cpp:311-312）
 *     弧度 → 米：    meters  = rad * height    （height = 光流到地面的垂直距离）
 *   再除以时间：     m/s     = meters / dt
 *
 *   本工具把 height **固定**成 -z 给的常数（默认 0.10 m = 10 cm），
 *   因为测试板上没有测距仪。⚠️ 手册 Table 3 工作距离下限是 **80 mm**，
 *   10 cm 只比下限高 2 cm —— 能测，但太低时容易失焦，建议先试 15~20 cm。
 *
 * ★ 与 PX4 驱动的关系：**不要同时跑**
 *   两边都操作同一片 CS(PE14) 和同一组寄存器，而 Motion/Delta 寄存器是
 *   **读后清零**（read-to-clear）⇒ 同时跑会互相偷走对方的运动量，
 *   读出来全是碎片。本工具启动时会检查 /dev/pmw3901，存在就拒绝启动；
 *   确实要跑用 -F 强制。正确做法：先 `pmw3901 stop`，再 `flowtest start`。
 *
 * 用法：
 *   flowtest                       等价于 flowtest id（只认芯片，打印 ID）
 *   flowtest id                    读 Product_ID(0x00) / Inverse_Product_ID(0x5F)
 *   flowtest start [-d /dev/ttyS1] [-B 115200] [-b 3] [-z 0.10] [-i 10000] [-p 100000] [-C] [-n] [-F]
 *   flowtest stop
 *   flowtest status
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/spi.h>

#include <drivers/drv_sensor.h>
#include <drivers/drv_hrt.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <nuttx/spi/spi.h>

/* getreg32 / putreg32 与 RCC/SPI 寄存器地址（诊断用） */
#include <nuttx/arch.h>
#include <hardware/stm32h7x3xx_memorymap.h>
#include <hardware/stm32h7x3xx_rcc.h>

/* px4_spibus_initialize() 的声明在这里（展开成 stm32_spibus_initialize） */
#include <px4_platform/micro_hal.h>

/* ── 寄存器地址（手册 Table 9，Page 29）────────────────────────────────── */
#define REG_PRODUCT_ID          0x00   /* RO，复位值 0x49 */
#define REG_REVISION_ID         0x01
#define REG_MOTION              0x02
#define REG_DELTA_X_L           0x03
#define REG_DELTA_X_H           0x04
#define REG_DELTA_Y_L           0x05
#define REG_DELTA_Y_H           0x06
#define REG_SQUAL               0x07
#define REG_POWER_UP_RESET      0x3A
#define REG_INVERSE_PRODUCT_ID  0x5F   /* RO，复位值 0xB6 */

#define PMW3901_PRODUCT_ID          0x49
#define PMW3901_INVERSE_PRODUCT_ID  0xB6

/* SPI 读写方向位：写=bit7 置 1，读=bit7 清 0（手册 §8.1） */
#define DIR_WRITE(a) ((uint8_t)((a) | (1 << 7)))
#define DIR_READ(a)  ((uint8_t)((a) & 0x7f))

#define PMW3901_SPI_HZ       2000000      /* 手册 Table 3 上限 2 MHz */
#define PMW3901_SPI_MODE     SPIDEV_MODE0
#define PMW3901_TSWW_US      11           /* 两次写命令间隔，手册 tSWW ≥ 45us；上游用 11us 的实测值 */

/* 像素 → 弧度 的标度（上游驱动 PMW3901.cpp:311-312 用的就是这个） */
#define PMW3901_PX_TO_RAD    385.0f

#define DFLT_DEV             "/dev/ttyS1" /* USART2 = U21 座 */
#define DFLT_BUS             3            /* SPI3 */
#define DFLT_BAUD            115200
#define DFLT_HEIGHT_M        0.10f        /* 固定 10 cm */
#define DFLT_INTERVAL_US     10000        /* 10 ms 采一次（上游 PMW3901_SAMPLE_INTERVAL） */
#define DFLT_PRINT_US        100000       /* 100 ms 打一行 */

/* PX4 在板级 spi.cpp 里用 initSPIDevice(DRV_FLOW_DEVTYPE_PMW3901, CS=PE14) 登记，
 * 生成的 devid 就是 PX4_SPIDEV_ID(PX4_SPI_DEVICE_ID, devtype)；stm32_spixselect()
 * 拿这个 devid 匹配后再拉 CS GPIO（platforms/nuttx/.../stm32_common/spi/spi.cpp:185-197）
 * ⇒ 必须用同一个值，否则 SPI_SELECT 找不到设备、CS 永远不动。 */
#define PMW3901_SPI_DEVID    PX4_SPIDEV_ID(PX4_SPI_DEVICE_ID, DRV_FLOW_DEVTYPE_PMW3901)

namespace
{

volatile bool g_running = false;
volatile int g_task = -1;
volatile uint32_t g_lines = 0;
volatile uint32_t g_errors = 0;
volatile int g_last_id = -1;
volatile bool g_last_id_ok = false;

/* ★ 启动诊断：本板没有串口控制台，PX4_ERR 看不到。把每一步的结果写进
 * 这块固定 RAM，用 ST-Link 读符号地址就能知道卡在哪一步。
 *   0 = 还没进 start
 *   1 = 参数解析完成
 *   2 = access() 检查通过
 *   3 = px4_task_spawn_cmd 返回（看 g_task）
 *   4 = 任务已进入 flowtest_task
 *   5 = spi_open 成功
 *   6 = serial_open_out 成功
 *   7 = 读到芯片 ID（看 g_last_id）
 * 负值 = 对应步骤失败（-5 = spi_open 失败，-6 = 串口打不开） */
volatile int g_stage = 0;
volatile int g_spi_ret = 0;
volatile int g_ser_fd = -999;

/* ★ RCC 诊断：ST-Link 直接 mdw 读 RCC（D3 域）不可靠（读到 0 或空），
 * 所以让固件自己读出来、存在 RAM 里，再由 ST-Link 读 RAM —— RAM 读取是可靠的。 */
volatile uint32_t g_rcc_apb1lenr = 0;
volatile uint32_t g_spi3_cr1 = 0;
volatile uint32_t g_spi3_cfg1 = 0;
volatile uint32_t g_spi3_cfg2 = 0;
volatile uint32_t g_spi3_sr = 0;
volatile uint32_t g_spi2_cfg2 = 0;
/* write 诊断：有多少字节成功写出、最后一次 errno、失败次数 */
volatile uint32_t g_tx_bytes = 0;
volatile int g_tx_errno = 0;
volatile uint32_t g_tx_fail = 0;

/* 光流原始量（供 ST-Link 判读"为什么 squal=0"） */
volatile uint32_t g_last_raw = 0;       /* 0x08 RawData_Sum 均值 */
volatile uint32_t g_last_shutter = 0;   /* 0x0C:0x0B 曝光时间 */

struct spi_dev_s *g_spi = nullptr;

/****************************************************************************
 * SPI 底层：直接拿 NuttX 的 spi_dev_s，自己控制 CS / 模式 / 频率
 *
 * 为什么不用 device::SPI：那套是给 C++ 驱动用的，还要过 CDev 建设备节点。
 * 这里只要"读几个字节"，用 px4_mtd.cpp 的同一套裸接口最省事。
 ****************************************************************************/
int spi_open(int bus)
{
	/* ★ 2026-11-18 诊断结论（保留这段只为了下次不用重查）：
	 *
	 * 一开始用 ST-Link 的 `mdw 0x580244E8`（RCC_APB1LENR）读出来是 0x00024001，
	 * bit15(SPI3EN)=0，于是误判成"SPI3 时钟没开"。**这个判断是错的** ——
	 * ST-Link 直接读 RCC（D3 域）不可靠，读到的不是实时值。
	 *
	 * 让固件自己 getreg32 读出来存进 RAM，再让 ST-Link 读 RAM（RAM 读取可靠），
	 * 得到的才是真相：APB1LENR = 0x0000c001（bit14 SPI2EN + bit15 SPI3EN 都在）、
	 * SPI3 CR1 = 0x1001（SPE|MSTR）、CFG2 = 0x5007c007、SR = 0x1002。
	 * ⇒ NuttX 的 rcc_enableapb1() 本来就正确使能了 SPI3，不需要我们补。
	 *
	 * 所以下面这次 putreg32 只是**幂等兜底**（写的是同一个值，无副作用），
	 * 真正让 flowtest 跑起来的是修好了 bootloader（见 hw_config.h 的
	 * INTERFACE_USART=0：USART1 悬空收噪声会擦掉 app）。
	 */
	g_rcc_apb1lenr = getreg32(STM32_RCC_APB1LENR);
	g_spi2_cfg2 = getreg32(STM32_SPI2_BASE + 0x08);   /* 对照：能工作的 SPI2 */
	putreg32(g_rcc_apb1lenr | RCC_APB1LENR_SPI3EN, STM32_RCC_APB1LENR);
	__asm__ volatile("dsb" ::: "memory");
	g_rcc_apb1lenr = getreg32(STM32_RCC_APB1LENR);

	g_spi = px4_spibus_initialize(bus);

	if (g_spi == nullptr) {
		return -ENODEV;
	}

	/* 先设好总线参数；SPI_SELECT(devid,false) 把 CS 拉高（不选中） */
	SPI_LOCK(g_spi, true);
	SPI_SETFREQUENCY(g_spi, PMW3901_SPI_HZ);
	SPI_SETBITS(g_spi, 8);
	SPI_SETMODE(g_spi, PMW3901_SPI_MODE);
	SPI_SELECT(g_spi, PMW3901_SPI_DEVID, false);
	SPI_LOCK(g_spi, false);

	/* 把 SPI3 的寄存器也存一份进 RAM，供 ST-Link 读取诊断 */
	g_spi3_cr1  = getreg32(STM32_SPI3_BASE + 0x00);
	g_spi3_cfg1 = getreg32(STM32_SPI3_BASE + 0x04);
	g_spi3_cfg2 = getreg32(STM32_SPI3_BASE + 0x08);
	g_spi3_sr   = getreg32(STM32_SPI3_BASE + 0x14);

	return PX4_OK;
}

int spi_read_reg(uint8_t reg, uint8_t *out, unsigned count)
{
	uint8_t buf[8];

	if (count > sizeof(buf) - 1) {
		return -EINVAL;
	}

	buf[0] = DIR_READ(reg);

	for (unsigned i = 1; i <= count; i++) {
		buf[i] = 0;
	}

	SPI_LOCK(g_spi, true);
	SPI_SELECT(g_spi, PMW3901_SPI_DEVID, true);
	SPI_EXCHANGE(g_spi, buf, buf, count + 1);
	SPI_SELECT(g_spi, PMW3901_SPI_DEVID, false);
	SPI_LOCK(g_spi, false);

	memcpy(out, &buf[1], count);
	return PX4_OK;
}

int spi_write_reg(uint8_t reg, uint8_t value)
{
	uint8_t buf[2] = { DIR_WRITE(reg), value };

	SPI_LOCK(g_spi, true);
	SPI_SELECT(g_spi, PMW3901_SPI_DEVID, true);
	SPI_EXCHANGE(g_spi, buf, nullptr, 2);
	SPI_SELECT(g_spi, PMW3901_SPI_DEVID, false);
	SPI_LOCK(g_spi, false);

	usleep(PMW3901_TSWW_US);
	return PX4_OK;
}

/****************************************************************************
 * 芯片初始化
 *
 *   ① 上电复位（0x3A=0x5A）+ 把运动寄存器读一遍清掉
 *      —— 手册 §8.2 与上游 PMW3901.cpp:50-61 都要求复位后先读一次
 *   ② 写性能优化寄存器
 *      手册 Table 10 给了 15 条；上游还多了一批来自
 *      "PixArt demo kit V3.20 (21 Aug 2018)" 的寄存器，其中 0x67/0x73/0x70/0x71
 *      决定 shuttered 参数与 LED 驱动电流。
 *      这里**照抄上游的完整序列**：它是唯一经过实机验证的组合，
 *      少写几条会让 Squal（信号质量）偏低、静止时有伪运动。
 ****************************************************************************/
void pmw3901_init_chip()
{
	uint8_t v = 0, c1 = 0, c2 = 0;

	spi_write_reg(REG_POWER_UP_RESET, 0x5A);
	usleep(5000);

	spi_read_reg(REG_MOTION, &v, 1);
	spi_read_reg(REG_DELTA_X_L, &v, 1);
	spi_read_reg(REG_DELTA_X_H, &v, 1);
	spi_read_reg(REG_DELTA_Y_L, &v, 1);
	spi_read_reg(REG_DELTA_Y_H, &v, 1);
	usleep(1000);

	spi_write_reg(0x7F, 0x00);
	spi_write_reg(0x55, 0x01);
	spi_write_reg(0x50, 0x07);
	spi_write_reg(0x7F, 0x0E);
	spi_write_reg(0x43, 0x10);

	spi_read_reg(0x67, &v, 1);
	spi_write_reg(0x48, (v & (1 << 7)) ? 0x04 : 0x02);

	spi_write_reg(0x7F, 0x00);
	spi_write_reg(0x51, 0x7B);
	spi_write_reg(0x50, 0x00);
	spi_write_reg(0x55, 0x00);

	spi_write_reg(0x7F, 0x0E);
	spi_read_reg(0x73, &v, 1);

	if (v == 0) {
		spi_read_reg(0x70, &c1, 1);
		c1 = (c1 <= 28) ? (uint8_t)(c1 + 14) : (uint8_t)(c1 + 11);

		if (c1 > 0x3F) {
			c1 = 0x3F;
		}

		spi_read_reg(0x71, &c2, 1);
		c2 = (uint8_t)(((unsigned)c2 * 45) / 100);

		spi_write_reg(0x7F, 0x00);
		spi_write_reg(0x61, 0xAD);
		spi_write_reg(0x51, 0x70);
		spi_write_reg(0x7F, 0x0E);
		spi_write_reg(0x70, c1);
		spi_write_reg(0x71, c2);
	}

	spi_write_reg(0x7F, 0x00); spi_write_reg(0x61, 0xAD);
	spi_write_reg(0x7F, 0x03); spi_write_reg(0x40, 0x00);
	spi_write_reg(0x7F, 0x05); spi_write_reg(0x41, 0xB3);
	spi_write_reg(0x43, 0xF1); spi_write_reg(0x45, 0x14);
	spi_write_reg(0x5B, 0x32); spi_write_reg(0x5F, 0x34);
	spi_write_reg(0x7B, 0x08);
	spi_write_reg(0x7F, 0x06); spi_write_reg(0x44, 0x1B);
	spi_write_reg(0x40, 0xBF); spi_write_reg(0x4E, 0x3F);
	spi_write_reg(0x7F, 0x08); spi_write_reg(0x65, 0x20);
	spi_write_reg(0x6A, 0x18);
	spi_write_reg(0x7F, 0x09); spi_write_reg(0x4F, 0xAF);
	spi_write_reg(0x5F, 0x40); spi_write_reg(0x48, 0x80);
	spi_write_reg(0x49, 0x80); spi_write_reg(0x57, 0x77);
	spi_write_reg(0x60, 0x78); spi_write_reg(0x61, 0x78);
	spi_write_reg(0x62, 0x08); spi_write_reg(0x63, 0x50);
	spi_write_reg(0x7F, 0x0A); spi_write_reg(0x45, 0x60);
	spi_write_reg(0x7F, 0x00); spi_write_reg(0x4D, 0x11);
	spi_write_reg(0x55, 0x80); spi_write_reg(0x74, 0x21);
	spi_write_reg(0x75, 0x1F); spi_write_reg(0x4A, 0x78);
	spi_write_reg(0x4B, 0x78); spi_write_reg(0x44, 0x08);
	spi_write_reg(0x45, 0x50); spi_write_reg(0x64, 0xFF);
	spi_write_reg(0x65, 0x1F);
	spi_write_reg(0x7F, 0x14); spi_write_reg(0x65, 0x67);
	spi_write_reg(0x66, 0x08); spi_write_reg(0x63, 0x70);
	spi_write_reg(0x7F, 0x15); spi_write_reg(0x48, 0x48);
	spi_write_reg(0x7F, 0x07); spi_write_reg(0x41, 0x0D);
	spi_write_reg(0x43, 0x14); spi_write_reg(0x4B, 0x0E);
	spi_write_reg(0x45, 0x0F); spi_write_reg(0x44, 0x42);
	spi_write_reg(0x4C, 0x80);
	spi_write_reg(0x7F, 0x10); spi_write_reg(0x5B, 0x02);
	spi_write_reg(0x7F, 0x07); spi_write_reg(0x40, 0x41);
	spi_write_reg(0x70, 0x00);

	usleep(10000);

	spi_write_reg(0x32, 0x44);
	spi_write_reg(0x7F, 0x07); spi_write_reg(0x40, 0x40);
	spi_write_reg(0x7F, 0x06); spi_write_reg(0x62, 0xF0);
	spi_write_reg(0x63, 0x00);
	spi_write_reg(0x7F, 0x0D); spi_write_reg(0x48, 0xC0);
	spi_write_reg(0x6F, 0xD5);
	spi_write_reg(0x7F, 0x00); spi_write_reg(0x5B, 0xA0);
	spi_write_reg(0x4E, 0xA8); spi_write_reg(0x5A, 0x50);
	spi_write_reg(0x40, 0x80);
}

/****************************************************************************
 * 串口输出
 *
 * ⚠️ 两个必须照做的细节（都是 NuttX 的特性，都踩过）：
 *   1) c_cflag 必须带 **CS8** —— NuttX 的 stm32_serial.c 在 case TCSETS 里
 *      硬校验 (c_cflag & CSIZE) == CS8，否则 tcsetattr 直接 -EINVAL。
 *   2) 端口要在**真正用它的任务里**打开 —— NuttX 的 fd 表是**按任务**的，
 *      A 任务开的 fd 拿到 B 任务里用就是 EBADF。
 ****************************************************************************/
int serial_open_out(const char *dev, int baud)
{
	/* ★ 2026-11-18：先占住 0/1/2 三个低级 fd。
	 *
	 * 为什么：这个工具现在是在 rc.board_sensors 里**前台同步**执行的，
	 * 而 rcS 是以 `.`（source）方式跑脚本 — 此时进程的 0/1/2 可能指向
	 * 别处（或者根本没打开）。如果 open() 拿到 fd 0，write(0,...) 就
	 * 写进了标准输入，串口上一个字节都看不到（实测：g_lines 在涨、
	 * g_tx_bytes 也在涨，但串口空——就是写到 fd 0 去了）。
	 * 先把 0/1/2 用 /dev/null 占掉，open() 就会从 3 开始分配。 */
	for (int i = 0; i < 3; i++) {
		if (::open("/dev/null", O_RDWR) < 0) {
			break;
		}
	}

	int fd = ::open(dev, O_WRONLY | O_NONBLOCK | O_NOCTTY);

	if (fd < 0) {
		return -errno;
	}

	struct termios t;

	if (tcgetattr(fd, &t) != 0) {
		::close(fd);
		return -errno;
	}

	cfsetspeed(&t, (speed_t)baud);
	t.c_cflag = CS8 | CLOCAL | CREAD;
	t.c_iflag = 0;
	t.c_oflag = 0;
	t.c_lflag = 0;
	t.c_cc[VMIN] = 0;
	t.c_cc[VTIME] = 0;

	tcflush(fd, TCIOFLUSH);

	if (tcsetattr(fd, TCSANOW, &t) != 0) {
		int err = -errno;
		::close(fd);
		return err;
	}

	return fd;
}

void serial_puts(int fd, const char *s)
{
	size_t len = strlen(s);

	while (len > 0) {
		ssize_t n = write(fd, s, len);

		if (n <= 0) {
			if (errno == EINTR) {
				continue;
			}

			g_tx_errno = errno;
			g_tx_fail++;
			return;   /* 串口没接 / 缓冲满：丢弃，绝不阻塞采样 */
		}

		g_tx_bytes += (uint32_t)n;

		s += n;
		len -= (size_t)n;
	}
}

/****************************************************************************
 * 读一次运动量
 *
 * 一次传输读 6 个寄存器（0x02~0x07），共 12 字节。这是上游驱动的做法
 * （PMW3901.cpp:352-356），比 6 次单独传输快得多，也避免中间被插入。
 *
 * ★ 这些寄存器是**读后清零**（read-to-clear）：读走多少就没了，
 *   必须自己累加，不能指望"下次再读还在"。
 ****************************************************************************/
struct flow_raw {
	int16_t dx;
	int16_t dy;
	uint8_t squal;
	uint8_t raw_sum;     /* 0x08 RawData_Sum   —— 视场内原始像素和，越小越"没东西可看" */
	uint8_t raw_max;     /* 0x09 Maximum_RawData */
	uint8_t raw_min;     /* 0x0A Minimum_RawData */
	uint16_t shutter;    /* 0x0B/0x0C Shutter —— 曝光时间，越大说明越暗 */
};

/* 读 0x02~0x0C 的运动 + 光照相关寄存器。
 *
 * ★★★ 2026-11-18 踩过的坑，务必记住：
 *   PMW3901MB **不是**"发一次地址就能连续读"的芯片（不像 BMI088 那种）。
 *   它要求**每个寄存器单独发一次带地址的读命令**。
 *   我一开始图快，写成"发 DIR_READ(0x02) 然后后面 21 字节全填 0"，
 *   结果后面所有字节读回来都是 0x49 —— 那是 Product_ID(0x00) 的值。
 *   于是 raw=73(0x49)、shut=0x4949=18761、[min..max]=[73..73] 全是假的，
 *   看着像"没光照"，其实是读法错了。
 *   ⇒ 上游驱动 PMW3901.cpp:352-356 那种一次读 6 个寄存器、
 *     中间夹着 DIR_READ(...) 地址字节的写法才是对的。
 *
 * ★ 为什么要把 0x08~0x0C 也读出来：
 *   Squal(0x07)=0 有两种完全不同的原因，光看 Squal 分不清：
 *     a) 没纹理（对着白墙/纯色桌面） → RawData_Sum 低、Maximum 与 Minimum 接近
 *     b) 光照不足 / 镜头没进光       → Shutter 会顶到最大（≥0x1FFF）
 *   加上这几个量才能一眼判断到底该换表面还是该查光学/供电。
 */
void pmw3901_read_motion(flow_raw *out)
{
	/* 一次传输里把 11 个寄存器的"地址 + 数据"成对发出去：
	 *   [DIR_READ(0x02), x, DIR_READ(0x03), x, ... DIR_READ(0x0C), x]
	 * 每个 DIR_READ 之后的那一字节是占位（发送时给 0，接收时收回数据）。 */
	static const uint8_t regs[] = {
		REG_MOTION, REG_DELTA_X_L, REG_DELTA_X_H, REG_DELTA_Y_L, REG_DELTA_Y_H,
		REG_SQUAL, 0x08, 0x09, 0x0A, 0x0B, 0x0C
	};
	const unsigned nregs = sizeof(regs) / sizeof(regs[0]);
	uint8_t buf[2 * 11];

	for (unsigned i = 0; i < nregs; i++) {
		buf[2 * i]     = DIR_READ(regs[i]);
		buf[2 * i + 1] = 0;
	}

	SPI_LOCK(g_spi, true);
	SPI_SELECT(g_spi, PMW3901_SPI_DEVID, true);
	SPI_EXCHANGE(g_spi, buf, buf, 2 * nregs);
	SPI_SELECT(g_spi, PMW3901_SPI_DEVID, false);
	SPI_LOCK(g_spi, false);

	/* buf[2*i] 是地址回显（忽略），buf[2*i+1] 才是数据 */
	const uint8_t d_motion  = buf[1];
	const uint8_t d_dxl     = buf[3];
	const uint8_t d_dxh     = buf[5];
	const uint8_t d_dyl     = buf[7];
	const uint8_t d_dyh     = buf[9];
	const uint8_t d_squal   = buf[11];
	const uint8_t d_rawsum  = buf[13];
	const uint8_t d_rawmax  = buf[15];
	const uint8_t d_rawmin  = buf[17];
	const uint8_t d_shutl   = buf[19];
	const uint8_t d_shuth   = buf[21];

	out->dx = (int16_t)(((uint16_t)d_dxh << 8) | d_dxl);
	out->dy = (int16_t)(((uint16_t)d_dyh << 8) | d_dyl);
	out->raw_sum = d_rawsum;
	out->raw_max = d_rawmax;
	out->raw_min = d_rawmin;
	out->shutter = (uint16_t)(((uint16_t)d_shuth << 8) | d_shutl);

	/* 上游的垃圾过滤（PMW3901.cpp:365-372）：单帧位移 > 240 px 视为 SPI 噪声 */
	if (out->dx > 240 || out->dx < -240 || out->dy > 240 || out->dy < -240) {
		out->squal = 0;
		out->dx = 0;
		out->dy = 0;
		g_errors++;

	} else {
		out->squal = d_squal;
	}

	(void)d_motion;
}

/****************************************************************************
 * 后台采样任务
 ****************************************************************************/
struct flowtest_args {
	char dev[32];
	int baud;
	int bus;
	float height_m;
	uint32_t interval_us;
	uint32_t print_us;
	bool csv;
	bool no_init;
};

/* ★ 2026-11-18：改成**前台同步执行**，不再用 px4_task_spawn_cmd。
 *
 * 原因：px4_task_spawn_cmd 在这块板上不返回（g_stage 卡在 1、g_task 拿到 pid
 * 但线程永远不调度），怀疑与 clearenv() / 优先级有关。
 * 本工具的目的是"看光流数据"，前台阻塞完全够用：
 *   · rc.board_sensors 执行到这一行会一直停在这里（后面的 gyrofft_bridge 不启动）
 *   · 但串口数据会立刻持续输出，正是我们要的
 * 想停下来只需复位板子。 */
void flowtest_run(const flowtest_args &cfg_in)
{
	flowtest_args cfg = cfg_in;

	g_running = true;
	g_stage = 4;

	const int spi_ret = spi_open(cfg.bus);
	g_spi_ret = spi_ret;

	if (spi_ret != PX4_OK) {
		g_stage = -5;
		g_running = false;
		g_task = -1;
		return;
	}

	g_stage = 5;

	int fd = serial_open_out(cfg.dev, cfg.baud);
	g_ser_fd = fd;

	if (fd < 0) {
		PX4_ERR("flowtest: 打开 %s 失败 (%d)", cfg.dev, fd);
		g_stage = -6;
		g_running = false;
		g_task = -1;
		return;
	}

	g_stage = 6;

	char line[192];

	/* 先认芯片，认不到就退出——省的对着一片没接的板子调半天 */
	uint8_t pid = 0, ipid = 0;
	spi_read_reg(REG_PRODUCT_ID, &pid, 1);
	spi_read_reg(REG_INVERSE_PRODUCT_ID, &ipid, 1);
	g_last_id = pid;
	g_last_id_ok = (pid == PMW3901_PRODUCT_ID);
	g_stage = 7;

	snprintf(line, sizeof(line),
		 "# flowtest: bus=SPI%d cs=PE14 mode=0 %dHz dev=%s height=%.3fm scale=%.0fpx/rad\r\n",
		 cfg.bus, PMW3901_SPI_HZ, cfg.dev, (double)cfg.height_m, (double)PMW3901_PX_TO_RAD);
	serial_puts(fd, line);

	snprintf(line, sizeof(line),
		 "# Product_ID(0x00)=0x%02x %s Inverse_ID(0x5F)=0x%02x\r\n",
		 pid, (pid == PMW3901_PRODUCT_ID) ? "OK" : "!! expect 0x49, check wiring/power !!", ipid);
	serial_puts(fd, line);

	if (pid != PMW3901_PRODUCT_ID) {
		/* 认不到芯片就不 init（避免往一片没接的芯片乱写） */
		serial_puts(fd, "# chip not found, stop. check VIN=3.3V / GND / SCK=PC10 / MISO=PC11 / MOSI=PB2 / CS=PE14\r\n");
		usleep(200000);
		::close(fd);
		g_running = false;
		g_task = -1;
		return;
	}

	if (!cfg.no_init) {
		pmw3901_init_chip();
		serial_puts(fd, "# init register sequence written\r\n");
	}

	if (cfg.csv) {
		serial_puts(fd, "# X_mm,Y_mm,X_mm_s,Y_mm_s,quality,有信号\r\n");

	} else {
		serial_puts(fd, "\r\n");
		serial_puts(fd, "   时间      X位移    Y位移      X速度    Y速度    质量\r\n");
		serial_puts(fd, "    (s)       (mm)     (mm)     (mm/s)   (mm/s)   (0-45)\r\n");
		serial_puts(fd, "----------------------------------------------------------------\r\n");
	}

	hrt_abstime t_start = hrt_absolute_time();
	hrt_abstime t_last_print = t_start;
	int32_t sum_dx = 0, sum_dy = 0;
	uint32_t sum_q = 0;
	uint32_t sum_raw = 0;
	uint32_t n_samples = 0;

	while (g_running) {
		flow_raw raw{};
		pmw3901_read_motion(&raw);

		sum_dx += raw.dx;
		sum_dy += raw.dy;
		sum_q += raw.squal;
		sum_raw += raw.raw_sum;
		n_samples++;

		hrt_abstime now = hrt_absolute_time();

		if ((now - t_last_print) >= cfg.print_us && n_samples > 0) {
			const float dt_s = (now - t_last_print) * 1e-6f;

			/* 像素 → 弧度 → 米（用固定高度换算） */
			const float x_mm = ((float)sum_dx / PMW3901_PX_TO_RAD) * cfg.height_m * 1000.0f;
			const float y_mm = ((float)sum_dy / PMW3901_PX_TO_RAD) * cfg.height_m * 1000.0f;
			const float x_mm_s = x_mm / dt_s;
			const float y_mm_s = y_mm / dt_s;
			const uint32_t q_avg = sum_q / n_samples;
			const uint32_t raw_avg = sum_raw / n_samples;

			if (cfg.csv) {
				snprintf(line, sizeof(line),
					 "%.2f,%.2f,%.2f,%.2f,%" PRIu32 ",%s\r\n",
					 (double)x_mm, (double)y_mm, (double)x_mm_s, (double)y_mm_s,
					 q_avg, (q_avg > 0) ? "yes" : "no");

			} else {
				snprintf(line, sizeof(line),
					 " %7.2f  %+8.2f  %+8.2f  %+8.2f  %+8.2f   %3" PRIu32 "  %s\r\n",
					 (double)((now - t_start) * 1e-6f),
					 (double)x_mm, (double)y_mm, (double)x_mm_s, (double)y_mm_s,
					 q_avg, (q_avg == 0) ? "  << 看不到纹理" : "");
			}

			serial_puts(fd, line);
			g_lines++;
			g_last_raw = raw_avg;

			sum_dx = 0;
			sum_dy = 0;
			sum_q = 0;
			sum_raw = 0;
			n_samples = 0;
			t_last_print = now;
		}

		px4_usleep(cfg.interval_us);
	}

	serial_puts(fd, "# flowtest stopped\r\n");
	usleep(50000);
	::close(fd);
	g_task = -1;
	return;
}

int cmd_id()
{
	if (spi_open(DFLT_BUS) != PX4_OK) {
		PX4_ERR("flowtest: SPI%d 打不开（该总线没在 NuttX 配置里使能？）", DFLT_BUS);
		return -1;
	}

	uint8_t pid = 0, rev = 0, ipid = 0;
	spi_read_reg(REG_PRODUCT_ID, &pid, 1);
	spi_read_reg(REG_REVISION_ID, &rev, 1);
	spi_read_reg(REG_INVERSE_PRODUCT_ID, &ipid, 1);

	/* 没有串口控制台 → 同时往 U21 打一份，否则什么都看不到 */
	int fd = serial_open_out(DFLT_DEV, DFLT_BAUD);

	if (fd >= 0) {
		char line[192];
		snprintf(line, sizeof(line),
			 "flowtest id: Product_ID=0x%02x (expect 0x49) Revision=0x%02x Inverse_ID=0x%02x (expect 0xB6)\r\n",
			 pid, rev, ipid);
		serial_puts(fd, line);
		usleep(200000);
		::close(fd);
	}

	PX4_INFO("flowtest id: Product_ID=0x%02x Revision=0x%02x Inverse=0x%02x", pid, rev, ipid);

	return (pid == PMW3901_PRODUCT_ID) ? 0 : 1;
}

void usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
PMW3901 standalone optical-flow test. Bypasses the PX4 sensors/EKF chain and
talks to the chip on SPI3 directly, converting pixel flow to velocity using a
fixed height. Output goes to a serial port (default U21 = USART2 = /dev/ttyS1
because this board has CONFIG_NO_SERIAL_CONSOLE).

### Examples
flowtest id
flowtest start
flowtest start -z 0.10 -C
flowtest stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("flowtest", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("id", "read Product_ID / Inverse_Product_ID");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "sample in background and stream to serial");
	PRINT_MODULE_USAGE_PARAM_STRING('d', DFLT_DEV, "<dev>", "output serial port (default U21)", true);
	PRINT_MODULE_USAGE_PARAM_INT('B', DFLT_BAUD, 9600, 3000000, "baudrate", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', DFLT_BUS, 1, 6, "SPI bus (default SPI3)", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('z', DFLT_HEIGHT_M, 0.01f, 30.0f, "fixed height in m (default 0.10)", true);
	PRINT_MODULE_USAGE_PARAM_INT('i', DFLT_INTERVAL_US, 1000, 100000, "sample interval us", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', DFLT_PRINT_US, 10000, 2000000, "print interval us", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('C', "CSV output", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('n', "skip init register sequence", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('F', "force start even if /dev/pmw3901 exists", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "stop background task");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "print status");
}

} // namespace

extern "C" __EXPORT int flowtest_main(int argc, char *argv[]);

int flowtest_main(int argc, char *argv[])
{
	if (argc < 2) {
		return cmd_id();
	}

	const char *verb = argv[1];

	if (!strcmp(verb, "id")) {
		return cmd_id();
	}

	if (!strcmp(verb, "status")) {
		PX4_INFO("flowtest: %s, lines=%" PRIu32 ", err=%" PRIu32 ", last_id=0x%02x(%s)",
			 g_running ? "running" : "stopped", g_lines, g_errors, g_last_id,
			 g_last_id_ok ? "OK" : "BAD");
		return 0;
	}

	if (!strcmp(verb, "stop")) {
		if (!g_running) {
			PX4_INFO("flowtest: not running");
			return 0;
		}

		g_running = false;

		for (int i = 0; i < 100 && g_task != -1; i++) {
			usleep(10000);
		}

		PX4_INFO("flowtest: stopped");
		return 0;
	}

	if (!strcmp(verb, "start")) {
		if (g_running) {
			PX4_ERR("flowtest: already running, stop first");
			return -1;
		}

		static flowtest_args cfg;
		memset(&cfg, 0, sizeof(cfg));
		strncpy(cfg.dev, DFLT_DEV, sizeof(cfg.dev) - 1);
		cfg.baud = DFLT_BAUD;
		cfg.bus = DFLT_BUS;
		cfg.height_m = DFLT_HEIGHT_M;
		cfg.interval_us = DFLT_INTERVAL_US;
		cfg.print_us = DFLT_PRINT_US;
		cfg.csv = false;
		cfg.no_init = false;
		bool force = false;

		int myoptind = 1;
		const char *myoptarg = nullptr;
		int ch;

		while ((ch = px4_getopt(argc, argv, "d:B:b:z:i:p:CnF", &myoptind, &myoptarg)) != EOF) {
			switch (ch) {
			case 'd':
				if (myoptarg) {
					strncpy(cfg.dev, myoptarg, sizeof(cfg.dev) - 1);
					cfg.dev[sizeof(cfg.dev) - 1] = '\0';
				}

				break;

			case 'B': cfg.baud = atoi(myoptarg); break;
			case 'b': cfg.bus = atoi(myoptarg); break;
			case 'z': cfg.height_m = (float)atof(myoptarg); break;
			case 'i': cfg.interval_us = (uint32_t)atoi(myoptarg); break;
			case 'p': cfg.print_us = (uint32_t)atoi(myoptarg); break;
			case 'C': cfg.csv = true; break;
			case 'n': cfg.no_init = true; break;
			case 'F': force = true; break;
			default: usage(); return -1;
			}
		}

		g_lines = 0;
		g_errors = 0;
		g_stage = 1;

		/* 和 PX4 驱动抢同一片 CS / 同一组读后清零寄存器 —— 必须拦一下 */
		if (!force && access("/dev/pmw3901", F_OK) == 0) {
			PX4_ERR("flowtest: PX4 pmw3901 driver is running; both will steal each other's "
				"read-to-clear motion. Run `pmw3901 stop` first, or use -F.");
			g_stage = -2;
			return -1;
		}

		g_stage = 2;

		/* ★ 前台同步跑：这个函数返回之前，串口数据会一直往外发。
		 *   所以 rc.board_sensors 会停在这里（后面的命令不执行）。
		 *   要看数据就让它跑；要停就复位板子。 */
		flowtest_run(cfg);
		g_stage = 3;

		return 0;
	}

	usage();
	return -1;
}
