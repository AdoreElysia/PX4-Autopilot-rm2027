/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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

#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

/* 本板 SPI 总线（来源：原理图直读）
 *
 *   SPI1 = IMU：华轩阳（HXY）「ICM-42688P」六轴模块，WHO_AM_I(0x01)=0x6A
 *          （厂商手册 §5.1；0x00 不是寄存器，手册 §4.1）
 *          SCK=PB3  MISO=PA6  MOSI=PD7   CS=PB4(GYRO_CS)   DRDY=PD6(GYRO_INT1)
 *   SPI2 = 空闲（SCK=PD3 MISO=PC2 MOSI=PC3）—— 本板未引出片选，故不声明设备
 *   SPI3 = 空闲（SCK=PC10 MISO=PC11 MOSI=PB2）—— 同上
 *   SPI4 = 本板未用（模板板用 PE12/13/14，本板 PE13 是 ESC DShot1、PE14 悬空）
 *
 * ★★ 为什么 SPI1 上一次登记 6 个 IMU 驱动（2026-09-30，T-A）
 *
 *   原理图标的是 ICM-42688-P，但实物【不实现 42688-P 的寄存器图】
 *   （0x75 可读可写、复位后读 0x00 —— 见开发日志 06）。既然磁罗盘/气压计已实测
 *   证明"实物 ≠ 图纸"，这颗 IMU 大概率也不是 42688-P，型号一度未知。
 *
 *   ⇒ 与其反复"改一行 → 重编 → 重烧 → 试"（一轮十几分钟），不如一次把
 *     "135 脚 2.5×3mm LGA"这一档的常见 SPI IMU 都编进固件，上电后在 QGC 的
 *     MAVLink Console 里逐个 `start` 试，谁 probe 成功谁就是真型号。
 *
 *   ★ 2026-09-30 定稿：以厂商手册为唯一基准，实物 = 华轩阳 HXY「ICM-42688P」
 *     （WHO_AM_I(0x01)=0x6A，手册 §5.1），驱动换成新建的独立驱动
 *     src/drivers/imu/hxy/hxy42688p（命令名 hxy42688p）。
 *     其余 5 行候选登记【故意保留】：硬件上 SPI1 只挂一颗芯片，多登记只是让
 *     "同一颗芯片的多种驱动假设"共存，谁 probe 成功谁就是真型号；而且
 *     devtype 精确匹配 ⇒ 只有对应 Kconfig 打开且 probe 通过的驱动才会占住设备。
 *
 * 【6 个候选共用同一条 CS(PB4) 与同一个 DRDY(PD6) —— 这是故意的】
 *   让 CS/DRDY 完全一致、只有 devtype 不同 ⇒ 变量单一。
 *   （驱动侧：SPIBusIterator 按 devtype 精确匹配登记项，再拿该登记的 devid 去选 CS ——
 *     见 platforms/nuttx/src/px4/stm/stm32_common/spi/spi.cpp:185 stm32_spixselect）
 *
 * ⚠️⚠️ 数量硬上限：SPI_BUS_MAX_DEVICES == 6
 *   platforms/common/include/px4_platform_common/spi.h:54 是裸 #define，板级不可覆盖；
 *   px4_spi_bus_devices_t::devices[] 就是这个长度（同文件:64）。
 *   ⇒ 本表 6 行【正好占满】SPI1 的设备槽，第 7 行会编译报错。
 *   ⇒ 想换候选型号必须"换一删一"，且必须【同时】改 default.px4board 里
 *     对应的 CONFIG_DRIVERS_IMU_* 开关（两处不一致 = 编译通过但运行时匹配不到）。
 *
 * 【安全性：逐个 start 试不会写坏芯片、也不会留下中断】
 *   1) probe() 只读 WHO_AM_I，且发生在建立设备节点【之前】
 *      （src/lib/drivers/device/nuttx/SPI.cpp:111，probe fail 直接 return）
 *   2) probe 失败 ⇒ init() 返回错误 ⇒ 实例被 delete
 *      （platforms/common/include/px4_platform_common/i2c_spi_buses.h:365-368）
 *   3) DRDY 的 EXTI 中断只在 RunImpl 里挂（各驱动的 DataReadyInterruptConfigure），
 *      失败实例根本进不到 Run ⇒ 不会留下挂在 PD6 上的中断。
 *      本板新驱动 hxy42688p 走"查询方式"，根本不挂中断。
 *   唯一副作用：42688P/42605/42670P/45686 的 probe 会先写一次 REG_BANK_SEL(0x76)，
 *   对非同族芯片而言是往某个未知寄存器写 1 字节（无害；换型号前可 CONFIG reboot 复位）。
 *   hxy42688p 的 probe 只读 WHO_AM_I(0x01)，无写操作。
 *
 * 【DRDY 是否必需】
 *   这 6 个驱动都把 DRDY 当【可选】：cfg.drdy_gpio == 0 时退回定时器调度采样；
 *   hxy42688p 则始终用定时轮询（查询方式，手册 §3.2.1 + §5.8），DRDY 只用于总线登记。
 *   本板原理图有 GYRO_INT1 = PD6，故 6 行统一给上，与原先 icm42688p 那一行保持一致
 *   （参考实现：boards/px4/fmu-v6x、boards/px4/fmu-v6c、boards/micoair/h743 等）。
 */
constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	initSPIBus(SPI::Bus::SPI1, {
		// ① Invensense ICM-42688-P —— 原理图标的型号；2026-09-29 实测 probe 失败（WHO_AM_I=0x00）
		initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::PortB, GPIO::Pin4}, SPI::DRDY{GPIO::PortD, GPIO::Pin6}),
		// ② Invensense ICM-42605 —— 同族但 WHO_AM_I=0x42、bank 切换寄存器不同
		initSPIDevice(DRV_IMU_DEVTYPE_ICM42605, SPI::CS{GPIO::PortB, GPIO::Pin4}, SPI::DRDY{GPIO::PortD, GPIO::Pin6}),
		// ③ Invensense ICM-42670-P —— WHO_AM_I=0x67，寄存器图与 42688 不同
		initSPIDevice(DRV_IMU_DEVTYPE_ICM42670P, SPI::CS{GPIO::PortB, GPIO::Pin4}, SPI::DRDY{GPIO::PortD, GPIO::Pin6}),
		// ④ Invensense ICM-45686 —— WHO_AM_I=0xE9，ID 寄存器地址都不一样（不是 0x75）
		initSPIDevice(DRV_IMU_DEVTYPE_ICM45686, SPI::CS{GPIO::PortB, GPIO::Pin4}, SPI::DRDY{GPIO::PortD, GPIO::Pin6}),
		// ⑤ 华轩阳（HXY）「ICM-42688P」六轴模块 —— 实物型号（手册 §5.1：WHO_AM_I(0x01)=0x6A）。
		//    驱动 = src/drivers/imu/hxy/hxy42688p（独立新驱动，按厂商手册时序初始化）。
		//    ⚠️ 本片读时序【没有 dummy 字节】（手册 §3.2.1），由该驱动的 transfer() 适配。
		initSPIDevice(DRV_IMU_DEVTYPE_HXY42688P, SPI::CS{GPIO::PortB, GPIO::Pin4}, SPI::DRDY{GPIO::PortD, GPIO::Pin6}),
		// ⑥ ST LSM6DSV（含 LSM6DSV16X）—— WHO_AM_I 在 0x0F（读 0x70）；
		//    ⚠️ 常量名是 DRV_IMU_DEVTYPE_ST_LSM6DSV（带 ST_ 前缀，drv_sensor.h:73），不是 DRV_IMU_DEVTYPE_LSM6DSV
		initSPIDevice(DRV_IMU_DEVTYPE_ST_LSM6DSV, SPI::CS{GPIO::PortB, GPIO::Pin4}, SPI::DRDY{GPIO::PortD, GPIO::Pin6}),
	}),

	/* ★ SPI2 = 第二颗板载 IMU：Bosch BMI088（2026-10-30 按 10-24 版 FMU_1 网表新增）
	 *
	 * 引脚（`PAD_NET` 硬证据，详见 work/FMU2-引脚识别-20261030/）：
	 *   SCK   PD3(84)    MISO PC2_C(17)   MOSI PC3_C(18)
	 *   加速度计 CS = PC0(15) = 网络 CSB1    DRDY = PE3(2) = 网络 INT1
	 *   陀螺仪   CS = PC1(16) = 网络 CSB2    DRDY = PE4(3) = 网络 INT3
	 *
	 * ★ BMI088 是【两颗独立的 die】共用一组 SPI 线：
	 *     加速度计 = SPI 从机，CSB1，中断 INT1/INT2
	 *     陀螺仪   = SPI 从机，CSB2，中断 INT3/INT4
	 *   所以这里必须登记【两条】initSPIDevice（ACC 与 GYR 各一条），
	 *   不能像单芯片 IMU 那样只登记一条。驱动的 -A / -G 分别对应这两条。
	 *
	 * ★ 中断脚选型（Kconfig 的四个选项刚好对上本板）：
	 *     加速度计 → BMI088_ACCELEROMETER_INT1 → PE3
	 *     陀螺仪   → BMI088_GYROSCOPE_INT3     → PE4
	 *   PE2(INT2) 与 PE5(INT4) 本板也接了，但**没有**对应驱动入口，
	 *   留着当备选/调试用；不要把它们登记成 DRDY，否则两个中断抢同一路数据。
	 *
	 * ⚠️⚠️ 读这一行之前先看 board README §3.1：`PC2_C`/`PC3_C` 是带模拟开关的模拟脚，
	 *   手册要求先闭合 SYSCFG 的开关才会把 SPI2 功能引到 `_C` 脚上，
	 *   而 NuttX 目前**没有**这段代码。若 SPI2 挂上去读不到 WHO_AM_I，
	 *   先怀疑这一点，不要先怀疑驱动或焊接。
	 */
	initSPIBus(SPI::Bus::SPI2, {
		// 加速度计：CS=PC0(CSB1)，DRDY=PE3(INT1)
		initSPIDevice(DRV_ACC_DEVTYPE_BMI088, SPI::CS{GPIO::PortC, GPIO::Pin0}, SPI::DRDY{GPIO::PortE, GPIO::Pin3}),
		// 陀螺仪：  CS=PC1(CSB2)，DRDY=PE4(INT3)
		initSPIDevice(DRV_GYR_DEVTYPE_BMI088, SPI::CS{GPIO::PortC, GPIO::Pin1}, SPI::DRDY{GPIO::PortE, GPIO::Pin4}),
	}),
};

static constexpr bool unused = validateSPIConfig(px4_spi_buses);
