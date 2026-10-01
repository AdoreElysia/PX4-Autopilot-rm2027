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
 * @file HXY42688P_registers.hpp
 *
 * Register map of the 华轩阳 (HuaXuanYang / HXY) "ICM-42688P" six-axis module.
 *
 * ⚠️ 唯一权威基准是厂商手册：
 *    《华轩阳HXY_ICM-42688P_六轴传感器模块_手册》
 *    路径：projects/competition/rm2027-drone/references/硬件手册/
 *          华轩阳HXY_ICM-42688P_六轴传感器模块_手册.pdf
 *          （文字提取版 …_手册_文字提取.txt）
 *    本文件里每条定义后面都标注了它在手册里的出处（§x.y）。
 *
 * ⚠️ 这份寄存器表【不是】 Bosch BMI270，也【不是】 InvenSense 原厂 ICM-42688-P：
 *    - 0x00 不是寄存器（手册 §4.1 通用寄存器表从 0x01 WHO_AM_I 开始，缺省 0x6A）
 *    - 没有 PWR_CONF(0x7C)、没有 CMD(0x7E)、没有 BMI270 的 FIFO 地址 0x24~0x26/0x46~0x49
 *    - 软复位是 SOFT_RST(0x4A)=0xA5（§5.42），不是 0x7E=0xB6
 *    - FIFO 在 0x1C~0x21（§5.16~§5.21），中断配置在 INT_CFG1/2 = 0x06/0x07（§5.4/§5.5）
 *
 * 本驱动只使用"查询方式"取数（轮询 0x0B~0x17），不用 FIFO、不用中断 ——
 * 手册 §3.2.1「SPI 读」与 §5.8「DATA_STAT 有 DRDY 状态位」共同支持这种用法。
 */

#pragma once

#include <stdint.h>

// 厂商手册「SPI 时钟频率 Fc(SPC)」典型值 = 10 MHz（本板实测同频无误：旧驱动 bad transfer = 0）
static constexpr uint32_t SPI_SPEED = 10 * 1000 * 1000;

namespace HXY42688P_REGS
{

// 手册 §3.2.1「SPI 读」：SPI 读命令 16 个时钟，Bit0 = 读写控制位，读 = 1。
// 写操作 bit7 = 0（命令字节 = 寄存器地址）。
static constexpr uint8_t DIR_READ = 0x80;

static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

/**
 * 通用寄存器（手册 §4.1 寄存器列表 / §5 寄存器描述）。
 * 只列出本驱动会碰到的地址；完整表见手册。
 */
enum class Register : uint8_t {
	WHO_AM_I	= 0x01,	// §5.1  r  缺省 0x6A（手册 §4.1：0x00 不在寄存器表里）
	OIS_CONF	= 0x04,	// §5.2  rw
	COM_CFG		= 0x05,	// §5.3  rw  缺省 0x50（B7 BOOT / B6 BDU / B4 Addr_Auto=1 / B0 SIM）
	INT_CFG1	= 0x06,	// §5.4  rw
	INT_CFG2	= 0x07,	// §5.5  rw
	HPF_LPF_CFG	= 0x08,	// §5.6  rw  缺省 0x80

	DATA_STAT	= 0x0B,	// §5.8  r   B5 GYR_CONF_ERR / B4 ACC_CONF_ERR / B2 DRDY_TMP / B1 DRDY_GYR / B0 DRDY_ACC

	ACCEL_DATA_X_H	= 0x0C,	// §5.9
	ACCEL_DATA_X_L	= 0x0D,
	ACCEL_DATA_Y_H	= 0x0E,	// §5.10
	ACCEL_DATA_Y_L	= 0x0F,
	ACCEL_DATA_Z_H	= 0x10,	// §5.11
	ACCEL_DATA_Z_L	= 0x11,
	GYRO_DATA_X_H	= 0x12,	// §5.12
	GYRO_DATA_X_L	= 0x13,
	GYRO_DATA_Y_H	= 0x14,	// §5.13
	GYRO_DATA_Y_L	= 0x15,
	GYRO_DATA_Z_H	= 0x16,	// §5.14
	GYRO_DATA_Z_L	= 0x17,

	TIME_H		= 0x18,	// §5.15
	TIME_M		= 0x19,
	TIME_L		= 0x1A,

	TEMP_H		= 0x22,	// §5.22  T = (int16)((TEMP_H<<8)|TEMP_L)/512 + 23℃
	TEMP_L		= 0x23,

	ACC_CONF	= 0x40,	// §5.37  rw  缺省 0xA8
	ACC_RANGE	= 0x41,	// §5.38  rw  缺省 0x02（±8g）
	GYR_CONF	= 0x42,	// §5.39  rw  缺省 0xA9
	GYR_RANGE	= 0x43,	// §5.40  rw  缺省 0x00（±2000dps）

	SOFT_RST	= 0x4A,	// §5.42  w   写 0xA5 = 复位整个电路，数据清零

	ACC_SELF_TEST	= 0x6D,	// §5.43  rw
	GYR_SELF_TEST	= 0x6F,	// §5.44  rw

	PWR_CTRL	= 0x7D,	// §5.45  rw  B3 TEMP_EN / B2 ACC_EN / B1 GYR_EN
	SEG_SEL		= 0x7F,	// §5.46  rw
};

// 手册 §5.42：SOFT_RST(0x4A) 写入 0xA5，复位整个电路、数据清零。
static constexpr uint8_t SOFT_RESET_CMD = 0xA5;

// 手册 §5.1 / §4.1：WHO_AM_I(0x01) 缺省值 0x6A。
static constexpr uint8_t WHO_AM_I_VALUE = 0x6A;

namespace PWR_CTRL_BIT
{
static constexpr uint8_t temp_en = Bit3;	// §5.45 B3 TEMP_EN
static constexpr uint8_t accel_en = Bit2;	// §5.45 B2 ACC_EN
static constexpr uint8_t gyr_en  = Bit1;	// §5.45 B1 GYR_EN
static constexpr uint8_t accel_gyr_temp_en = temp_en | accel_en | gyr_en; // = 0x0E（§5.45 备注要求先写 0x0E）
}

namespace DATA_STAT_BIT
{
static constexpr uint8_t gyr_conf_err = Bit5;	// §5.8 B5 陀螺仪初始化判断位（0 正常，1 错误）
static constexpr uint8_t acc_conf_err = Bit4;	// §5.8 B4 加速度计初始化判断位（0 正常，1 错误）
static constexpr uint8_t drdy_tmp     = Bit2;	// §5.8 B2 温度数据转换完成
static constexpr uint8_t drdy_gyr     = Bit1;	// §5.8 B1 陀螺三轴新数据全都转换完成
static constexpr uint8_t drdy_acc     = Bit0;	// §5.8 B0 加计三轴新数据全都转换完成
}

namespace ACC_CONF_BIT
{
// §5.37 ACC_CONF(0x40)：B7 ACC_FILTER_PERF | B6:B4 ACC_BWP[2:0] | B3:B0 ACC_ODR[3:0]
// §2.4 高性能模式（IMU）：ACC_FILTER_PERF = 1
static constexpr uint8_t acc_filter_perf = Bit7;
// §5.37 BWP 表：000 = OSR4_AVG1（高性能模式允许 ODR 12.5Hz~1.6kHz，1.6kHz 时平均次数 = 1）
static constexpr uint8_t acc_bwp_osr4_avg1 = 0 << 4;
// §5.37 ODR 表：1100 = 高性能模式 1600Hz
static constexpr uint8_t acc_odr_1600 = 0b1100;
// 组合结果：0b1_000_1100 = 0x8C（高性能 + BWP=000 + ODR=1600Hz）
static constexpr uint8_t acc_conf_high_perf_1600hz = acc_filter_perf | acc_bwp_osr4_avg1 | acc_odr_1600;
}

namespace ACC_RANGE_BIT
{
// §5.38 ACC_RANGE(0x41)：B7:B2 保留；B1:B0 = FS[1:0]
// 00 = ±2G / 01 = ±4G / 10 = ±8G（缺省）/ 11 = ±16G
static constexpr uint8_t acc_range_2g  = 0b00;
static constexpr uint8_t acc_range_4g  = 0b01;
static constexpr uint8_t acc_range_8g  = 0b10;
static constexpr uint8_t acc_range_16g = 0b11;
static constexpr uint8_t acc_range_mask = Bit1 | Bit0;
}

namespace GYR_CONF_BIT
{
// §5.39 GYR_CONF(0x42)：B7 GYR_FILTER_PERF | B6 GYR_NOISE_PERF | B5:B4 GYR_BWP[1:0] | B3:B0 GYR_ODR[3:0]
// §2.4 高性能模式（IMU）：GYR_FILTER_PERF = 1 且 GYR_NOISE_PERF = 1
static constexpr uint8_t gyr_filter_perf = Bit7;
static constexpr uint8_t gyr_noise_perf  = Bit6;
// §5.39 GYR_BWP 表：00 = OSR4_AVG1（平均次数 = 3.2kHz/ODR）
static constexpr uint8_t gyr_bwp_osr4_avg1 = 0 << 4;
// §5.39 ODR 表：1100 = 高性能/正常模式 1600Hz
// （§5.44 陀螺自检步骤 3 也把 0x42 配成 0x8C = FILTER_PERF=1 + BWP=00 + ODR=1100，即 1600Hz）
static constexpr uint8_t gyr_odr_1600 = 0b1100;
// 组合结果：0b11_00_1100 = 0xCC（高性能 + 噪声优化 + BWP=00 + ODR=1600Hz）
static constexpr uint8_t gyr_conf_high_perf_1600hz = gyr_filter_perf | gyr_noise_perf | gyr_bwp_osr4_avg1 | gyr_odr_1600;
}

namespace GYR_RANGE_BIT
{
// §5.40 GYR_RANGE(0x43)：**B3 OIS_RANGE**；B2:B0 = FS[2:0]
//
// ⚠️⚠️ 2026-10-01 更正（代码审查发现，已回手册逐列核对）：
//   手册 §5.40 的位域表列头是  `B7  B6  B5  B4  B3        B2   B1   B0`
//                       取值     `--  --  --  --  OIS_RANGE FS2  FS1  FS0`
//   ⇒ **OIS_RANGE 在 B3，不是 B4。** 本文件旧注释写 B4、旧代码写 `Bit4`，**都是错的**。
//   危害：旧 `gyr_range_mask = Bit4|Bit2|Bit1|Bit0 = 0x17`，读-改-写时会**去清 B4**，
//         而 B4 在手冊里是 `--`（保留位），且手册明确警告
//         「标识为"保留"的，在使用中不要更改，可能会引起永久性破坏」。
//   当前 `set_bits = 0`（FS=000、OIS_RANGE=0）⇒ 功能上今天看不出差别，
//   但**动保留位这件事本身必须消除**。
static constexpr uint8_t ois_range       = Bit3;	// ← 曾误写为 Bit4
static constexpr uint8_t gyr_fs_2000dps  = 0b000;
static constexpr uint8_t gyr_fs_1000dps  = 0b001;
static constexpr uint8_t gyr_fs_500dps   = 0b010;
static constexpr uint8_t gyr_fs_250dps   = 0b011;
static constexpr uint8_t gyr_fs_125dps   = 0b100;
static constexpr uint8_t gyr_fs_mask     = Bit2 | Bit1 | Bit0;
static constexpr uint8_t gyr_range_mask  = Bit3 | Bit2 | Bit1 | Bit0;	// = 0x0F（曾误为 0x17）
}

} // namespace HXY42688P_REGS
