/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file Bosch_BMI270_registers.hpp
 *
 * Bosch BMI270 registers.
 *
 */

#pragma once

#include <cstdint>
#include <cstddef>

// TODO: move to a central header
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

namespace Bosch_BMI270
{
static constexpr uint32_t SPI_SPEED = 10 * 1000 * 1000; // 10MHz SPI serial interface
static constexpr uint8_t DIR_READ = 0x80;

static constexpr uint8_t chip_id = 0x24;

enum class Register : uint8_t {
	CHIP_ID            = 0x00,

	// ★ rm2027/fmu-h743（2026-09-30）：本板兼容件（华轩阳 HXY「ICM-42688P」）把器件 ID
	//   放在 0x01（WHO_AM_I，缺省 0x6A），而 0x00 不在它的寄存器表里（读出 0x00）。
	//   正品 BMI270 的 CHIP_ID 在 0x00=0x24。见
	//   work/spi-runtime/器件手册对照-华轩阳ICM42688P.md
	WHO_AM_I           = 0x01,

	ERR_REG            = 0x02,
	STATUS	           = 0x03,

	// ★ rm2027/fmu-h743（2026-09-30）：本片特有 —— 数据就绪状态 + 「配置是否被器件接受」
	//   的错误标志（厂商手册 §5.8）。正品 BMI270 的 0x0B 不是这个语义，所以只在兼容件
	//   分支里读它（见 BMI270::CheckCompatibleDataStat）。
	DATA_STAT          = 0x0B,

	// ★ rm2027/fmu-h743 板级补丁（2026-09-30）：数据寄存器 0x0C~0x17。
	// 本板兼容件（华轩阳 HXY「ICM-42688P」）的 FIFO 在 0x1C~0x21（与 BMI270 的 0x46~0x49/
	// 0x24~0x26 不同地址），驱动当前用"查询方式"定时一次性突发读这 12 个字节
	// （见 BMI270.cpp 的 STATE::DATA_READ）。
	// 官方 BMI270 手册写的是 LSB 在前（0x0C=ACC_X_LSB、0x12=GYR_X_LSB），
	// 【但本板实测是高字节在前（MSB-first）】，厂商手册也把 0x0C 命名为 ACC_XH（高字节在前），
	// 实测过程与数据见 work/spi-runtime/实现-轮询读数据寄存器.md 与 BMI270.hpp 里的常量注释。
	DATA_0             = 0x0C, // ACC_X MSB（实测）
	DATA_1             = 0x0D, // ACC_X LSB
	DATA_2             = 0x0E, // ACC_Y MSB
	DATA_3             = 0x0F, // ACC_Y LSB
	DATA_4             = 0x10, // ACC_Z MSB
	DATA_5             = 0x11, // ACC_Z LSB
	DATA_6             = 0x12, // GYR_X MSB
	DATA_7             = 0x13, // GYR_X LSB
	DATA_8             = 0x14, // GYR_Y MSB
	DATA_9             = 0x15, // GYR_Y LSB
	DATA_10            = 0x16, // GYR_Z MSB
	DATA_11            = 0x17, // GYR_Z LSB

	EVENT	           = 0x1B,

	INTERNAL_STATUS    = 0X21,
	TEMP_MSB           = 0x22,
	TEMP_LSB           = 0x23,
	FIFO_LENGTH_0      = 0x24,
	FIFO_LENGTH_1      = 0x25,
	FIFO_DATA          = 0x26,

	ACC_CONF           = 0x40,
	ACC_RANGE          = 0x41,
	GYR_CONF           = 0x42,

	FIFO_DOWNS         = 0x45,
	FIFO_WTM_0         = 0x46,
	FIFO_WTM_1         = 0x47,
	FIFO_CONFIG_0      = 0x48,
	FIFO_CONFIG_1      = 0x49,

	// controls interrupt pin behaviour
	INT1_IO_CTRL       = 0x53,

	INT_MAP_DATA       = 0x58,
	CONFIG1            = 0x59,

	CONFIG2            = 0x5E,
	PWR_CONF           = 0x7C,
	PWR_CTRL           = 0x7D,
	CMD                = 0x7E,



};

// ★ rm2027/fmu-h743（2026-09-30）：DATA_STAT(0x0B) 的位定义 —— 本片（华轩阳 HXY「ICM-42688P」）
// 特有，见厂商手册 §5.8。两个 *_CONF_ERR 位是芯片在说"你给我的配置我不接受"（配错 ODR/BWP
// 时它会自动回落到缺省值并置位，见手册 §5.37/§5.39），所以是判断"配置到底生效没有"的第一手证据。
// ⚠️ 只用于兼容件分支：正品 BMI270 的 0x0B 不是这个寄存器。
enum DATA_STAT_BIT : uint8_t {
	gyr_conf_err = Bit5, ///< 陀螺仪配置错误（默认 0）
	acc_conf_err = Bit4, ///< 加速度计配置错误（默认 0）
	drdy_tmp     = Bit2, ///< 温度数据转换完成
	drdy_gyr     = Bit1, ///< 陀螺仪三轴新数据就绪
	drdy_acc     = Bit0, ///< 加速度计三轴新数据就绪
};

enum GYR_CONF_BIT : uint8_t {
	// 1.6 or 3.2kHz sample rate
	gyr_odr_1k6 = Bit3 | Bit2,
	gyr_odr_3k2 = Bit3 | Bit2 | Bit0,
	// set LPF to normal mode
	gyr_flt_mode_normal = Bit5,
	// set noise profile to high performance mode
	gyr_noise_hp = Bit6,
	// set filter profile to high performance mode
	gyr_flt_hp = Bit7
};

enum PWR_CTRL_BIT : uint8_t {
	gyr_en = Bit1,
	accel_en = Bit2,
	temp_en = Bit3,
};

// ACC_CONF
enum ACC_CONF_BIT : uint8_t {
	// [7:4] acc_bwp
	acc_bwp_Normal = Bit7 | Bit5,        // Filter setting normal

	// [3:0] acc_odr
	acc_odr_1600   = Bit3 | Bit2,        // ODR 1600 Hz
};

// ACC_RANGE
enum ACC_RANGE_BIT : uint8_t {
	acc_range_2g  = 0,           // ±2g
	acc_range_4g  = Bit0,        // ±4g
	acc_range_8g = Bit1,        // ±8g
	acc_range_16g = Bit1 | Bit0, // ±16g
};

// FIFO_CONFIG_0
enum FIFO_CONFIG_0_BIT : uint8_t {
	BIT1_ALWAYS = Bit1, // This bit must always be ‘1’.
	FIFO_mode   = Bit0,
};

// FIFO_CONFIG_1
enum FIFO_CONFIG_1_BIT : uint8_t {
	Acc_en      = Bit6,
	Gyr_en      = Bit7,
	BIT4_ALWAYS = Bit4, // This bit must always be ‘1’.
};

// INT1_IO_CONF
enum INT1_IO_CONF_BIT : uint8_t {
	int1_in  = Bit4,
	int1_out = Bit3,

	int1_lvl = Bit1,
};

// INT1_INT2_MAP_DATA
enum INT1_INT2_MAP_DATA_BIT : uint8_t {
	int2_drdy  = Bit6,
	int2_fwm   = Bit5,
	int2_ffull = Bit4,

	int1_drdy  = Bit2,
	int1_fwm   = Bit1,
	int1_ffull = Bit0,
};

// ACC_PWR_CONF
enum ACC_PWR_CONF_BIT : uint8_t {
	acc_pwr_save = 0x03
};

// ACC_PWR_CTRL
enum ACC_PWR_CTRL_BIT : uint8_t {
	acc_enable = 0x04
};


namespace FIFO
{
static constexpr size_t SIZE = 1024;
static constexpr size_t HEADER_SIZE = 1; // one header byte precedes each accel+gyro frame

struct Data {
	uint8_t x_lsb;
	uint8_t x_msb;
	uint8_t y_lsb;
	uint8_t y_msb;
	uint8_t z_lsb;
	uint8_t z_msb;
};


enum class Header : uint8_t {
	sensor_accel_frame          = 0b10000100,
	sensor_gyro_frame           = 0b10001000,
	skip_frame                  = 0b01000000,
	sensor_time_frame           = 0b01000100,
	FIFO_input_config_frame     = 0b01001000,
	sample_drop_frame           = 0b01010000,
};

} // namespace FIFO

} // namespace Bosch_BMI270
