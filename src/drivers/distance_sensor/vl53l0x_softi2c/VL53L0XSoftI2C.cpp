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
 * @file VL53L0XSoftI2C.cpp
 *
 * 初始化序列 = ST 官方 API（UM2039）的寄存器序列，逐条照抄自上游 PX4 驱动
 * `src/drivers/distance_sensor/vl53l0x/VL53L0X.cpp`（同一颗芯片、同一份 magic 值）。
 * 本文件的**新增**部分只有三处：
 *   ① 通信层换成 SoftI2C（软件 I²C）；
 *   ② 初始化前先读型号寄存器 0xC0 校验 = 0xEE（上游驱动不校验 ID，
 *      这正是当年"把 IST8310 认成 AF9838"那类事故的成因，见板级 README 的教训）；
 *   ③ 读结果后显式清中断（ST API 的 ClearInterruptMask），避免下一轮读到旧数据。
 *
 * ⚠️ 所有运行期打印一律 ASCII：PX4 nsh 控制台不显示 UTF-8 中文（见交接文档§9.7）。
 */

#include "VL53L0XSoftI2C.hpp"

#include <math.h>

#include <lib/mathlib/math/Limits.hpp>
#include <px4_platform_common/log.h>

using namespace time_literals;

/* ------------------------------------------------------------------------- */
/* VL53L0X 寄存器（与上游驱动同名同值）                                        */
/* ------------------------------------------------------------------------- */
#define REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HW   0x89
#define REG_MSRC_CONFIG_CONTROL                0x60
#define REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define REG_SYSTEM_SEQUENCE_CONFIG             0x01
#define REG_DYNAMIC_SPAD_REF_EN_START_OFFSET   0x4F
#define REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD 0x4E
#define REG_GLOBAL_CONFIG_REF_EN_START_SELECT  0xB6
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0   0xB0
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO       0x0A
#define REG_SYSRANGE_START                     0x00
#define REG_RESULT_INTERRUPT_STATUS            0x13
#define REG_SYSTEM_INTERRUPT_CLEAR             0x0B
#define REG_GPIO_HV_MUX_ACTIVE_HIGH            0x84
#define REG_RESULT_RANGE_STATUS                0x14

/* 结果块：0x14 起连续 12 字节，距离在最后两字节 */
#define RESULT_BLOCK_LEN                       12

/* 测距状态（上游同名宏）：0=有效，11=量程完成 */
#define RANGE_STATUS_MASK                      0x78
#define RANGE_STATUS_SHIFT                     3
#define RANGE_STATUS_VALID                     0
#define RANGE_STATUS_RANGECOMPLETE             11


/* SWD 可读状态块（声明在 hpp 里）。magic = 'VL0X'
 * ⚠️ 这里只能写成 extern "C" { ... } 的**定义**，不能写 `extern "C" 类型 名 = {...}`
 *    —— 后者在 C++ 里是 "initialized and declared extern"，本工程 -Werror 会直接拦下。 */
extern "C" {
	volatile Vl53l0xSoftI2cStatus rm2027_vl53l0x_status = {
		0x564C3058,   /* magic = 'VL0X' */
		1,            /* version */
	};
}

void VL53L0XSoftI2C::sample_pins()
{
	_scl_pin_high = _bus.scl_level() ? 1 : 0;
	_sda_pin_high = _bus.sda_level() ? 1 : 0;
}

void VL53L0XSoftI2C::update_status(uint32_t init_ok, uint32_t ack_ok)
{
	rm2027_vl53l0x_status.magic = 0x564C3058;
	rm2027_vl53l0x_status.version = 1;
	rm2027_vl53l0x_status.uptime_ms = static_cast<uint32_t>(hrt_absolute_time() / 1000);
	rm2027_vl53l0x_status.init_ok = init_ok;
	rm2027_vl53l0x_status.model_id = _model_id;
	rm2027_vl53l0x_status.bus_errors = _bus.errors();
	rm2027_vl53l0x_status.bus_nacks = _bus.nacks();
	rm2027_vl53l0x_status.bus_bytes = _bus.bytes();
	rm2027_vl53l0x_status.published = _published;
	rm2027_vl53l0x_status.last_range_status = _last_range_status;
	rm2027_vl53l0x_status.scl_high = _scl_pin_high;
	rm2027_vl53l0x_status.sda_high = _sda_pin_high;
	rm2027_vl53l0x_status.ack_ok = ack_ok;

	if (_last_distance_m >= 0.f) {
		rm2027_vl53l0x_status.last_dist_mm = static_cast<uint32_t>(_last_distance_m * 1000.f + 0.5f);
	}
}

VL53L0XSoftI2C::VL53L0XSoftI2C(rm2027::SoftI2C &bus, uint32_t device_id, enum Rotation rotation) :
	/* 放在 wq:I2C3 上：本板根本没有 I2C3 外设，所以这条队列只跑本驱动一个任务，
	 * 软件 I²C 的忙等（约 3 ms / 50 ms 周期）不会去挤别的模块。
	 * 对照：lp_default 上已经有 8 个模块，priority 还是 -50（更低），不合适。 */
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::I2C3),
	_bus(bus),
	_device_id(device_id),
	_rotation(rotation),
	_px4_rangefinder(device_id, rotation)
{
	/* VL53L0X 典型量程 0~2 m、视场角 25°（与上游驱动一致） */
	_px4_rangefinder.set_min_distance(0.f);
	_px4_rangefinder.set_max_distance(2.f);
	_px4_rangefinder.set_fov(math::radians(25.f));
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_VL53L0X);

	_perf_errors   = perf_alloc(PC_COUNT, MODULE_NAME": i2c errors");
	_perf_samples  = perf_alloc(PC_COUNT, MODULE_NAME": samples");
	_perf_timeouts = perf_alloc(PC_COUNT, MODULE_NAME": timeouts");
}

VL53L0XSoftI2C::~VL53L0XSoftI2C()
{
	perf_free(_perf_errors);
	perf_free(_perf_samples);
	perf_free(_perf_timeouts);
}

/* ------------------------------------------------------------------------- */
/* 寄存器读写                                                                 */
/* ------------------------------------------------------------------------- */

int VL53L0XSoftI2C::read_register(uint8_t reg, uint8_t &value)
{
	return read_register_multi(reg, &value, 1);
}

int VL53L0XSoftI2C::read_register_multi(uint8_t reg, uint8_t *value, uint8_t length)
{
	if (_bus.write_read(I2C_ADDRESS, &reg, 1, value, length) != 0) {
		perf_count(_perf_errors);
		return PX4_ERROR;
	}

	return PX4_OK;
}

int VL53L0XSoftI2C::write_register(uint8_t reg, uint8_t value)
{
	return write_register_multi(reg, &value, 1);
}

int VL53L0XSoftI2C::write_register_multi(uint8_t reg, const uint8_t *value, uint8_t length)
{
	if (length > 6) {
		return PX4_ERROR;
	}

	uint8_t cmd[7] {};
	cmd[0] = reg;
	memcpy(&cmd[1], value, length);

	if (_bus.write(I2C_ADDRESS, cmd, length + 1) != 0) {
		perf_count(_perf_errors);
		return PX4_ERROR;
	}

	return PX4_OK;
}

/* ------------------------------------------------------------------------- */
/* 初始化                                                                     */
/* ------------------------------------------------------------------------- */

int VL53L0XSoftI2C::single_ref_calibration(const uint8_t byte)
{
	uint8_t val = 0;

	if (write_register(REG_SYSRANGE_START, byte | 0x01) != PX4_OK) { return PX4_ERROR; }

	if (read_register(REG_RESULT_INTERRUPT_STATUS, val) != PX4_OK) { return PX4_ERROR; }

	hrt_abstime t0 = hrt_absolute_time();

	while ((val & 0x07) == 0) {
		if (hrt_elapsed_time(&t0) > 1_s) {
			PX4_ERR("ref calibration timeout");
			return PX4_ERROR;
		}

		if (read_register(REG_RESULT_INTERRUPT_STATUS, val) != PX4_OK) { return PX4_ERROR; }
	}

	if (write_register(REG_SYSTEM_INTERRUPT_CLEAR, 0x01) != PX4_OK) { return PX4_ERROR; }

	return write_register(REG_SYSRANGE_START, 0x00);
}

int VL53L0XSoftI2C::sensor_tuning()
{
	/* magic 值全部来自 ST API（照抄上游驱动） */
	const uint8_t seq[][2] = {
		{0xFF, 0x01}, {0x00, 0x00}, {0xFF, 0x00}, {0x09, 0x00}, {0x10, 0x00}, {0x11, 0x00},
		{0x24, 0x01}, {0x25, 0xFF}, {0x75, 0x00}, {0xFF, 0x01}, {0x4E, 0x2C}, {0x48, 0x00},
		{0x30, 0x20}, {0xFF, 0x00}, {0x30, 0x09}, {0x54, 0x00}, {0x31, 0x04}, {0x32, 0x03},
		{0x40, 0x83}, {0x46, 0x25}, {0x60, 0x00}, {0x27, 0x00}, {0x50, 0x06}, {0x51, 0x00},
		{0x52, 0x96}, {0x56, 0x08}, {0x57, 0x30}, {0x61, 0x00}, {0x62, 0x00}, {0x64, 0x00},
		{0x65, 0x00}, {0x66, 0xA0}, {0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14}, {0x49, 0xFF},
		{0x4A, 0x00}, {0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00}, {0x78, 0x21}, {0xFF, 0x01},
		{0x23, 0x34}, {0x42, 0x00}, {0x44, 0xFF}, {0x45, 0x26}, {0x46, 0x05}, {0x40, 0x40},
		{0x0E, 0x06}, {0x20, 0x1A}, {0x43, 0x40}, {0xFF, 0x00}, {0x34, 0x03}, {0x35, 0x44},
		{0xFF, 0x01}, {0x31, 0x04}, {0x4B, 0x09}, {0x4C, 0x05}, {0x4D, 0x04}, {0xFF, 0x00},
		{0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08}, {0x48, 0x28}, {0x67, 0x00}, {0x70, 0x04},
		{0x71, 0x01}, {0x72, 0xFE}, {0x76, 0x00}, {0x77, 0x00}, {0xFF, 0x01}, {0x0D, 0x01},
		{0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8}, {0xFF, 0x01}, {0x8E, 0x01}, {0x00, 0x01},
		{0xFF, 0x00}, {0x80, 0x00},
	};

	for (const auto &kv : seq) {
		if (write_register(kv[0], kv[1]) != PX4_OK) {
			PX4_ERR("tuning write 0x%02x failed", kv[0]);
			return PX4_ERROR;
		}
	}

	return PX4_OK;
}

int VL53L0XSoftI2C::spad_calculations()
{
	uint8_t val = 0;
	uint8_t ref_spad_map[6] = {};

	if (write_register(0x80, 0x01) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0xFF, 0x01) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0x00, 0x00) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0xFF, 0x06) != PX4_OK) { return PX4_ERROR; }

	if (read_register(0x83, val) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0x83, val | 0x04) != PX4_OK) { return PX4_ERROR; }

	if (write_register(0xFF, 0x07) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0x81, 0x01) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0x80, 0x01) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0x94, 0x6b) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0x83, 0x00) != PX4_OK) { return PX4_ERROR; }

	/* SPAD 就绪轮询（上游是无超时的死等；这里加 1 s 超时，避免硬件异常把任务卡死） */
	hrt_abstime t0 = hrt_absolute_time();

	if (read_register(0x83, val) != PX4_OK) { return PX4_ERROR; }

	while (val == 0x00) {
		if (hrt_elapsed_time(&t0) > 1_s) {
			PX4_ERR("SPAD info timeout");
			return PX4_ERROR;
		}

		if (read_register(0x83, val) != PX4_OK) { return PX4_ERROR; }
	}

	if (write_register(0x83, 0x01) != PX4_OK) { return PX4_ERROR; }
	if (read_register(0x92, val) != PX4_OK) { return PX4_ERROR; }

	const uint8_t spad_count = val & 0x7f;
	const bool spad_type_is_aperture = (val >> 7) & 0x01;

	if (write_register(0x81, 0x00) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0xFF, 0x06) != PX4_OK) { return PX4_ERROR; }

	if (read_register(0x83, val) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0x83, val & ~0x04) != PX4_OK) { return PX4_ERROR; }

	if (write_register(0xFF, 0x01) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0x00, 0x01) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0xFF, 0x00) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0x80, 0x00) != PX4_OK) { return PX4_ERROR; }

	if (read_register_multi(REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6) != PX4_OK) { return PX4_ERROR; }

	if (write_register(0xFF, 0x01) != PX4_OK) { return PX4_ERROR; }
	if (write_register(REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00) != PX4_OK) { return PX4_ERROR; }
	if (write_register(REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0xFF, 0x00) != PX4_OK) { return PX4_ERROR; }
	if (write_register(REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4) != PX4_OK) { return PX4_ERROR; }

	const uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;
	uint8_t spads_enabled = 0;

	for (uint8_t i = 0; i < 48; i++) {
		if (i < first_spad_to_enable || spads_enabled == spad_count) {
			ref_spad_map[i / 8] &= ~(1 << (i % 8));

		} else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
			spads_enabled++;
		}
	}

	if (write_register_multi(REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6) != PX4_OK) { return PX4_ERROR; }

	if (sensor_tuning() != PX4_OK) { return PX4_ERROR; }

	if (write_register(REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 4) != PX4_OK) { return PX4_ERROR; }   /* 4 = 新数据中断 */

	if (read_register(REG_GPIO_HV_MUX_ACTIVE_HIGH, val) != PX4_OK) { return PX4_ERROR; }
	if (write_register(REG_GPIO_HV_MUX_ACTIVE_HIGH, val & ~0x10) != PX4_OK) { return PX4_ERROR; }   /* 低有效 */

	if (write_register(REG_SYSTEM_INTERRUPT_CLEAR, 0x01) != PX4_OK) { return PX4_ERROR; }
	if (write_register(REG_SYSTEM_SEQUENCE_CONFIG, 0xE8) != PX4_OK) { return PX4_ERROR; }
	if (write_register(REG_SYSTEM_SEQUENCE_CONFIG, 0x01) != PX4_OK) { return PX4_ERROR; }

	if (single_ref_calibration(0x40) != PX4_OK) { return PX4_ERROR; }

	if (write_register(REG_SYSTEM_SEQUENCE_CONFIG, 0x02) != PX4_OK) { return PX4_ERROR; }

	if (single_ref_calibration(0x00) != PX4_OK) { return PX4_ERROR; }

	if (write_register(REG_SYSTEM_SEQUENCE_CONFIG, 0xE8) != PX4_OK) { return PX4_ERROR; }   /* 恢复配置 */

	return PX4_OK;
}

int VL53L0XSoftI2C::init_sensor()
{
	uint8_t val = 0;

	/* ① 型号校验：先确认"是谁"，再谈初始化。上游驱动不校验 ID —— 历史教训见板级 README */
	if (read_register(REG_MODEL_ID, _model_id) != PX4_OK) {
		PX4_ERR("no answer at 0x%02x (check wiring: SCL/SDA/VIN/GND)", I2C_ADDRESS);
		return PX4_ERROR;
	}

	if (_model_id != MODEL_ID_VL53L0X) {
		PX4_ERR("model id at 0xC0 = 0x%02x, expected VL53L0X id 0xEE", _model_id);
		PX4_ERR("(if the module is actually VL53L1X, this driver cannot drive it:");
		PX4_ERR(" VL53L0X and VL53L1X are register/API incompatible - see docs)");
		return PX4_ERROR;
	}

	/* ② ST 初始化：以下逐条照抄上游驱动（= UM2039 的参考序列） */
	if (read_register(REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HW, val) != PX4_OK) { return PX4_ERROR; }
	if (write_register(REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HW, val | 0x01) != PX4_OK) { return PX4_ERROR; }

	/* I²C 设为标准模式（2.8 V 侧） */
	if (write_register(0x88, 0x00) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0x80, 0x01) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0xFF, 0x01) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0x00, 0x00) != PX4_OK) { return PX4_ERROR; }
	if (read_register(0x91, val) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0x00, 0x01) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0xFF, 0x00) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0x80, 0x00) != PX4_OK) { return PX4_ERROR; }

	_stop_variable = val;

	/* 关掉 SIGNAL_RATE_MSRC(bit1) / SIGNAL_RATE_PRE_RANGE(bit4) 限制检查 */
	if (read_register(REG_MSRC_CONFIG_CONTROL, val) != PX4_OK) { return PX4_ERROR; }
	if (write_register(REG_MSRC_CONFIG_CONTROL, val | 0x12) != PX4_OK) { return PX4_ERROR; }

	/* 信号率阈值 0.1 MCPS */
	const float rate_limit = 0.1f * 65536.f;
	const uint8_t rate_limit_split[2] = {
		static_cast<uint8_t>(static_cast<uint16_t>(rate_limit) >> 8),
		static_cast<uint8_t>(static_cast<uint16_t>(rate_limit))
	};

	if (write_register_multi(REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, rate_limit_split, 2) != PX4_OK) { return PX4_ERROR; }
	if (write_register(REG_SYSTEM_SEQUENCE_CONFIG, 0xFF) != PX4_OK) { return PX4_ERROR; }

	return spad_calculations();
}

/* ------------------------------------------------------------------------- */
/* 周期测量                                                                   */
/* ------------------------------------------------------------------------- */

int VL53L0XSoftI2C::start_single_measurement()
{
	/* 触发一次单次（single shot）测距：先按 ST API 写 stop_variable 那一串 */
	if (write_register(0x80, 0x01) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0xFF, 0x01) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0x00, 0x00) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0x91, _stop_variable) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0x00, 0x01) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0xFF, 0x00) != PX4_OK) { return PX4_ERROR; }
	if (write_register(0x80, 0x00) != PX4_OK) { return PX4_ERROR; }

	return write_register(REG_SYSRANGE_START, 0x01);
}

int VL53L0XSoftI2C::range_status_to_quality(uint8_t range_status) const
{
	return (range_status == RANGE_STATUS_VALID || range_status == RANGE_STATUS_RANGECOMPLETE) ? 100 : 0;
}

int VL53L0XSoftI2C::collect()
{
	uint8_t val[RESULT_BLOCK_LEN] {};

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (read_register_multi(REG_RESULT_RANGE_STATUS, val, RESULT_BLOCK_LEN) != PX4_OK) {
		return PX4_ERROR;
	}

	const uint8_t range_status = (val[0] & RANGE_STATUS_MASK) >> RANGE_STATUS_SHIFT;
	const uint16_t distance_mm = (static_cast<uint16_t>(val[10]) << 8) | val[11];
	const float distance_m = distance_mm / 1000.f;

	/* 清中断标志（ST API: ClearInterruptMask）—— 不清的话下一轮会立刻"就绪"读到旧值 */
	if (write_register(REG_SYSTEM_INTERRUPT_CLEAR, 0x01) != PX4_OK) {
		return PX4_ERROR;
	}

	_last_range_status = range_status;
	_last_distance_m = distance_m;

	_px4_rangefinder.update(timestamp_sample, distance_m, range_status_to_quality(range_status));

	perf_count(_perf_samples);
	_published++;

	if (!_first_sample_logged) {
		_first_sample_logged = true;
		PX4_INFO("first sample: %u mm (range_status=%u, quality=%d)",
			 static_cast<unsigned>(distance_mm), static_cast<unsigned>(range_status),
			 range_status_to_quality(range_status));
	}

	return PX4_OK;
}

void VL53L0XSoftI2C::Run()
{
	/* 初始化没成功（模块没电 / 线断 / 被我之前的调试动作搞卡）→ 定时重试。
	 * 放在本驱动独占的 wq:I2C3 上，所以这段重初始化不会挤到别的模块。 */
	if (!_sensor_ready) {
		_init_attempts++;

		if (_bus.init() == 0 && init_sensor() == PX4_OK) {
			_sensor_ready = true;
			PX4_INFO("sensor init OK after %u retries", static_cast<unsigned>(_init_attempts));
			_init_attempts = 0;

		} else {
			sample_pins();
			update_status(0, 0);
			ScheduleDelayed(RETRY_INTERVAL_US);
			return;
		}
	}

	if (!_measurement_started) {
		_cycle_start = hrt_absolute_time();

		if (start_single_measurement() != PX4_OK) {
			/* 通信坏了：退避 100 ms 再试，不把工作队列刷爆 */
			ScheduleDelayed(100_ms);
			return;
		}

		_measurement_started = true;
		_measurement_start = _cycle_start;
		ScheduleDelayed(POLL_INTERVAL_US);
		return;
	}

	uint8_t int_status = 0;

	if (read_register(REG_RESULT_INTERRUPT_STATUS, int_status) != PX4_OK) {
		_measurement_started = false;
		ScheduleDelayed(100_ms);
		return;
	}

	if ((int_status & 0x07) == 0) {
		/* 还没测完 */
		if (hrt_elapsed_time(&_measurement_start) > MEASUREMENT_TIMEOUT_US) {
			perf_count(_perf_timeouts);
			_measurement_started = false;
			ScheduleDelayed(SAMPLE_PERIOD_US);
			return;
		}

		ScheduleDelayed(POLL_INTERVAL_US);
		return;
	}

	collect();
	_measurement_started = false;
	sample_pins();
	update_status(1, 1);

	/* 把周期补满（测量本身耗时 ~33 ms） */
	const hrt_abstime elapsed = hrt_elapsed_time(&_cycle_start);

	if (elapsed < SAMPLE_PERIOD_US) {
		ScheduleDelayed(SAMPLE_PERIOD_US - elapsed);

	} else {
		ScheduleDelayed(1_ms);
	}
}

/* ------------------------------------------------------------------------- */
/* 生命周期                                                                   */
/* ------------------------------------------------------------------------- */

int VL53L0XSoftI2C::Start()
{
	/* 允许 stop → start 反复切：把测量状态清干净，避免停在半途的状态被继承 */
	_measurement_started = false;
	_measurement_start = 0;
	_cycle_start = 0;

	/* 先把两条线的实际电平记下来 —— 这是"模块有没有上电/线有没有断"的第一手证据 */
	sample_pins();
	rm2027_vl53l0x_status.lines_ok = 0;
	update_status(0, 0);

	if (_bus.init() != 0) {
		PX4_ERR("software I2C bus not idle-high: check the 4 wires / module power");
		PX4_ERR("(SCL/SDA need the module's onboard pull-ups; this driver does not");
		PX4_ERR(" enable internal pull-ups on purpose - see SoftI2C.hpp)");
		sample_pins();
		update_status(0, 0);
		/* 不放弃：交给 Run() 每 15 s 重试一次（模块断电恢复后能自愈） */
		_sensor_ready = false;
		ScheduleDelayed(RETRY_INTERVAL_US);
		return PX4_ERROR;
	}


	const hrt_abstime t0 = hrt_absolute_time();

	if (init_sensor() != PX4_OK) {
		PX4_ERR("sensor init failed after %u ms; will retry every %u s",
			static_cast<unsigned>(hrt_elapsed_time(&t0) / 1000),
			static_cast<unsigned>(RETRY_INTERVAL_US / 1000000));
		sample_pins();
		update_status(0, 0);
		_sensor_ready = false;
		ScheduleDelayed(RETRY_INTERVAL_US);
		return PX4_ERROR;
	}

	_sensor_ready = true;

	rm2027_vl53l0x_status.lines_ok = 1;
	rm2027_vl53l0x_status.running = 1;
	sample_pins();
	update_status(1, 1);

	PX4_INFO("VL53L0X ready: model id 0x%02x, init took %u ms, rotation %d",
		 static_cast<unsigned>(_model_id), static_cast<unsigned>(hrt_elapsed_time(&t0) / 1000),
		 static_cast<int>(_rotation));

	ScheduleNow();
	return PX4_OK;
}

void VL53L0XSoftI2C::Stop()
{
	ScheduleClear();
	_bus.bus_release();
	rm2027_vl53l0x_status.running = 0;
	sample_pins();
	update_status(rm2027_vl53l0x_status.init_ok, 0);
}

void VL53L0XSoftI2C::PrintStatus()
{
	const uint32_t bytes = _bus.bytes();
	const uint32_t per_cycle = (_published > 0) ? (bytes / _published) : 0;

	PX4_INFO("device_id: 0x%08" PRIx32, _device_id);
	PX4_INFO("model id : 0x%02x %s", static_cast<unsigned>(_model_id),
		 _model_id == MODEL_ID_VL53L0X ? "(VL53L0X)" : "(unknown)");
	PX4_INFO("published: %" PRIu32 "  last: %.3f m  range_status: %" PRIu32,
		 _published, static_cast<double>(_last_distance_m), _last_range_status);
	PX4_INFO("soft i2c : %" PRIu32 " bytes total, ~%" PRIu32 " bytes/cycle, errors=%" PRIu32 ", nacks=%" PRIu32,
		 bytes, per_cycle, _bus.errors(), _bus.nacks());
	PX4_INFO("pins     : SCL = PE8 (U18.3 'T') / SDA = PE7 (U18.2 'R')  [TEMPORARY]");

	perf_print_counter(_perf_errors);
	perf_print_counter(_perf_samples);
	perf_print_counter(_perf_timeouts);
}
