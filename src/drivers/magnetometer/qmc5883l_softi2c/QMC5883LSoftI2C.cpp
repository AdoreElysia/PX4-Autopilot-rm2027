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
 * @file QMC5883LSoftI2C.cpp
 * @brief QMC5883L 磁罗盘 + 软件 I²C 实现。
 *
 * 寄存器时序与上游 src/drivers/magnetometer/qmc5883l/QMC5883L.cpp 逐条对齐
 * （同样的 CNTL1/CNTL2/SET_RESET_PERIOD 配置、同样的 12000 LSB/Gauss 标度、
 *  同样的 X/-Y/-Z 轴向变换）。差别只在"字节怎么送出去"：
 * 上游走 device::I2C（硬件总线），这里走 rm2027::SoftI2C（GPIO bit-bang）。
 */

#include "QMC5883LSoftI2C.hpp"

#include <math.h>

using namespace QST_QMC5883L;
using namespace time_literals;

namespace rm2027
{

/* device_id 位布局（src/lib/drivers/device/Device.hpp 的 DeviceStructure）：
 *     devtype(8) | address(8) | bus(5) | bus_type(3)
 * bus_type: 1 = I2C。
 * ⚠️ bus 字段填 7 只是**为了让 device_id 唯一、便于区分**：
 *    软件 I²C 不属于任何一条硬件总线，7 不是 STM32 的 I²C 外设号。
 *    （与 vl53l0x_softi2c 的约定一致） */
static constexpr uint8_t kBusTypeI2C = 1;
static constexpr uint8_t kSoftBusId = 7;

uint32_t QMC5883LSoftI2C::make_device_id()
{
	return (static_cast<uint32_t>(DRV_MAG_DEVTYPE_QMC5883L) << 24)
	       | (static_cast<uint32_t>(I2C_ADDRESS) << 16)
	       | (static_cast<uint32_t>(kSoftBusId) << 3)
	       | kBusTypeI2C;
}

namespace
{
/* CHIP_ID 校验的重试次数：界面上电瞬间可能还没准备好，给 3 次 */
constexpr int kProbeRetries = 3;

/* 软复位后等 POR 完成。手册要求 >= 5 ms，上游驱动取 100 ms，这里一致。 */
constexpr hrt_abstime kPorDelayUs = 100_ms;

/* 连续失败多少次就整体重初始化（与上游一致） */
constexpr uint32_t kMaxFailureCount = 10;

inline int16_t combine(uint8_t msb, uint8_t lsb)
{
	return static_cast<int16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
}
} // namespace

QMC5883LSoftI2C::QMC5883LSoftI2C(SoftI2C &bus, enum Rotation rotation, bool external) :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_bus(bus),
	_px4_mag(make_device_id(), rotation, external),
	_reset_perf(perf_alloc(PC_COUNT, MODULE_NAME": reset")),
	_bad_register_perf(perf_alloc(PC_COUNT, MODULE_NAME": bad register")),
	_bad_transfer_perf(perf_alloc(PC_COUNT, MODULE_NAME": bad transfer"))
{
}

QMC5883LSoftI2C::~QMC5883LSoftI2C()
{
	ScheduleClear();
	perf_free(_reset_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
}

int QMC5883LSoftI2C::init()
{
	_px4_mag.set_device_type(DRV_MAG_DEVTYPE_QMC5883L);
	_px4_mag.set_scale(1.f / 12000.f);	// 12000 LSB/Gauss（量程 ±2 G）

	_state = State::RESET;
	_reset_timestamp = hrt_absolute_time();

	/* 注意：这里**同步**跑一次 RESET 状态，好处是 init() 返回失败时
	 * 上层能立刻知道"器件没应答"，而不是等后台调度默默超时。
	 * 软复位本身只是写一个寄存器，耗时在 100 µs 量级。 */
	if (!Reset()) {
		return PX4_ERROR;
	}

	ScheduleOnInterval(MEASUREMENT_INTERVAL_US);

	return PX4_OK;
}

bool QMC5883LSoftI2C::Reset()
{
	if (!WriteRegister(static_cast<uint8_t>(Register::CNTL2), CNTL2_BIT::SOFT_RST)) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	perf_count(_reset_perf);
	_reset_timestamp = hrt_absolute_time();
	_failure_count = 0;
	_state = State::WAIT_FOR_RESET;

	return true;
}

bool QMC5883LSoftI2C::Configure()
{
	/* 与上游 _register_cfg 完全一致的三条：
	 *   CNTL1 = ODR_50HZ | Mode_Continuous，清 OSR_512/RNG_2G（即 ±2G、512 过采样）
	 *   CNTL2 = ROL_PNT，清 SOFT_RST
	 *   SET_RESET_PERIOD = 0x01（手册明确要求）
	 */
	struct RegCfg {
		uint8_t reg;
		uint8_t set_bits;
		uint8_t clear_bits;
	};

	static constexpr RegCfg kRegCfg[] = {
		{
			static_cast<uint8_t>(Register::CNTL1),
			static_cast<uint8_t>(CNTL1_BIT::ODR_50HZ | CNTL1_BIT::Mode_Continuous),
			static_cast<uint8_t>(CNTL1_BIT::OSR_512 | CNTL1_BIT::RNG_2G)
		},
		{
			static_cast<uint8_t>(Register::CNTL2),
			static_cast<uint8_t>(CNTL2_BIT::ROL_PNT),
			static_cast<uint8_t>(CNTL2_BIT::SOFT_RST)
		},
		{
			static_cast<uint8_t>(Register::SET_RESET_PERIOD),
			static_cast<uint8_t>(SET_RESET_PERIOD_BIT::FBR), 0
		},
	};

	for (const auto &cfg : kRegCfg) {
		uint8_t orig = 0;

		if (!ReadRegisters(cfg.reg, &orig, 1)) {
			perf_count(_bad_transfer_perf);
			return false;
		}

		const uint8_t val = static_cast<uint8_t>((orig & ~cfg.clear_bits) | cfg.set_bits);

		if (orig != val && !WriteRegister(cfg.reg, val)) {
			perf_count(_bad_transfer_perf);
			return false;
		}
	}

	/* 回读校验：防止"写进去了但芯片没接住"（I²C 有 ACK 不代表寄存器真的改了） */
	for (const auto &cfg : kRegCfg) {
		uint8_t val = 0;

		if (!ReadRegisters(cfg.reg, &val, 1)) {
			perf_count(_bad_transfer_perf);
			return false;
		}

		if ((cfg.set_bits && (val & cfg.set_bits) != cfg.set_bits)
		    || (cfg.clear_bits && (val & cfg.clear_bits) != 0)) {
			perf_count(_bad_register_perf);
			return false;
		}
	}

	return true;
}

bool QMC5883LSoftI2C::ReadRegisters(uint8_t first_reg, uint8_t *buf, size_t len)
{
	return _bus.write_read(I2C_ADDRESS, &first_reg, 1, buf, len) == 0;
}

bool QMC5883LSoftI2C::WriteRegister(uint8_t reg, uint8_t value)
{
	const uint8_t payload[2] { reg, value };
	return _bus.write(I2C_ADDRESS, payload, sizeof(payload)) == 0;
}

int QMC5883LSoftI2C::read_and_publish()
{
	/* 一次读 7 字节：0x00~0x05 是三轴数据，0x06 是 STATUS。
	 * 用一次传输而不是 7 次，既是效率也是**一致性**：
	 * 分开读会拿到不同时刻的样本，三轴拼起来会抖。 */
	uint8_t buffer[7] {};

	if (!ReadRegisters(static_cast<uint8_t>(Register::X_LSB), buffer, sizeof(buffer))) {
		perf_count(_bad_transfer_perf);
		return PX4_ERROR;
	}

	const uint8_t status = buffer[6];

	if (status & STATUS_BIT::DRDY) {
		if (status & STATUS_BIT::OVL) {
			/* 溢出：数据不可信，跳过这一帧但不复位器件 */
			perf_count(_bad_register_perf);
			return PX4_ERROR;
		}

		int16_t x = combine(buffer[1], buffer[0]);
		int16_t y = combine(buffer[3], buffer[2]);
		int16_t z = combine(buffer[5], buffer[4]);

		/* 轴向对齐 PX4 的 FRD（与上游 QMC5883L.cpp:196-200 完全一致）：
		 *   Forward X := +X, Right Y := -Y, Down Z := -Z
		 * ⚠️ INT16_MIN 取反会溢出，和上游一样夹到 INT16_MAX。 */
		y = (y == INT16_MIN) ? INT16_MAX : static_cast<int16_t>(-y);
		z = (z == INT16_MIN) ? INT16_MAX : static_cast<int16_t>(-z);

		_last_publication = hrt_absolute_time();
		_px4_mag.update(_last_publication, x, y, z);

		if (_failure_count > 0) {
			_failure_count--;
		}

		return PX4_OK;
	}

	return PX4_ERROR;
}

void QMC5883LSoftI2C::Run()
{
	switch (_state) {
	case State::RESET:
		if (Reset()) {
			ScheduleDelayed(kPorDelayUs);

		} else {
			ScheduleDelayed(100_ms);
		}

		break;

	case State::WAIT_FOR_RESET: {
			/* 软复位完成的标志：CHIP_ID 能读回 0xFF，且 CNTL2 的 SOFT_RST 已自动清零 */
			uint8_t chip_id = 0;
			uint8_t cntl2 = 0;

			const bool id_ok = ReadRegisters(static_cast<uint8_t>(Register::CHIP_ID), &chip_id, 1)
					   && chip_id == Chip_ID;
			const bool rst_done = ReadRegisters(static_cast<uint8_t>(Register::CNTL2), &cntl2, 1)
					      && (cntl2 & CNTL2_BIT::SOFT_RST) == 0;

			if (id_ok && rst_done) {
				_state = State::CONFIGURE;
				ScheduleDelayed(10_ms);

			} else if (hrt_elapsed_time(&_reset_timestamp) > 1_s) {
				PX4_DEBUG("reset not complete after 1 s, retrying");
				_state = State::RESET;
				ScheduleDelayed(100_ms);

			} else {
				ScheduleDelayed(10_ms);
			}

			break;
		}

	case State::CONFIGURE:
		if (Configure()) {
			_state = State::READ;
			ScheduleOnInterval(MEASUREMENT_INTERVAL_US);

		} else if (hrt_elapsed_time(&_reset_timestamp) > 1_s) {
			PX4_DEBUG("configure failed after 1 s, resetting");
			_state = State::RESET;
			ScheduleDelayed(100_ms);

		} else {
			ScheduleDelayed(100_ms);
		}

		break;

	case State::READ:
		if (read_and_publish() != PX4_OK) {
			if (++_failure_count > kMaxFailureCount) {
				PX4_DEBUG("too many read failures, resetting");
				_state = State::RESET;
			}
		}

		break;
	}
}

void QMC5883LSoftI2C::print_status()
{
	PX4_INFO("state: %s", _state == State::READ ? "READ" :
		 (_state == State::CONFIGURE ? "CONFIGURE" :
		  (_state == State::WAIT_FOR_RESET ? "WAIT_FOR_RESET" : "RESET")));
	PX4_INFO("failure count: %" PRIu32, _failure_count);
	PX4_INFO("bus errors: %" PRIu32 " (nacks: %" PRIu32 ")",
		 _bus.errors(), _bus.nacks());
	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
}

void QMC5883LSoftI2C::error()
{
	_bus.bus_release();
}

} // namespace rm2027
