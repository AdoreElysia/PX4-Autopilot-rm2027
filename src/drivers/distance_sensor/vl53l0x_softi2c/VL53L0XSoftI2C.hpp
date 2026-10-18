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
 * @file VL53L0XSoftI2C.hpp
 * @brief VL53L0X 激光测距 —— 走**软件 I²C**（板子没有把硬件 I²C 引到插座上）。
 *
 * 与上游驱动 `src/drivers/distance_sensor/vl53l0x/` 的关系：
 *   两者**同一个硅片、同一套 ST 官方寄存器序列**（本文件的初始化序列逐条照抄
 *   上游 VL53L0X.cpp，那份来自 ST API / UM2039）。
 *   唯一区别是"怎么把字节送出去"：上游走硬件 I²C 外设（I2CSPIDriver + transfer()），
 *   这里走两根 GPIO 自己拉时序（SoftI2C）。寄存器地址、magic 值、判据完全一致。
 *
 * 发布：
 *   `distance_sensor`（经 PX4Rangefinder 库），device_type 沿用
 *   DRV_DIST_DEVTYPE_VL53L0X，所以下游（mavlink / EKF2 / logger）看到的和
 *   上游驱动没有区别。设备号里 bus_type 标成 I2C、bus=7 只是**借用**一个编号
 *   （软件 I²C 不是任何一条硬件总线），见 vl53l0x_softi2c_main.cpp 的说明。
 *
 * ⚠️ 临时引脚：SCL/SDA 两个脚由板级 board_config.h 指定
 *    （BOARD_VL53L0X_SOFTI2C_SCL_PINSET / _SDA_PINSET），当前借用 U18(UART7) 座。
 *    等硬件改版留出真正的 I²C 接口后，本驱动整体可以删掉、改用上游 vl53l0x 驱动。
 */

#pragma once

#include "SoftI2C.hpp"

#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <lib/conversion/rotation.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

/* ★ SWD 可读状态块（本板无串口时的观测手段，与 src/modules/rc_throttle 的
 *   rm2027_rc_status 同一套路）：布局固定、无指针，用 nm 找到符号地址后直接读内存。
 *
 *   取地址：
 *     arm-none-eabi-nm build/rm2027_fmu-h743_default/rm2027_fmu-h743_default.elf \
 *         | grep rm2027_vl53l0x_status
 *   读（openocd 的 mdw 在 ST-Link 上无效，用 gdb）：
 *     gdb-multiarch -q -batch -ex "target remote :3333" -ex "monitor halt" \
 *       -ex "x/20wx <地址>" -ex "monitor resume" -ex "detach"
 *
 *   字段索引（按 32 位字从 0 数）：0 magic('VL0X') 1 version 2 running 3 uptime_ms
 *     4 init_ok 5 model_id 6 bus_errors 7 bus_nacks 8 bus_bytes 9 published
 *     10 last_dist_mm 11 last_range_status 12 lines_ok 13 scl_high 14 sda_high 15 ack_ok
 */
extern "C" {
struct Vl53l0xSoftI2cStatus {
	uint32_t magic;
	uint32_t version;
	uint32_t running;
	uint32_t uptime_ms;
	uint32_t init_ok;
	uint32_t model_id;
	uint32_t bus_errors;
	uint32_t bus_nacks;
	uint32_t bus_bytes;
	uint32_t published;
	uint32_t last_dist_mm;
	uint32_t last_range_status;
	uint32_t lines_ok;
	uint32_t scl_high;
	uint32_t sda_high;
	uint32_t ack_ok;
};
extern volatile Vl53l0xSoftI2cStatus rm2027_vl53l0x_status;
}

class VL53L0XSoftI2C : public px4::ScheduledWorkItem
{
public:
	/** VL53L0X 出厂 7bit 地址（手册 §4.2 参考寄存器 / 上游驱动同值） */
	static constexpr uint8_t  I2C_ADDRESS = 0x29;

	/** 型号识别寄存器与期望值（DS11555 Rev6 §4.2） */
	static constexpr uint8_t  REG_MODEL_ID = 0xC0;
	static constexpr uint8_t  MODEL_ID_VL53L0X = 0xEE;

	/** 单次测量周期：50 ms = 20 Hz（上游驱动也是 50 ms） */
	static constexpr uint32_t SAMPLE_PERIOD_US = 50000;

	/** 单次测距内部轮询间隔与总超时。
	 *  轮询一次 = 一次复合传输（写寄存器号 + 重复 START + 读 1 字节）≈ 0.36 ms @100 kHz，
	 *  所以轮询间隔不要太小：测量本身约 33 ms，10 ms 粒度已经够（约 4 次轮询）。 */
	static constexpr uint32_t POLL_INTERVAL_US = 10000;
	static constexpr uint32_t MEASUREMENT_TIMEOUT_US = 150000;

	/** 初始化失败后的重试间隔（模块断电恢复后靠这个自愈，不需要控制台） */
	static constexpr uint32_t RETRY_INTERVAL_US = 15 * 1000000;

	VL53L0XSoftI2C(rm2027::SoftI2C &bus, uint32_t device_id, enum Rotation rotation);
	~VL53L0XSoftI2C() override;

	/** 完整初始化（阻塞，约 0.3~0.6 s）+ 开始周期调度。返回 PX4_OK / PX4_ERROR */
	int  Start();

	/** 停止调度（对象保留，避免 delete 与工作队列竞争） */
	void Stop();

	void PrintStatus();

	uint8_t model_id() const { return _model_id; }

private:
	void Run() override;

	/** 把当前状态写进 rm2027_vl53l0x_status（SWD 可读） */
	void update_status(uint32_t init_ok, uint32_t ack_ok);

	/** 读一次两根线的实际电平（只读，不驱动总线） */
	void sample_pins();

	int  init_sensor();
	int  start_single_measurement();
	int  collect();
	int  range_status_to_quality(uint8_t range_status) const;

	/* 寄存器读写：全部经软件 I²C */
	int read_register(uint8_t reg, uint8_t &value);
	int write_register(uint8_t reg, uint8_t value);
	int read_register_multi(uint8_t reg, uint8_t *value, uint8_t length);
	int write_register_multi(uint8_t reg, const uint8_t *value, uint8_t length);

	/* ST 官方初始化三步（照抄上游 VL53L0X.cpp） */
	int spad_calculations();
	int sensor_tuning();
	int single_ref_calibration(uint8_t byte);

	rm2027::SoftI2C &_bus;
	const uint32_t   _device_id;
	const enum Rotation _rotation;

	PX4Rangefinder _px4_rangefinder;

	uint8_t _stop_variable{0};
	uint8_t _model_id{0};

	bool        _sensor_ready{false};
	uint32_t    _init_attempts{0};
	bool        _measurement_started{false};
	hrt_abstime _measurement_start{0};
	hrt_abstime _cycle_start{0};

	perf_counter_t _perf_errors;
	perf_counter_t _perf_samples;
	perf_counter_t _perf_timeouts;

	uint32_t _scl_pin_high{0};
	uint32_t _sda_pin_high{0};
	uint32_t _published{0};
	uint32_t _last_range_status{0xFF};
	float    _last_distance_m{-1.f};
	bool     _first_sample_logged{false};
};
