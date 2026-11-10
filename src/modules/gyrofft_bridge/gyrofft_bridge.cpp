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
 * @file gyrofft_bridge.cpp
 * @brief 把板载 gyro_fft 的峰值结果 + 当时油门打包到 debug_array
 *
 * ── 为什么需要这个模块 ────────────────────────────────────────────
 *
 * 本板 CONFIG_BOARD_NO_SDCARD=y，也没有 USB，唯一观测通道是 MAVLink 串口。
 * 而 sensor_gyro_fft 没有对应的 MAVLink 流，数据根本出不来。
 *
 * 直接传原始陀螺数据也不可行：
 *   · 陀螺原始速率实测 1997.95 Hz，HIGHRES_IMU 约 74 B/帧 ⇒ 约 148 KB/s
 *   · 921600 波特率实测只能到约 12 KB/s
 *   · PX4 的 mavlink 主循环 px4_usleep(_main_loop_delay) 把单流频率
 *     硬限在约 154 Hz ⇒ Nyquist 仅 77 Hz，看不到 167 Hz 的电机基频
 *
 * 所以让板子本地算 FFT（CMSIS-DSP，1997.95 Hz 采样，Nyquist 999 Hz），
 * 这里只把「峰值结论 + 当时的油门 + 姿态」打包发出去，
 * 带宽需求从 148 KB/s 降到约 2.5 KB/s。
 *
 * ── 调试数组字段表（array_id = 0, name = "GYROFFT"）────────────────
 *
 *   [0]      sensor_sample_rate_hz     板载 FFT 用的采样率（分辨率依赖它）
 *   [1]      resolution_hz             = sample_rate / fft_len
 *   [2]      device_id（转 float）
 *   [3..5]   peak_freq_x[0..2]         Roll 轴三个峰值（Hz）
 *   [6..8]   peak_freq_y[0..2]         Pitch 轴
 *   [9..11]  peak_freq_z[0..2]         Yaw 轴
 *   [12..14] peak_snr_x[0..2]          对应 SNR（dB）
 *   [15..17] peak_snr_y[0..2]
 *   [18..20] peak_snr_z[0..2]
 *   [21]     throttle_avg             四个电机推力均值（归一化 0..1）★
 *   [22..25] motor[0..3]              四个电机各自推力 ★
 *   [26]     roll_deg                 当时的姿态
 *   [27]     pitch_deg
 *   [28]     yaw_deg
 *   [29]     seq                      帧序号，用于检测丢帧
 *
 *   ★ 油门是必需字段：没有它无法判断峰值频率是否随转速移动。
 *     峰值随油门线性移动 ⇒ 电机/桨的转动噪声，该设陷波；
 *     峰值不随油门变    ⇒ 结构共振，含义完全不同。
 *
 * ── 用法 ──────────────────────────────────────────────────────────
 *   gyrofft_bridge start      开始发布（10 Hz）
 *   gyrofft_bridge stop
 *   gyrofft_bridge status     打印当前缓存与统计
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/sensor_gyro_fft.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/vehicle_attitude.h>

#include <drivers/drv_hrt.h>
#include <math.h>
#include <mathlib/mathlib.h>

using namespace time_literals;

namespace
{
constexpr uint16_t kArrayId     = 0;
constexpr const char *kArrayName = "GYROFFT";
constexpr hrt_abstime kPeriod   = 100_ms;   // 10 Hz，远低于带宽上限
} // namespace

class GyroFftBridge : public ModuleBase, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	GyroFftBridge();
	~GyroFftBridge() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;

private:
	void Run() override;
	void publish();

	uORB::Subscription _fft_sub{ORB_ID(sensor_gyro_fft)};
	uORB::Subscription _motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};

	uORB::Publication<debug_array_s> _debug_pub{ORB_ID(debug_array)};

	sensor_gyro_fft_s _fft{};
	actuator_motors_s  _motors{};
	vehicle_attitude_s _att{};

	bool _have_motors{false};
	bool _have_att{false};

	uint32_t _seq{0};
	uint32_t _published{0};
	uint32_t _skipped{0};
};

ModuleBase::Descriptor GyroFftBridge::desc{task_spawn, custom_command, print_usage};

GyroFftBridge::GyroFftBridge() :
	px4::ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

void GyroFftBridge::Run()
{
	// sensor_gyro_fft 用 updated() 判断：拿到才发，避免把全 0 当成"没有振动"
	if (!_fft_sub.updated()) {
		_skipped++;
		return;
	}

	if (!_fft_sub.copy(&_fft)) {
		_skipped++;
		return;
	}

	if (_motors_sub.updated()) {
		if (_motors_sub.copy(&_motors)) {
			_have_motors = true;
		}
	}

	if (_att_sub.updated()) {
		if (_att_sub.copy(&_att)) {
			_have_att = true;
		}
	}

	publish();
}

void GyroFftBridge::publish()
{
	debug_array_s dbg{};
	dbg.timestamp = hrt_absolute_time();
	dbg.id = kArrayId;
	strncpy(dbg.name, kArrayName, sizeof(dbg.name) - 1);

	// 默认 NaN：下游能区分「没有这个字段」和「值真的是 0」
	for (unsigned i = 0; i < debug_array_s::ARRAY_SIZE; i++) {
		dbg.data[i] = NAN;
	}

	// [0..2] FFT 元信息
	dbg.data[0] = _fft.sensor_sample_rate_hz;
	dbg.data[1] = _fft.resolution_hz;
	dbg.data[2] = static_cast<float>(_fft.device_id);

	// [3..11] 三轴峰值频率
	for (unsigned i = 0; i < 3; i++) {
		dbg.data[3 + i] = _fft.peak_frequencies_x[i];
		dbg.data[6 + i] = _fft.peak_frequencies_y[i];
		dbg.data[9 + i] = _fft.peak_frequencies_z[i];
	}

	// [12..20] 对应 SNR
	for (unsigned i = 0; i < 3; i++) {
		dbg.data[12 + i] = _fft.peak_snr_x[i];
		dbg.data[15 + i] = _fft.peak_snr_y[i];
		dbg.data[18 + i] = _fft.peak_snr_z[i];
	}

	// [21..25] 油门
	float sum = 0.f;
	unsigned cnt = 0;

	for (unsigned i = 0; i < 4; i++) {
		const float v = _have_motors ? _motors.control[i] : NAN;
		dbg.data[22 + i] = v;

		if (PX4_ISFINITE(v)) {
			sum += v;
			cnt++;
		}
	}

	dbg.data[21] = (cnt > 0) ? (sum / cnt) : NAN;

	// [26..28] 姿态角（度）
	if (_have_att) {
		const float w = _att.q[0], x = _att.q[1], y = _att.q[2], z = _att.q[3];
		constexpr float r2d = 57.295779513f;

		const float roll  = atan2f(2.f * (w * x + y * z), 1.f - 2.f * (x * x + y * y));
		float sp = 2.f * (w * y - z * x);
		sp = math::constrain(sp, -1.f, 1.f);
		const float pitch = asinf(sp);
		const float yaw   = atan2f(2.f * (w * z + x * y), 1.f - 2.f * (y * y + z * z));

		dbg.data[26] = roll * r2d;
		dbg.data[27] = pitch * r2d;
		dbg.data[28] = yaw * r2d;
	}

	// [29] 帧序号
	dbg.data[29] = static_cast<float>(_seq++ & 0xFFFF);

	_debug_pub.publish(dbg);
	_published++;
}

int GyroFftBridge::print_status()
{
	PX4_INFO("array_id=%u name=%s  period=%" PRIu64 " ms",
		 kArrayId, kArrayName, (uint64_t)(kPeriod / 1000));
	PX4_INFO("published=%" PRIu32 "  skipped(no new fft)=%" PRIu32,
		 _published, _skipped);
	PX4_INFO("have_motors=%d have_att=%d", (int)_have_motors, (int)_have_att);

	PX4_INFO("fft: sample_rate=%.2f Hz  resolution=%.3f Hz",
		 (double)_fft.sensor_sample_rate_hz, (double)_fft.resolution_hz);
	PX4_INFO("  X: %7.1f %7.1f %7.1f  Hz (snr %.1f %.1f %.1f)",
		 (double)_fft.peak_frequencies_x[0], (double)_fft.peak_frequencies_x[1],
		 (double)_fft.peak_frequencies_x[2],
		 (double)_fft.peak_snr_x[0], (double)_fft.peak_snr_x[1], (double)_fft.peak_snr_x[2]);
	PX4_INFO("  Y: %7.1f %7.1f %7.1f  Hz (snr %.1f %.1f %.1f)",
		 (double)_fft.peak_frequencies_y[0], (double)_fft.peak_frequencies_y[1],
		 (double)_fft.peak_frequencies_y[2],
		 (double)_fft.peak_snr_y[0], (double)_fft.peak_snr_y[1], (double)_fft.peak_snr_y[2]);
	PX4_INFO("  Z: %7.1f %7.1f %7.1f  Hz (snr %.1f %.1f %.1f)",
		 (double)_fft.peak_frequencies_z[0], (double)_fft.peak_frequencies_z[1],
		 (double)_fft.peak_frequencies_z[2],
		 (double)_fft.peak_snr_z[0], (double)_fft.peak_snr_z[1], (double)_fft.peak_snr_z[2]);

	if (_have_motors) {
		PX4_INFO("motors: %.3f %.3f %.3f %.3f",
			 (double)_motors.control[0], (double)_motors.control[1],
			 (double)_motors.control[2], (double)_motors.control[3]);
	}

	return PX4_OK;
}

int GyroFftBridge::task_spawn(int argc, char *argv[])
{
	GyroFftBridge *instance = new GyroFftBridge();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		instance->ScheduleOnInterval(kPeriod);

		PX4_INFO("started: debug_array id=%u name=%s @ 10 Hz", kArrayId, kArrayName);

		return PX4_OK;
	}

	PX4_ERR("alloc failed");
	return PX4_ERROR;
}

int GyroFftBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GyroFftBridge::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Bridge the onboard gyro FFT peaks (sensor_gyro_fft) plus the current motor
thrust and attitude into a debug_array, so they can be streamed over MAVLink
DEBUG_FLOAT_ARRAY.

Needed on boards without SD card or USB, where ulog download is impossible and
the raw gyro rate does not fit the serial link. On this board the raw gyro rate
is 1997.95 Hz (about 148 KB/s), while the serial link tops out near 12 KB/s and
the mavlink main loop limits a single stream to about 154 Hz.

Publishes debug_array with id=0, name="GYROFFT" at 10 Hz.

Field layout:
  [0] sensor_sample_rate_hz   [1] resolution_hz        [2] device_id
  [3..5] peak_x  [6..8] peak_y  [9..11] peak_z          (Hz)
  [12..14] snr_x [15..17] snr_y [18..20] snr_z          (dB)
  [21] throttle_avg   [22..25] motor[0..3]
  [26] roll_deg  [27] pitch_deg  [28] yaw_deg
  [29] seq

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gyrofft_bridge", "module");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");

	return 0;
}

extern "C" __EXPORT int gyrofft_bridge_main(int argc, char *argv[])
{
	return ModuleBase::main(GyroFftBridge::desc, argc, argv);
}
