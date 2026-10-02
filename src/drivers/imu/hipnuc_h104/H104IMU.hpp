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
 * @file H104IMU.hpp
 * @brief H104（HiPNUC HI04M0）外置陀螺 —— 串口 IMU 驱动
 *
 * 数据来源：USART1 (=/dev/ttyS0, TX=PB6 / RX=PA10)，8N1 @115200，HI91 @100 Hz。
 * 解析见 h104_protocol.hpp（无 PX4 依赖，可主机单元测试）。
 *
 * 发布：**同一个 device_id** 发 sensor_gyro + sensor_accel（沿用 PX4 IMU 驱动的
 *      DRV_IMU_DEVTYPE_* 约定，accel 与 gyro 共用同一个 device_id，见 BMI270 同样写法）。
 *      `sensors` 模块随后按 uORB 实例号自动为这一对建 VehicleIMU
 *      （src/modules/sensors/sensors.cpp: InitializeVehicleIMU）。
 *
 * ⚠️ 坐标系：解析器已把模块的 RFU 载体系换算成 PX4 的 **FRD**（[y, x, −z]），
 *    所以本驱动给基类传 ROTATION_NONE；`-r` 参数留给**安装方向**偏差。
 *    实测依据：静止时 acc = (0.02, 0.17, −9.84) m/s²，正是水平静止的期望值。
 *    ⚠️ X/Y 的正负方向**尚未**用实物转动验证 ⇒ 见 docs 的"放飞前必须解决"。
 */

#pragma once

#include "h104_protocol.hpp"

#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <lib/conversion/rotation.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

class H104IMU : public px4::ScheduledWorkItem
{
public:
	H104IMU(const char *port, uint32_t device_id, enum Rotation rotation);
	~H104IMU() override;

	/** 打开串口并开始周期读 */
	int  Start();

	/** 停止（清调度 + 关串口），对象保留（避免 delete 与工作队列的竞争） */
	void Stop();

	void PrintStatus();

	/** 解析器回调跳板（Parser 用的是普通函数指针） */
	static void SampleTrampoline(void *ctx, const hipnuc_h104::Sample &sample)
	{
		static_cast<H104IMU *>(ctx)->Publish(sample);
	}

private:
	void Run() override;

	int  OpenPort();
	void Publish(const hipnuc_h104::Sample &sample);
	void UpdatePerfCounters();

	const char  *_port;
	const uint32_t _device_id;
	const enum Rotation _rotation;

	int _fd{-1};

	hipnuc_h104::Parser _parser;
	PX4Accelerometer    _px4_accel;
	PX4Gyroscope        _px4_gyro;

	/* perf 计数器（任务要求：收帧数 / 坏帧数 / CRC 错） */
	perf_counter_t _perf_frames;
	perf_counter_t _perf_crc_bad;
	perf_counter_t _perf_len_bad;
	perf_counter_t _perf_resync;
	perf_counter_t _perf_read_errors;
	perf_counter_t _perf_reopens;
	perf_counter_t _perf_overflow;

	float _temperature{NAN};
	bool  _ever_opened{false};
	bool  _first_frame_logged{false};
	bool  _stats_logged{false};
	uint32_t _publish_count{0};
	uint32_t _read_error_count{0};
	uint32_t _open_fail_count{0};
	uint32_t _no_data_rounds{0};
	hrt_abstime _last_frame_time{0};
	hrt_abstime _start_time{0};
};
