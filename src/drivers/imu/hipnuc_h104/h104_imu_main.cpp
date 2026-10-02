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
 * @file h104_imu_main.cpp
 * @brief H104（HiPNUC HI04M0）外置陀螺串口 IMU 驱动的模块入口
 *
 * 用法：
 *   h104_imu start [-p /dev/ttyS0] [-r 0]     启动
 *   h104_imu stop                             停止
 *   h104_imu status                           打印状态 + perf 计数
 *
 * -r 是**安装方向**旋转（PX4 Rotation 枚举值，默认 0）。
 *    注意：解析器已把模块的 RFU 载体系换算成 PX4 的 FRD，所以正常情况下保持 0；
 *    只有当模块在机架上的贴装方向与"右前上"不一致时才需要调它。
 */

#include "H104IMU.hpp"

#include <stdlib.h>
#include <string.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

/* device_id 的位布局（见 src/lib/drivers/device/Device.hpp 的 DeviceStructure）：
 *     devtype(8) | address(8) | bus(5) | bus_type(3)
 * 这里直接按布局拼，不依赖 device::Device 的可见性。
 * bus_type: 0=UNKNOWN 1=I2C 2=SPI 3=UAVCAN 4=SIMULATION 5=SERIAL 6=MAVLINK */
static constexpr uint32_t kBusTypeSerial = 5;
static constexpr uint8_t  kBusUSART1 = 1;

static uint32_t make_h104_device_id()
{
	return (static_cast<uint32_t>(DRV_IMU_DEVTYPE_HIPNUC_HI04) << 24)
	       | (static_cast<uint32_t>(0) << 16)                 /* address */
	       | (static_cast<uint32_t>(kBusUSART1) << 3)
	       | kBusTypeSerial;
}

/* 对象保留到进程结束（stop 只清调度 + 关串口），避免 delete 与工作队列的竞争 */
static H104IMU *g_dev = nullptr;

static void print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
H104 (HiPNUC HI04M0) external IMU over a serial (UART) link.

Publishes sensor_gyro + sensor_accel with the same device_id, so the sensors
module automatically pairs them into a VehicleIMU and EKF2 can use them.

The frame parser (HI91, 82 bytes, CRC-16/XMODEM) is unit tested on the host
against a real capture: see test_h104_protocol.cpp.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("h104_imu", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "启动");
	PRINT_MODULE_USAGE_PARAM_STRING('p', "/dev/ttyS0", "<file:dev>", "串口设备（本板 USART1=/dev/ttyS0）", true);
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 0, 40, "安装方向旋转（PX4 Rotation，默认 0）", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "停止");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "状态 + perf 计数");
}

extern "C" __EXPORT int h104_imu_main(int argc, char *argv[]);

int h104_imu_main(int argc, char *argv[])
{
	const char *verb = (argc > 1) ? argv[1] : nullptr;

	const char *port = "/dev/ttyS0";
	enum Rotation rotation = ROTATION_NONE;

	int myoptind = 2;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "p:r:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			port = myoptarg;
			break;

		case 'r':
			rotation = static_cast<enum Rotation>(atoi(myoptarg));
			break;

		default:
			print_usage();
			return 1;
		}
	}

	if (verb == nullptr) {
		print_usage();
		return 1;
	}

	if (!strcmp(verb, "start")) {
		if (g_dev != nullptr) {
			PX4_WARN("h104_imu 已在运行");
			return 0;
		}

		const uint32_t device_id = make_h104_device_id();
		g_dev = new H104IMU(port, device_id, rotation);

		if (g_dev == nullptr) {
			PX4_ERR("h104_imu: 分配失败");
			return 1;
		}

		/* OpenPort 失败时对象内部会自己 1 s 重试，所以这里不算致命 */
		g_dev->Start();
		return 0;
	}

	if (!strcmp(verb, "stop")) {
		if (g_dev == nullptr) {
			PX4_WARN("h104_imu 未在运行");
			return 0;
		}

		g_dev->Stop();
		return 0;
	}

	if (!strcmp(verb, "status")) {
		if (g_dev == nullptr) {
			PX4_WARN("h104_imu 未在运行");
			return 0;
		}

		g_dev->PrintStatus();
		return 0;
	}

	print_usage();
	return 1;
}
