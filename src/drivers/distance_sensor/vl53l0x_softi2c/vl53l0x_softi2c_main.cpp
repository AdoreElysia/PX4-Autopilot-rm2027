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
 * @file vl53l0x_softi2c_main.cpp
 * @brief VL53L0X 测距（软件 I²C）—— 模块入口
 *
 * 用法：
 *   vl53l0x_softi2c start [-R 25] [-f 100]   启动（默认向下安装、100 kHz）
 *   vl53l0x_softi2c stop
 *   vl53l0x_softi2c status
 *   vl53l0x_softi2c test                     接线自检（不启动测量，逐个环节报结论）
 *
 * ★★★ 临时引脚（2026-10-18）★★★
 *   本板（rm2027/fmu-h743）没有把任何一对可做硬件 I²C 的引脚引到插座上，
 *   所以本驱动用**软件 I²C**，占用 **U18 座（UART7 那一路）** 的 4 根线：
 *
 *        U18 丝印(左→右)： 5    R    T    G
 *        U18 引脚号     ： 1    2    3    4
 *        接到 VL53L0X    ： VIN  SDA  SCL  GND
 *        本该是          ： 5V  UART7_RX(PE7)  UART7_TX(PE8)  GND
 *
 *   代价（有意接受，属于临时方案）：
 *     - UART7 不能再当串口用（ESC3 的串口遥测没了）；
 *     - U14（ESC3 座）的 2 号脚与 U18 的 2 号脚是同一根线（PE7），
 *       所以**不要**再往 U14 上接 ESC3 的遥测线，否则会和 SDA 打架；
 *     - 两脚是软件时序，不是硬件外设：速率低（默认 100 kHz 全是软件拉的），
 *       且要占 CPU —— 按 100 kHz 估算约 5 ms / 50 ms 周期（≈10% 一个核），
 *       以"连续 ≤1.4 ms 的忙等块"形式出现在**独占队列 wq:I2C3** 上；
 *       嫌重可以把 `-f` 调到 200（字节数不变、时间减半）。
 *   ⇒ 硬件改版把真正的 I²C 引出后：删掉本驱动、改回上游 vl53l0x 驱动即可。
 */

#include "SoftI2C.hpp"
#include "VL53L0XSoftI2C.hpp"

#include <stdlib.h>
#include <string.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_arch/micro_hal.h>

#include <board_config.h>

#if !defined(BOARD_VL53L0X_SOFTI2C_SCL_PINSET) || !defined(BOARD_VL53L0X_SOFTI2C_SDA_PINSET)
#  define VL53L0X_SOFTI2C_NO_BOARD_PINS 1
#endif

/* ------------------------------------------------------------------------- */
/* 真实引脚实现：STM32H7 GPIO（开漏输出）+ hrt 忙等                             */
/* ------------------------------------------------------------------------- */

#ifndef VL53L0X_SOFTI2C_NO_BOARD_PINS
class Stm32SoftI2CPins : public rm2027::SoftI2CPins
{
public:
	explicit Stm32SoftI2CPins(uint32_t half_period_us) : _half_period_us(half_period_us) {}

	void configure() override
	{
		px4_arch_configgpio(BOARD_VL53L0X_SOFTI2C_SCL_PINSET);
		px4_arch_configgpio(BOARD_VL53L0X_SOFTI2C_SDA_PINSET);
		/* 开漏 + 写 1 = 释放。本驱动**不**使能内部上拉（上拉应由模块提供） */
		px4_arch_gpiowrite(BOARD_VL53L0X_SOFTI2C_SCL_PINSET, 1);
		px4_arch_gpiowrite(BOARD_VL53L0X_SOFTI2C_SDA_PINSET, 1);
	}

	void set_scl(bool high) override { px4_arch_gpiowrite(BOARD_VL53L0X_SOFTI2C_SCL_PINSET, high); }
	void set_sda(bool high) override { px4_arch_gpiowrite(BOARD_VL53L0X_SOFTI2C_SDA_PINSET, high); }
	bool get_scl() override { return px4_arch_gpioread(BOARD_VL53L0X_SOFTI2C_SCL_PINSET) != 0; }
	bool get_sda() override { return px4_arch_gpioread(BOARD_VL53L0X_SOFTI2C_SDA_PINSET) != 0; }

	/** 半个位周期 = 忙等。100 kHz ⇒ 5 µs；VL53L0X 手册允许任意慢（FI2C 最小 0 Hz）。
	 *  ⚠️ 循环有**硬上限**（约 1 ms 的迭代数）：万一 hrt 时间异常停止前进，
	 *     这里也只是变慢，不会把工作队列线程永久卡死（踩过一次"驱动静默停住"）。 */
	void delay_half_period() override
	{
		const hrt_abstime t0 = hrt_absolute_time();
		uint32_t spins = 0;

		while (hrt_elapsed_time(&t0) < _half_period_us) {
			if (++spins > 20000u) { break; }
		}
	}

	void set_half_period_us(uint32_t us) { _half_period_us = us; }

private:
	uint32_t _half_period_us;
};
#endif /* VL53L0X_SOFTI2C_NO_BOARD_PINS */

/* ------------------------------------------------------------------------- */
/* 模块状态                                                                   */
/* ------------------------------------------------------------------------- */

/* device_id 位布局（src/lib/drivers/device/Device.hpp 的 DeviceStructure）：
 *     devtype(8) | address(8) | bus(5) | bus_type(3)
 * bus_type: 1 = I2C。
 * ⚠️ bus 字段填 7 只是**为了让 device_id 唯一、便于区分**：
 *    软件 I²C 不属于任何一条硬件总线，7 不是 STM32 的 I2C 外设号。 */
static constexpr uint8_t kBusTypeI2C = 1;
static constexpr uint8_t kSoftBusId = 7;

static uint32_t make_device_id()
{
	return (static_cast<uint32_t>(DRV_DIST_DEVTYPE_VL53L0X) << 24)
	       | (static_cast<uint32_t>(VL53L0XSoftI2C::I2C_ADDRESS) << 16)
	       | (static_cast<uint32_t>(kSoftBusId) << 3)
	       | kBusTypeI2C;
}

#ifndef VL53L0X_SOFTI2C_NO_BOARD_PINS
static Stm32SoftI2CPins *g_pins = nullptr;
static rm2027::SoftI2C	*g_bus = nullptr;
static VL53L0XSoftI2C	*g_dev = nullptr;
static bool		 g_running = false;   /* 允许 stop → test → start 来回切，且对象不 delete */

/** 把自检结果写进 SWD 状态块（这样即使没有串口，也能从 SWD 读出 test 的结论） */
static void status_fill_from_test(bool lines_ok, bool ack, uint8_t model)
{
	rm2027_vl53l0x_status.magic = 0x564C3058;
	rm2027_vl53l0x_status.version = 1;
	rm2027_vl53l0x_status.uptime_ms = static_cast<uint32_t>(hrt_absolute_time() / 1000);
	rm2027_vl53l0x_status.lines_ok = lines_ok ? 1 : 0;
	rm2027_vl53l0x_status.ack_ok = ack ? 1 : 0;
	rm2027_vl53l0x_status.model_id = model;
	rm2027_vl53l0x_status.scl_high = (g_bus && g_bus->scl_level()) ? 1 : 0;
	rm2027_vl53l0x_status.sda_high = (g_bus && g_bus->sda_level()) ? 1 : 0;
	rm2027_vl53l0x_status.bus_errors = g_bus ? g_bus->errors() : 0;
	rm2027_vl53l0x_status.bus_nacks = g_bus ? g_bus->nacks() : 0;
	rm2027_vl53l0x_status.bus_bytes = g_bus ? g_bus->bytes() : 0;
}

/** 引脚/总线对象只建一次（重复 start 时复用） */
static int ensure_bus(uint32_t half_period_us)
{
	if (g_pins == nullptr) {
		g_pins = new Stm32SoftI2CPins(half_period_us);
		g_bus = new rm2027::SoftI2C(*g_pins);

	} else {
		g_pins->set_half_period_us(half_period_us);   /* 允许换 SCL 频率 */
	}

	return (g_pins != nullptr && g_bus != nullptr) ? 0 : -1;
}
#endif

static void print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
VL53L0X time-of-flight distance sensor over a **software I2C** (bit-banged) bus.

This board has no hardware I2C bus broken out to a connector, so this driver
drives two plain GPIOs itself. The register sequences are the same ST reference
sequence used by the upstream vl53l0x driver; only the byte transport differs.

Publishes `distance_sensor`. Use `vl53l0x_softi2c test` first if you are not
sure about the wiring: it checks pins, addressing and the chip model id step by
step and prints the verdict in plain words.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("vl53l0x_softi2c", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "start driver");
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 0, 25, "sensor rotation (25 = downward facing)", true);
	PRINT_MODULE_USAGE_PARAM_INT('f', 100, 10, 400, "software I2C clock in kHz", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "print status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "wiring / chip self test (driver must be stopped)");
}

extern "C" __EXPORT int vl53l0x_softi2c_main(int argc, char *argv[]);

int vl53l0x_softi2c_main(int argc, char *argv[])
{
	const char *verb = (argc > 1) ? argv[1] : nullptr;

	int rotation = 25;   /* ROTATION_DOWNWARD_FACING */
	int bus_khz = 100;

	int myoptind = 2;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "R:f:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = atoi(myoptarg);
			break;

		case 'f':
			bus_khz = atoi(myoptarg);
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

#ifdef VL53L0X_SOFTI2C_NO_BOARD_PINS
	PX4_ERR("board_config.h does not define BOARD_VL53L0X_SOFTI2C_{SCL,SDA}_PINSET");
	PX4_ERR("=> this board has no (temporary) pins assigned for this driver");
	return 1;
#else

	if (!strcmp(verb, "start")) {
		if (g_running) {
			PX4_WARN("already running");
			return 0;
		}

		if (bus_khz < 10 || bus_khz > 400) {
			PX4_ERR("invalid bus clock %d kHz", bus_khz);
			return 1;
		}

		const uint32_t half_period_us = 500u / static_cast<uint32_t>(bus_khz);
		PX4_INFO("software I2C: SCL on U18.3 / SDA on U18.2, ~%d kHz (TEMPORARY pins)",
			 bus_khz);

		if (ensure_bus(half_period_us) != 0) {
			PX4_ERR("allocation failed");
			return 1;
		}

		if (g_dev == nullptr) {
			g_dev = new VL53L0XSoftI2C(*g_bus, make_device_id(), static_cast<enum Rotation>(rotation));

			if (g_dev == nullptr) {
				PX4_ERR("allocation failed");
				return 1;
			}
		}

		if (g_dev->Start() != PX4_OK) {
			PX4_ERR("start failed (run 'vl53l0x_softi2c test' for a step-by-step check)");
			return 1;
		}

		g_running = true;
		return 0;
	}

	if (!strcmp(verb, "stop")) {
		if (!g_running) {
			PX4_WARN("not running");
			return 0;
		}

		g_dev->Stop();
		g_running = false;
		return 0;
	}

	if (!strcmp(verb, "status")) {
		if (g_dev == nullptr) {
			PX4_WARN("not running");
			return 0;
		}

		g_dev->PrintStatus();
		return 0;
	}

	if (!strcmp(verb, "test")) {
		if (g_running) {
			PX4_WARN("driver is running - stop it first (the test drives the bus itself)");
			return 1;
		}

		if (ensure_bus(500u / static_cast<uint32_t>(bus_khz)) != 0) {
			PX4_ERR("allocation failed");
			return 1;
		}

		PX4_INFO("=== vl53l0x_softi2c wiring test ===");
		PX4_INFO("pins: SCL = U18.3 (PE8 / silkscreen 'T'), SDA = U18.2 (PE7 / 'R')");
		PX4_INFO("      VIN = U18.1 (5 V), GND = U18.4   [TEMPORARY WIRING]");

		const int init_ret = g_bus->init();
		PX4_INFO("[1/3] bus idle state : %s", init_ret == 0 ? "OK (both lines high)" :
			 "FAIL - SCL or SDA is not high");

		if (init_ret != 0) {
			PX4_ERR("     => module not powered / not plugged / SDA-SCL swapped");
			PX4_ERR("     => remember: the pull-ups live on the sensor module, and this");
			PX4_ERR("        driver deliberately does not enable internal pull-ups");
			status_fill_from_test(false, false, 0);
			return 1;
		}

		bool scl_ok = false, sda_ok = false;
		const bool lines_ok = g_bus->selftest_lines(&scl_ok, &sda_ok);
		PX4_INFO("[2/3] pin drive test : SCL %s, SDA %s", scl_ok ? "OK" : "FAIL",
			 sda_ok ? "OK" : "FAIL");

		if (!lines_ok) {
			PX4_ERR("     => a line cannot be pulled low or does not come back high");
			PX4_ERR("        (open wire, wrong connector pin, or missing pull-up)");
		}

		const bool ack = g_bus->probe(VL53L0XSoftI2C::I2C_ADDRESS);
		PX4_INFO("[3/3] address 0x29   : %s", ack ? "ACK (a device answered)" :
			 "NO ACK (nothing at 0x29)");

		if (!ack) {
			PX4_ERR("     => check VIN/GND, then check for swapped SDA/SCL");
			status_fill_from_test(lines_ok, false, 0);
			return 1;
		}

		uint8_t model = 0;
		const uint8_t reg = VL53L0XSoftI2C::REG_MODEL_ID;

		if (g_bus->write_read(VL53L0XSoftI2C::I2C_ADDRESS, &reg, 1, &model, 1) != 0) {
			PX4_ERR("     model id read failed (I2C error)");
			status_fill_from_test(lines_ok, true, 0);
			return 1;
		}

		PX4_INFO("      model id (0xC0) : 0x%02x %s", static_cast<unsigned>(model),
			 model == VL53L0XSoftI2C::MODEL_ID_VL53L0X ? "= VL53L0X  -> 'start' should work" :
			 "!= 0xEE (VL53L1X? then this driver cannot drive it)");

		PX4_INFO("i2c stats: errors=%" PRIu32 " nacks=%" PRIu32, g_bus->errors(), g_bus->nacks());
		status_fill_from_test(lines_ok, true, model);
		return (model == VL53L0XSoftI2C::MODEL_ID_VL53L0X) ? 0 : 1;
	}

	print_usage();
	return 1;
#endif /* VL53L0X_SOFTI2C_NO_BOARD_PINS */
}
