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
 * @file qmc5883l_softi2c_main.cpp
 * @brief QMC5883L 磁罗盘命令行入口（软件 I²C 版本）。
 *
 * 用法：
 *   qmc5883l_softi2c start [-R <rot>] [-f <kHz>] [-X]
 *   qmc5883l_softi2c stop
 *   qmc5883l_softi2c status
 *   qmc5883l_softi2c test          ← 接线自检（驱动必须已停）
 *
 * 为什么需要 -X：
 *   PX4 的校准逻辑里"外部罗盘"与"内部罗盘"走不同分支
 *   （SENS_MAG_AUTOROT 只对外部罗盘自动解算安装方向，
 *     外部件默认校准优先级 75 > 内部 50）。
 *   本模块是外挂件，所以正常用法是带 -X。
 *   ⚠️ 这里的 -X 不是"总线过滤"（软件 I²C 没有硬件总线号），
 *      只用来给 PX4Magnetometer 打 external 标记。
 */

#include "QMC5883LSoftI2C.hpp"

#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <cstring>

using namespace rm2027;

/* ------------------------------------------------------------------------- */
/* 真实引脚实现：STM32H7 GPIO（开漏输出）+ hrt 忙等                             */
/* ------------------------------------------------------------------------- */

#ifndef QMC5883L_SOFTI2C_NO_BOARD_PINS
class Stm32SoftI2CPins : public SoftI2CPins
{
public:
	explicit Stm32SoftI2CPins(uint32_t half_period_us) : _half_period_us(half_period_us) {}

	void configure() override
	{
		px4_arch_configgpio(BOARD_QMC5883L_SOFTI2C_SCL_PINSET);
		px4_arch_configgpio(BOARD_QMC5883L_SOFTI2C_SDA_PINSET);
		/* 开漏 + 写 1 = 释放。**不开内部上拉** —— 上拉由模块板载 4.7kΩ 提供，
		 * 这样"空闲时读不到高电平"就能直接说明线没接好。 */
		px4_arch_gpiowrite(BOARD_QMC5883L_SOFTI2C_SCL_PINSET, 1);
		px4_arch_gpiowrite(BOARD_QMC5883L_SOFTI2C_SDA_PINSET, 1);
	}

	void set_scl(bool high) override { px4_arch_gpiowrite(BOARD_QMC5883L_SOFTI2C_SCL_PINSET, high); }
	void set_sda(bool high) override { px4_arch_gpiowrite(BOARD_QMC5883L_SOFTI2C_SDA_PINSET, high); }
	bool get_scl() override { return px4_arch_gpioread(BOARD_QMC5883L_SOFTI2C_SCL_PINSET) != 0; }
	bool get_sda() override { return px4_arch_gpioread(BOARD_QMC5883L_SOFTI2C_SDA_PINSET) != 0; }

	/** 半个位周期 = 忙等。100 kHz ⇒ 5 µs。
	 *  ⚠️ 循环有**硬上限**：万一 hrt 异常停住，这里只会变慢，
	 *     不会把工作队列线程永久卡死（VL53L0X 那边踩过一次"驱动静默停住"）。 */
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
#endif /* QMC5883L_SOFTI2C_NO_BOARD_PINS */

/* ------------------------------------------------------------------------- */
/* 全局对象（只建一次，stop/start 复用）                                        */
/* ------------------------------------------------------------------------- */

#ifndef QMC5883L_SOFTI2C_NO_BOARD_PINS
static Stm32SoftI2CPins	*g_pins = nullptr;
static SoftI2C		*g_bus = nullptr;
static QMC5883LSoftI2C	*g_dev = nullptr;
static bool		 g_running = false;
#endif

static void print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
QMC5883L magnetometer over a **software I2C** (bit-banged) bus.

This board does not route a usable hardware I2C bus to any connector, so this
driver drives two plain GPIOs itself. The register sequence and scaling are
identical to the upstream `qmc5883l` driver; only the byte transport differs.

Publishes `sensor_mag`. Run `qmc5883l_softi2c test` first if unsure about the
wiring: it checks pins, addressing and the chip id step by step and prints a
plain-language verdict.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("qmc5883l_softi2c", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("magnetometer");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "start driver");
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 41, "sensor rotation", true);
	PRINT_MODULE_USAGE_PARAM_INT('f', 100, 10, 400, "software I2C clock in kHz", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('X', "external magnetometer (affects calibration priority)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "print status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "wiring / chip self test (driver must be stopped)");
}

extern "C" __EXPORT int qmc5883l_softi2c_main(int argc, char *argv[]);

int qmc5883l_softi2c_main(int argc, char *argv[])
{
#ifdef QMC5883L_SOFTI2C_NO_BOARD_PINS
	PX4_ERR("board_config.h does not define BOARD_QMC5883L_SOFTI2C_{SCL,SDA}_PINSET");
	PX4_ERR("=> this board has no pins assigned for this driver");
	return 1;
#else

	const char *verb = (argc > 1) ? argv[1] : nullptr;

	int rotation = 0;	/* ROTATION_NONE，方向交给校准时的自动解算 */
	int bus_khz = 100;
	bool external = false;

	int myoptind = 2;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "R:f:X", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = atoi(myoptarg);
			break;

		case 'f':
			bus_khz = atoi(myoptarg);
			break;

		case 'X':
			external = true;
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

	/* 引脚/总线对象只建一次，允许 stop → test → start 来回切 */
	const uint32_t half_period_us = 1000u / (bus_khz * 2u);

	if (g_pins == nullptr) {
		g_pins = new Stm32SoftI2CPins(half_period_us);
		g_bus = new SoftI2C(*g_pins);

	} else {
		g_pins->set_half_period_us(half_period_us);
	}

	if (g_pins == nullptr || g_bus == nullptr) {
		PX4_ERR("allocation failed");
		return 1;
	}

	if (!strcmp(verb, "start")) {
		if (g_running) {
			PX4_WARN("already running");
			return 0;
		}

		if (bus_khz < 10 || bus_khz > 400) {
			PX4_ERR("invalid bus clock %d kHz", bus_khz);
			return 1;
		}

		/* init() 里会先做一次软复位，失败就直接报错（不会默默超时） */
		if (g_bus->init() != 0) {
			PX4_ERR("bus init failed: SCL/SDA not pulled high -> check wiring "
				"(module C/D pads, 4.7k pull-ups, 3.3V supply)");
			return 1;
		}

		if (g_dev == nullptr) {
			g_dev = new QMC5883LSoftI2C(*g_bus, static_cast<enum Rotation>(rotation), external);

			if (g_dev == nullptr) {
				PX4_ERR("allocation failed");
				return 1;
			}
		}

		if (g_dev->init() != PX4_OK) {
			PX4_ERR("device init failed (chip id mismatch or no ACK at 0x%02X)", QMC5883LSoftI2C::I2C_ADDRESS);
			return 1;
		}

		g_running = true;
		PX4_INFO("started on software I2C @ %d kHz, addr 0x%02X, rotation %d, external=%d",
			 bus_khz, QMC5883LSoftI2C::I2C_ADDRESS, rotation, external ? 1 : 0);
		return 0;
	}

	if (!strcmp(verb, "stop")) {
		if (!g_running) {
			PX4_WARN("not running");
			return 0;
		}

		/* 只停调度，不 delete 对象：允许反复 start/stop，也避免 device_id 变化 */
		g_dev->ScheduleClear();
		g_running = false;
		PX4_INFO("stopped");
		return 0;
	}

	if (!strcmp(verb, "status")) {
		if (!g_running || g_dev == nullptr) {
			PX4_WARN("not running");
			return 0;
		}

		g_dev->print_status();
		return 0;
	}

	if (!strcmp(verb, "test")) {
		if (g_running) {
			PX4_ERR("stop the driver first: qmc5883l_softi2c stop");
			return 1;
		}

		PX4_INFO("--- QMC5883L software I2C self test ---");

		/* 1. 引脚自检：能不能拉低、放开后能不能被上拉拉回高 */
		g_bus->init();	/* 内部会 configure 引脚 */

		bool scl_ok = false;
		bool sda_ok = false;
		const bool lines_ok = g_bus->selftest_lines(&scl_ok, &sda_ok);

		PX4_INFO("1) pins    : SCL %s, SDA %s",
			 scl_ok ? "OK" : "FAIL (stuck low / no pull-up)",
			 sda_ok ? "OK" : "FAIL (stuck low / no pull-up)");

		if (!lines_ok) {
			PX4_ERR("   -> 线没接好 / 模块没供电 / 上拉缺失。检查模块 C/D 焊盘与 GND/VCC。");
			return 1;
		}

		/* 2. 地址应答：0x0D 有没有 ACK */
		const bool ack = g_bus->probe(QMC5883LSoftI2C::I2C_ADDRESS);
		PX4_INFO("2) address : 0x%02X %s", QMC5883LSoftI2C::I2C_ADDRESS,
			 ack ? "ACK (device present)" : "NACK (no device at this address)");

		if (!ack) {
			PX4_ERR("   -> 地址不对，或 SDA/SCL 接反了。");
			return 1;
		}

		/* 3. 芯片 ID：QMC5883L 固定读回 0xFF */
		uint8_t reg = static_cast<uint8_t>(QST_QMC5883L::Register::CHIP_ID);
		uint8_t chip_id = 0;
		const int rd = g_bus->write_read(QMC5883LSoftI2C::I2C_ADDRESS, &reg, 1, &chip_id, 1);

		if (rd != 0) {
			PX4_ERR("3) chip id : read failed");
			return 1;
		}

		PX4_INFO("3) chip id : 0x%02X (expected 0x%02X) %s", chip_id,
			 QST_QMC5883L::Chip_ID, chip_id == QST_QMC5883L::Chip_ID ? "OK" : "MISMATCH");

		if (chip_id != QST_QMC5883L::Chip_ID) {
			PX4_ERR("   -> 不是 QMC5883L（注意 QMC5883P 的 ID 不同，需要另一个驱动）。");
			return 1;
		}

		PX4_INFO("--- all checks passed; safe to 'qmc5883l_softi2c start -X' ---");
		return 0;
	}

	print_usage();
	return 1;
#endif /* QMC5883L_SOFTI2C_NO_BOARD_PINS */
}
