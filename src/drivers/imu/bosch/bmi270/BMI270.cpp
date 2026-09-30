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
#include "BMI270.hpp"
#define BMI270_DEBUG
#define BMI270_MAX_FIFO_SAMPLES 8

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

// ★ rm2027/fmu-h743 板级补丁（2026-09-30）：原 ProcessAccel/ProcessGyro 做 FLU→FRD
// （y、z 取反）时用 `(v == INT16_MIN) ? INT16_MAX : -v` 防溢出，轮询路径保持一致。
static constexpr int16_t negate_clamped(int16_t v)
{
	return (v == INT16_MIN) ? INT16_MAX : -v;
}

// ★ rm2027/fmu-h743 板级补丁（2026-09-30）：这几个 BMI270 的 FIFO/INT 寄存器在
// 本板兼容件（华轩阳 HXY「ICM-42688P」）上【地址根本不存在】（见厂商手册 §4.1 寄存器表：
// 本片 FIFO 在 0x1C~0x21、中断配置在 0x06/0x07），所以写进去回读恒 0x00 是正常的
// ——不是"片子没有 FIFO"。兼容件分支下跳过它们的写与检查，避免做无用功。
static constexpr bool is_fifo_or_int_register(Register reg)
{
	switch (reg) {
	case Register::FIFO_WTM_0:
	case Register::FIFO_WTM_1:
	case Register::FIFO_CONFIG_0:
	case Register::FIFO_CONFIG_1:
	case Register::INT1_IO_CTRL:
	case Register::INT_MAP_DATA:
		return true;

	default:
		return false;
	}
}

/**
* The following device config microcode has the following copyright:
*
* Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
uint8_t maximum_fifo_config_file[] = { 0x5E,
				       0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0x1a, 0x00, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00,
				       0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0x90, 0x32, 0x21, 0x2e, 0x59, 0xf5,
				       0x10, 0x30, 0x21, 0x2e, 0x6a, 0xf5, 0x1a, 0x24, 0x22, 0x00, 0x80, 0x2e, 0x3b, 0x00, 0xc8, 0x2e, 0x44, 0x47, 0x22,
				       0x00, 0x37, 0x00, 0xa4, 0x00, 0xff, 0x0f, 0xd1, 0x00, 0x07, 0xad, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
				       0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00,
				       0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x00, 0x00,
				       0x00, 0x00, 0x00, 0x00, 0x11, 0x24, 0xfc, 0xf5, 0x80, 0x30, 0x40, 0x42, 0x50, 0x50, 0x00, 0x30, 0x12, 0x24, 0xeb,
				       0x00, 0x03, 0x30, 0x00, 0x2e, 0xc1, 0x86, 0x5a, 0x0e, 0xfb, 0x2f, 0x21, 0x2e, 0xfc, 0xf5, 0x13, 0x24, 0x63, 0xf5,
				       0xe0, 0x3c, 0x48, 0x00, 0x22, 0x30, 0xf7, 0x80, 0xc2, 0x42, 0xe1, 0x7f, 0x3a, 0x25, 0xfc, 0x86, 0xf0, 0x7f, 0x41,
				       0x33, 0x98, 0x2e, 0xc2, 0xc4, 0xd6, 0x6f, 0xf1, 0x30, 0xf1, 0x08, 0xc4, 0x6f, 0x11, 0x24, 0xff, 0x03, 0x12, 0x24,
				       0x00, 0xfc, 0x61, 0x09, 0xa2, 0x08, 0x36, 0xbe, 0x2a, 0xb9, 0x13, 0x24, 0x38, 0x00, 0x64, 0xbb, 0xd1, 0xbe, 0x94,
				       0x0a, 0x71, 0x08, 0xd5, 0x42, 0x21, 0xbd, 0x91, 0xbc, 0xd2, 0x42, 0xc1, 0x42, 0x00, 0xb2, 0xfe, 0x82, 0x05, 0x2f,
				       0x50, 0x30, 0x21, 0x2e, 0x21, 0xf2, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0xf0, 0x6f, 0x02, 0x30, 0x02, 0x42, 0x20,
				       0x26, 0xe0, 0x6f, 0x02, 0x31, 0x03, 0x40, 0x9a, 0x0a, 0x02, 0x42, 0xf0, 0x37, 0x05, 0x2e, 0x5e, 0xf7, 0x10, 0x08,
				       0x12, 0x24, 0x1e, 0xf2, 0x80, 0x42, 0x83, 0x84, 0xf1, 0x7f, 0x0a, 0x25, 0x13, 0x30, 0x83, 0x42, 0x3b, 0x82, 0xf0,
				       0x6f, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x00, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x3e, 0x84,
				       0x00, 0x40, 0x40, 0x42, 0x7e, 0x82, 0xe1, 0x7f, 0xf2, 0x7f, 0x98, 0x2e, 0x6a, 0xd6, 0x21, 0x30, 0x23, 0x2e, 0x61,
				       0xf5, 0xeb, 0x2c, 0xe1, 0x6f
				     };
// end of Bosch microcode copyright


BMI270::BMI270(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_drdy_gpio(config.drdy_gpio),
	_px4_accel(get_device_id(), config.rotation, config.external),
	_px4_gyro(get_device_id(), config.rotation, config.external)
{
	if (_drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");
	}

	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
}

BMI270::~BMI270()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_duplicate_sample_perf);
	perf_free(_drdy_missed_perf);
}

int BMI270::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	PX4_DEBUG("init function called, resetting...");


	return Reset() ? 0 : -1;
}

bool BMI270::Reset()
{
	_state = STATE::RESET;
	DataReadyInterruptDisable();
	ScheduleClear();
	ScheduleNow();
	return true;
}

// Debug function that Ardupilot is equipped with
void BMI270::CheckErrorRegister()
{
#ifdef BMI270_DEBUG
	uint8_t err = RegisterRead(Register::ERR_REG);

	if (err) {
		if ((err & 1) == 1) {
			uint8_t status =  RegisterRead(Register::INTERNAL_STATUS);

			switch (status & 0xF) {
			case 0:
				PX4_DEBUG("BMI270: not_init");
				break;

			case 2:
				PX4_DEBUG("BMI270: init_err");
				break;

			case 3:
				PX4_DEBUG("BMI270: drv_err");
				break;

			case 4:
				PX4_DEBUG("BMI270: sns_stop");
				break;

			case 5:
				PX4_DEBUG("BMI270: nvm_error");
				break;

			case 6:
				PX4_DEBUG("BMI270: start_up_error");
				break;

			case 7:
				PX4_DEBUG("BMI270: compat_error");
				break;

			case 1: // init ok
				if ((status >> 5 & 1) == 1) {
					PX4_DEBUG("BMI270: axes_remap_error");

				} else if ((status >> 6 & 1) == 1) {
					PX4_DEBUG("BMI270: odr_50hz_error");
				}

				break;
			}

		} else if ((err >> 6 & 1) == 1) {
			PX4_DEBUG("BMI270: fifo_err");

		} else if ((err >> 7 & 1) == 1) {
			PX4_DEBUG("BMI270: aux_err");

		} else {
			PX4_DEBUG("BMI270: internal error detected %d", err >> 1 & 0xF);
		}
	}

#endif

}

void BMI270::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void BMI270::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("FIFO empty interval: %d us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);

	// ★ rm2027/fmu-h743 板级补丁：兼容件走的是"查询方式"轮询数据寄存器（不是 BMI270 的 FIFO 路径）
	if (_skip_microcode) {
		PX4_INFO("compatible part: polling data registers 0x0C-0x17 every %u us (%u Hz)",
			 (unsigned)POLL_INTERVAL_US, (unsigned)(1000000 / POLL_INTERVAL_US));

		// 最近一次配置自查的快照（厂商手册 §5.8）。两个 *_CONF_ERR 位为 0 才说明
		// ACC_CONF/GYR_CONF 被器件接受了 —— 这是判断"读数偏低"是不是配置问题的第一手证据。
		PX4_INFO("compatible part: last DATA_STAT(0x0B)=0x%02x (ACC_CONF_ERR=%d GYR_CONF_ERR=%d)",
			 (unsigned)_compat_data_stat,
			 (int)((_compat_data_stat & DATA_STAT_BIT::acc_conf_err) != 0),
			 (int)((_compat_data_stat & DATA_STAT_BIT::gyr_conf_err) != 0));
	}

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	// ★ rm2027/fmu-h743：轮询路径的重复样本计数。
	//   长期快速上涨 ⇒ 器件真实 ODR 远低于轮询的 1 kHz（配置没生效的现场证据）。
	//   上板实测（FIFO 填充速率两法交叉验证）ODR ≈ 1593 Hz ⇒ 这个计数应长期为 0。
	perf_print_counter(_duplicate_sample_perf);
	perf_print_counter(_drdy_missed_perf);
}

int BMI270::probe()
{
	// When starting communication with the BMI270, according page 123 of the
	// datasheet a rising edge on CSB is required to start SPI.
	// It is recommended to just read the CHIP_ID register once to do that
	// but ignore the result.
	RegisterRead(Register::CHIP_ID);

	// It takes 200us for the device to start SPI communication.
	px4_usleep(200);

	const uint8_t CHIP_ID = RegisterRead(Register::CHIP_ID);

	if (CHIP_ID != chip_id) {
		// ★ rm2027/fmu-h743 板级补丁：本板器件是「BMI270 寄存器图兼容件」，
		//    寄存器图/功能与 BMI270 一致，但【不能上传微码】。
		//
		//    识别特征有两条，任一成立即认作兼容件（并置 _skip_microcode）：
		//      1) WHO_AM_I(0x01) == 0x6A —— 厂商手册（华轩阳 HXY「ICM-42688P」§4.1/§5.1）
		//         口径：本片把器件 ID 从 BMI270 的 0x00 挪到了 0x01，0x00 不在它的寄存器表里。
		//      2) CHIP_ID(0x00) == 0x00 —— 历史口径，已在实机验证过驱动能启动、数据能出，
		//         保留作兜底：即使 0x01 读出来不是 0x6A，也不会退化成"驱动起不来"。
		//    ⚠️ 红线：兼容件【绝不能】上传微码（实测会把器件锁死，只能断电恢复）。
		const uint8_t who_am_i = RegisterRead(Register::WHO_AM_I);

		if ((who_am_i == compat_who_am_i) || (CHIP_ID == 0x00)) {
			_skip_microcode = true;
			PX4_INFO("BMI270-compatible part detected (WHO_AM_I(0x01)=0x%02x, CHIP_ID(0x00)=0x%02x): microcode upload will be SKIPPED",
				 (unsigned)who_am_i, (unsigned)CHIP_ID);
			return PX4_OK;
		}

		DEVICE_DEBUG("unexpected CHIP_ID 0x%02x (WHO_AM_I 0x%02x)", CHIP_ID, who_am_i);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void BMI270::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:

		// ★ rm2027/fmu-h743 板级补丁（2026-09-30）：兼容件【跳过软复位】。
		//   0xB6 是 BMI270 的 ACC_SOFTRESET；按数据手册，软复位之后必须【重新上传微码】，
		//   否则加速度计部分不工作（FIFO 永远为空）。而本板器件禁用微码（上传会锁死），
		//   于是每 16 ms 一次的 "软复位→无微码→FIFO空→判失败→再软复位" 死循环。
		//   实测证据：手工把 PWR_CTRL(0x7D) 写成 0x0E（=accel_en|gyr_en|temp_en，与
		//   本驱动 _register_cfg 里的值一致）后，0x0C~0x17 立即出现实时 accel/gyro 数据，
		//   全程【没有】上传微码 ⇒ 芯片上电态本来就能出数据，是软复位把它打哑了。
		//   见 work/spi-fingerprint/（寄存器指纹）与 work/spi-runtime/。
		if (_skip_microcode) {
			_reset_timestamp = now;
			_failure_count = 0;
			_state = STATE::WAIT_FOR_RESET;
			ScheduleDelayed(1_ms);
			break;
		}

		// 0xB6 is written to the CMD register for a soft reset
		RegisterWrite(Register::CMD, 0xB6);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(1_ms); // Following a delay of 1 ms, all configuration settings are overwritten with their reset value.
		break;

	case STATE::WAIT_FOR_RESET:

		/*
		Hardware initialization steps according to datasheet:
		1. Disable PWR_CONF.adv_power_save and wait for 450us
		2. Write 0x00 to INIT_CTRL
		3. Burst write initialization file to INIT_DATA
		4. Write 0x01 to INIT_CTRL
		5. Wait 150ms and read register INTERNAL_STATUS for value 0b001
		6. If step 5 passed, enter configure state


		*/
	{
		const uint8_t chip_id_now = RegisterRead(Register::CHIP_ID);

		if ((chip_id_now == chip_id) || (_skip_microcode && (chip_id_now == 0x00))) {
			PX4_DEBUG("Read from CHIP_ID register and the IDs match (or compatible part)");

			// 1. Disable PWR_CONF.adv_power_save and wait for 450us
			//
			// ★ rm2027/fmu-h743（2026-09-30）：兼容件（华轩阳 HXY「ICM-42688P」）的寄存器表里
			//   【没有 0x7C 这个地址】（表从 0x6D/0x6F 直接跳到 0x7D），而手册 §4.1 明确警告
			//   "标识为保留的，在使用中不要更改，可能会引起永久性破坏"。
			//   兼容件分支不写 PWR_CONF —— Configure() 与周期性回读都已经跳过它，
			//   这里是最后一处漏网的写操作（WAIT_FOR_RESET 每次 Reset 都会走到），一并去掉。
			//   正品 BMI270 仍照 upstream 写（adv_power_save 是它必须关掉的东西）。
			if (!_skip_microcode) {
				RegisterWrite(Register::PWR_CONF, 0x00);
			}

			_state = STATE::MICROCODE_LOAD;
			ScheduleDelayed(450_us);

		} else {
			// RESET not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Reset failed, retrying");
				_state = STATE::RESET;
				ScheduleDelayed(100_ms);

			} else {
				PX4_DEBUG("Reset not complete, check again in 10 ms");
				ScheduleDelayed(10_ms);
			}
		}

		break;
	}

	case STATE::MICROCODE_LOAD:

		{
			// ★ rm2027/fmu-h743 板级补丁：BMI270 兼容件必须【跳过】微码上传。
			//    实测上传 329 字节微码会把该器件锁死（此后读全 0xFF，只能断电恢复）。
			//    已证明不上传微码它照样能出数据（PWR_CTRL=0x0E → 0x0C~0x17 有活数据）。
			if (_skip_microcode) {
				// 只打印第一次：兼容件每次 Reset 都会重新走到这里，
				// 每次都 PX4_INFO 会刷满 console 环形缓冲区。
				if (!_microcode_skip_logged) {
					PX4_INFO("Skipping BMI270 microcode upload (compatible part)");
					_microcode_skip_logged = true;
				}

				_state = STATE::CONFIGURE;
				ScheduleDelayed(1_ms);
				break;
			}

			// 2. Write 0x00 to INIT_CTRL

			RegisterWrite(Register::CONFIG1, 0x00);
			// give it the maximum FIFO config file
			PX4_DEBUG("attempting to upload initialization file onto BMI270");

			//  3. Burst write initialization file to INIT_DATA

			int res = transfer(maximum_fifo_config_file, nullptr, sizeof(maximum_fifo_config_file));

			if (res == PX4_OK) {
				RegisterWrite(Register::CONFIG1, 1);
				PX4_DEBUG("Successfully uploaded initialization file onto BMI270");
				PX4_DEBUG("Preparing to read INTERNAL_STATUS register");
				_state = STATE::CONFIGURE;
				ScheduleDelayed(150_ms);


			} else {
				PX4_DEBUG("Failed to upload initialization file onto BMI270, resetting");
				_state = STATE::RESET;
				ScheduleDelayed(10_ms);
			}

			break;
		}

	case STATE::CONFIGURE:

		PX4_DEBUG("IMU now in configure state");

		if (Configure()) {

			// ★★ rm2027/fmu-h743 板级补丁（2026-09-30）：本板兼容件走【查询方式】取数，
			//    Configure() 成功后不走 FIFO_READ，改走 DATA_READ
			//    （定时轮询、一次性突发读数据寄存器 0x0C~0x17）。
			//    为什么会走到这一步：BMI270 的 FIFO 寄存器地址(0x46~0x49/0x24~0x26)在本片上
			//    不存在（厂商手册：本片 FIFO 在 0x1C~0x21），原 FIFO 路径读不到任何东西。
			//    下面的 FIFO 路径完整保留给正品 BMI270（CHIP_ID=0x24），
			//    这样以后换回正品芯片时这份驱动不需要回滚代码，只要 _skip_microcode 为 false。
			if (_skip_microcode) {

				// 【强制走定时器路径，不用 DRDY 中断】：
				// BMI270 的 INT1_IO_CTRL(0x53)/INT_MAP_DATA(0x58) 在本片寄存器表里【不存在】，
				// 厂商手册里中断是 INT_CFG1(0x06)/INT_CFG2(0x07)（INT1_SEL[4:0] 选 DRDY_ACC/
				// DRDY_GYR…），DRDY 状态位在 DATA_STAT(0x0B)。所以原驱动的中断配置在本片是空操作、
				// 中断从未被验证过；而手册§描述明确支持"查询方式"取数 ⇒ 老老实实定时轮询，不赌中断。
				// 故：_data_ready_interrupt_enabled 显式置 false，
				//     且【不调用】DataReadyInterruptConfigure()。
				_data_ready_interrupt_enabled = false;

				// 清掉可能残留的失败计数/配置检查游标，避免带着上一次循环的状态进入新路
				_failure_count = 0;
				_checked_register = 0;
				_last_config_check_timestamp = now;

				// 轮询周期 1 ms（1 kHz）：见 POLL_INTERVAL_US 的说明
				// （1 kHz < 芯片 1600 Hz ODR ⇒ 只丢样本、不重复样本）
				ScheduleOnInterval(POLL_INTERVAL_US, POLL_INTERVAL_US);

				// 注意：这里【不】调用 FIFOReset()。FIFOReset 写的是 CMD(0x7E)=0xB0，
				// 而本片 0x7E 不在寄存器表里、软复位其实是 SOFT_RST(0x4A)=0xA5；
				// 对不存在的地址做写操作没有意义，不碰。
				_state = STATE::DATA_READ;
				break;
			}

			// if configure succeeded then start reading from FIFO

			if (DataReadyInterruptConfigure()) {
				_data_ready_interrupt_enabled = true;

				// backup schedule as a watchdog timeout
				ScheduleDelayed(100_ms);

			} else {
				_data_ready_interrupt_enabled = false;
				ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
			}

			FIFOReset();
			_state = STATE::FIFO_READ;


		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				PX4_DEBUG("Configure failed, retrying");
			}

			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::DATA_READ: {

			// ★★ rm2027/fmu-h743 板级补丁（2026-09-30）：兼容件专用取数路径 ——
			//    每个 ScheduleOnInterval 周期（1 ms）一次性突发读 0x0C~0x17 共 12 字节，
			//    组装成 6 个 int16 后交给 _px4_accel.update() / _px4_gyro.update() 发布。
			//
			//    为什么走这条路：原 FIFO 路径用的是 BMI270 的 FIFO 地址（配置 0x46~0x49、
			//    长度 0x24/0x25、数据 0x26），而本片（厂商手册：华轩阳 HXY「ICM-42688P」）
			//    的 FIFO 在 0x1C~0x21 ⇒ 原路径永远拿不到数据，会陷入
			//    "失败→Reset→重配置"的死循环。厂商手册明确支持"查询方式"读数据寄存器，
			//    所以这里按查询方式直接读 0x0C~0x17（实测是活数据且静止时自洽）。
			//    （本片 FIFO 是存在的，2048 Byte/341 组，默认 By-Pass；以后要省 CPU/不丢样本
			//      可以再按手册实现一条 FIFO 路径。）

			hrt_abstime timestamp_sample = now;

			if (DataReadPublish(timestamp_sample)) {
				_failure_count = 0;

				// 周期性回读关键配置寄存器（每秒 1 个，循环），确认配置没被器件丢掉。
				// 只查兼容件真正写过、且本片寄存器表里存在的 _register_cfg[1..4]
				// （PWR_CTRL / ACC_CONF / GYR_CONF / ACC_RANGE）；
				// [0] 是 PWR_CONF(0x7C)——本片没有这个地址、兼容件分支也不再写它；
				// FIFO/INT 那几个地址同样不存在。查了只会白白累加 _bad_register_perf。
				// 失败只计数、不 Reset（和原 FIFO 路径一致，原代码里那处 Reset() 本来就是注释掉的）。
				if (hrt_elapsed_time(&_last_config_check_timestamp) > 1_s) {
					_last_config_check_timestamp = now;

					const uint8_t idx = 1 + (_checked_register % (size_register_cfg_compat - 1));

					if (!RegisterCheck(_register_cfg[idx])) {
						perf_count(_bad_register_perf);
					}

					_checked_register++;

					_px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf));
					_px4_gyro.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf));
				}

			} else {
				_failure_count++;

				// 连续失败就做一次完整 Reset（Reset() 会 ScheduleClear() 再 ScheduleNow()，
				// 不会残留 interval、也不会有第二个调度源 ⇒ 不会死循环刷屏；
				// 日志一律 PX4_DEBUG，release 构建里根本不输出）。
				if (_failure_count > 10) {
					PX4_DEBUG("data register read failed %d times, resetting...", _failure_count);
					Reset();
					return;
				}
			}

			// 约 1 Hz 更新一次温度（照抄原 FIFO_READ 尾部的写法）
			if (hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {
				UpdateTemperature();
				_temperature_update_timestamp = now;
			}
		}

		break;

	case STATE::FIFO_READ: {

			PX4_DEBUG("reading from FIFO");

			hrt_abstime timestamp_sample = now;

			if (_data_ready_interrupt_enabled) {
				PX4_DEBUG("data ready interrupt enabled");
				// scheduled from interrupt if _drdy_timestamp_sample was set as expected
				const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample.fetch_and(0);

				if ((now - drdy_timestamp_sample) < _fifo_empty_interval_us) {
					timestamp_sample = drdy_timestamp_sample;

				} else {
					perf_count(_drdy_missed_perf);
				}

				// push backup schedule back
				ScheduleDelayed(_fifo_empty_interval_us * 2);
			}



			bool success = false;
			const uint16_t fifo_count = FIFOReadCount();

			// more bytes than what the buffer takes so an overflow
			if (fifo_count >= FIFO::SIZE) {
				FIFOReset();
				perf_count(_fifo_overflow_perf);

			} else if (fifo_count == 0) {
				perf_count(_fifo_empty_perf);

			} else {

				uint8_t samples = fifo_count / sizeof(FIFO::Data);

				// tolerate minor jitter, leave sample to next iteration if behind by only 1
				if (samples == _fifo_gyro_samples + 1) {
					timestamp_sample -= static_cast<int>(FIFO_SAMPLE_DT);
					samples--;
				}

				if (samples > FIFO_MAX_SAMPLES) {
					// not technically an overflow, but more samples than we expected or can publish
					FIFOReset();
					perf_count(_fifo_overflow_perf);

				} else if (samples >= _fifo_gyro_samples) {
					if (FIFORead(timestamp_sample, fifo_count)) {
						success = true;

						if (_failure_count > 0) {
							_failure_count--;
						}
					}
				}

			}

			if (!success) {
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 10) {
					PX4_DEBUG("failure count > 10, resetting...");
					Reset();
					return;
				}
			}

			if (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
				// check configuration registers periodically or immediately following any failure
				if (RegisterCheck(_register_cfg[_checked_register])) {
					_last_config_check_timestamp = now;
					_checked_register = (_checked_register + 1) % size_register_cfg;

				} else {
					PX4_DEBUG("register check failed, resetting...");

					// register check failed, force reset
					perf_count(_bad_register_perf);
					// Reset();
				}

			} else {
				// periodically update temperature (~1 Hz)
				if (hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {
					UpdateTemperature();
					_temperature_update_timestamp = now;
				}
			}

		}

		break;
	}
}

void BMI270::SetAccelScaleAndRange()
{
	const uint8_t ACC_RANGE = RegisterRead(Register::ACC_RANGE) & (Bit1 | Bit0);

	// ★ rm2027/fmu-h743 板级补丁（2026-09-30）：兼容件的加速度计绝对灵敏度比手册低 ~1.73x
	//   （实测：ACC_RANGE=0x03 时静止 1g 只读到 1185 counts，手册应为 2048 counts/g；
	//    0x00/0x01/0x02 分别是 9289.5/4648/2316，严格 8:4:2:1 ⇒ 只有绝对增益偏小）。
	//   这里乘上实测修正，使静止合矢量 = 1g。正品芯片（_skip_microcode == false）不受影响。
	//
	// ⚠️ 修正只作用于【scale】，不要作用于 set_range()：
	//    set_range() 声明的是器件的【物理量程】（±2/4/8/16g），是硬件属性；
	//    1.73x 是"每 g 对应的码值偏小"，属于 scale 的换算系数，量程并没有变。
	//    （先前 set_range 也被乘了修正：虽然 _clip_limit = range/scale 里分子分母同时
	//     抵消、裁剪行为没变，但 sensor_accel.range 会报成 1.73 倍，对外语义是错的。）
	const float gain_correction = _skip_microcode ? ACCEL_COMPAT_GAIN_CORRECTION : 1.f;

	switch (ACC_RANGE) {
	case acc_range_2g:
		_px4_accel.set_scale(2.f * CONSTANTS_ONE_G / 32768.f * gain_correction);
		_px4_accel.set_range(2.f * CONSTANTS_ONE_G);
		break;

	case acc_range_4g:
		_px4_accel.set_scale(4.f * CONSTANTS_ONE_G / 32768.f * gain_correction);
		_px4_accel.set_range(4.f * CONSTANTS_ONE_G);
		break;

	case acc_range_8g:
		_px4_accel.set_scale(8.f * CONSTANTS_ONE_G / 32768.f * gain_correction);
		_px4_accel.set_range(8.f * CONSTANTS_ONE_G);
		break;

	case acc_range_16g:
		_px4_accel.set_scale(16.f * CONSTANTS_ONE_G / 32768.f * gain_correction);
		_px4_accel.set_range(16.f * CONSTANTS_ONE_G);
		break;
	}
}

void BMI270::SetGyroScale()
{
	// data is 16 bits with 2000dps range
	const float scale = math::radians(2000.0f) / 32767.0f;
	_px4_gyro.set_scale(scale);

}

void BMI270::ConfigureSampleRate(int sample_rate)
{
	// round down to nearest FIFO sample dt * SAMPLES_PER_TRANSFER
	const float min_interval = FIFO_SAMPLE_DT;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	// works out to be 2 ...
	_fifo_gyro_samples = math::min((float)_fifo_empty_interval_us / (1e6f / RATE), (float)FIFO_MAX_SAMPLES);

	// recompute FIFO empty interval (us) with actual sample limit
	_fifo_empty_interval_us = _fifo_gyro_samples * (1e6f / RATE);

	ConfigureFIFOWatermark(_fifo_gyro_samples);
}


// when this register is set an interrupt is triggered when the FIFO reaches this many samples
void BMI270::ConfigureFIFOWatermark(uint8_t samples)
{
	// FIFO_WTM: 13 bit FIFO watermark level value
	// unit of the fifo watermark is one byte
	// each combined accel+gyro frame is 1 header byte + 6 gyro bytes + 6 accel bytes
	const uint16_t fifo_watermark_threshold = samples * (2 * sizeof(FIFO::Data) + FIFO::HEADER_SIZE);

	for (auto &r : _register_cfg) {
		if (r.reg == Register::FIFO_WTM_0) {
			// fifo_water_mark[7:0]
			r.set_bits = fifo_watermark_threshold & 0x00FF;
			r.clear_bits = ~r.set_bits;

		} else if (r.reg == Register::FIFO_WTM_1) {
			// fifo_water_mark[12:8]
			r.set_bits = (fifo_watermark_threshold & 0x0700) >> 8;
			r.clear_bits = ~r.set_bits;
		}
	}
}

bool BMI270::Configure()
{

	bool success = false;

	// check internal status first as per datasheet
	uint8_t internal_status = RegisterRead(Register::INTERNAL_STATUS);
	PX4_DEBUG("Internal status register value: 0x%02hhX", internal_status);

	if ((internal_status & 1) == 1) {
		PX4_DEBUG("INTERNAL_STATUS 0x01, ready for configure");

	} else {
		// ★ rm2027/fmu-h743 板级补丁（2026-09-30）：兼容件的 INTERNAL_STATUS(0x21) 恒读 0x00
		// （它是"微码已加载/自检完成"的状态位，而兼容件根本没有微码），
		// 走这个分支只会白白把 _state 置成 RESET（虽然随后被调用方覆盖）。
		// 兼容件下忽略这个检查，理由与"跳过微码上传"一致。
		if (_skip_microcode) {
			PX4_DEBUG("compatible part: ignoring INTERNAL_STATUS check");

		} else {
			PX4_DEBUG("INTERNAL_STATUS check failed, resetting");
			_state = STATE::RESET;
			ScheduleDelayed(10_ms);
		}
	};


	// ★ rm2027/fmu-h743 板级修正（2026-09-30）：厂商手册（华轩阳 HXY「ICM-42688P」）里
	//   有三条 upstream 没满足的约束，兼容件分支按手册顺序配置：
	//
	//   §5.45：「0x7D 要先写入 0x0E 并延时 10ms 之后再进行其他配置。另外，在配置完寄存器后
	//           建议延时 1ms 再进行寄存器读取操作。」
	//   §5.39：「GYR_NOISE_PERF 在切换配置时需要先将加速度计和陀螺仪关闭再进行配置，然后再
	//           打开加速度计和陀螺仪」—— 本驱动写的 GYR_CONF(0xEC) 正好带 GYR_NOISE_PERF=1，
	//           所以这一笔必须在【关闭状态】下写，否则该位可能根本进不去。
	//   §4.1 ：PWR_CONF(0x7C) 不在本片寄存器表里（本片没有这个寄存器），不要写它。
	//
	//   综合 §5.39 与 §5.45，顺序取：先关 → 关着写 GYR_CONF → 开(0x0E) → 10ms →
	//   写 ACC_CONF/ACC_RANGE → 1ms → 回读 + DATA_STAT 自查。
	//   正品 BMI270 仍走 upstream 原路径（下面的 else 分支），不引入回归。
	if (_skip_microcode) {

		// 1) 先把加速度计/陀螺仪关掉（上电/复位后本来就是关的，这里显式写一次，
		//    既满足 §5.39 的前提，也让每次（重）配置都从确定的初态开始）
		RegisterWrite(Register::PWR_CTRL, 0x00);
		px4_usleep(1_ms);   // 手册未给具体延时，取 1ms 保证关断生效

		// 2) 【关着】写 GYR_CONF（含 GYR_NOISE_PERF，手册 §5.39 要求）
		ApplyRegisterConfig(Register::GYR_CONF);

		// 3) 手册 §5.45：先写 PWR_CTRL = 0x0E（accel_en|gyr_en|temp_en），再延时 10ms
		RegisterSetAndClearBits(Register::PWR_CTRL,
					PWR_CTRL_BIT::accel_en | PWR_CTRL_BIT::gyr_en | PWR_CTRL_BIT::temp_en, 0);
		px4_usleep(10_ms);

		// 4) 其余配置（ACC_CONF / ACC_RANGE）。跳过：已经写过的 PWR_CTRL/GYR_CONF、
		//    本片不存在的 PWR_CONF，以及本片寄存器表里不存在的 FIFO_*/INT_* 地址
		//    （厂商手册：FIFO 在 0x1C~0x21、中断在 0x06/0x07，而本表用的是 BMI270 的
		//     0x46~0x49 / 0x53 / 0x58）。
		for (const auto &reg_cfg : _register_cfg) {
			if (reg_cfg.reg == Register::PWR_CONF
			    || reg_cfg.reg == Register::PWR_CTRL
			    || reg_cfg.reg == Register::GYR_CONF) {
				continue;
			}

			if (is_fifo_or_int_register(reg_cfg.reg)) {
				continue;
			}

			RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
		}

		// 5) 手册 §5.45：配完延时 1ms 再回读
		px4_usleep(1_ms);

		// 6) 自查：读 DATA_STAT(0x0B)（手册 §5.8），看器件有没有接受刚才的配置。
		//    这一步是判断"静止 1g 只读到 ~1185 counts（手册 2048）"到底是
		//    【配置没生效】还是【器件增益/零偏】问题的第一手证据 —— 上板先看这条日志。
		CheckCompatibleDataStat();

	} else {

		// first set and clear all configured register bits
		for (const auto &reg_cfg : _register_cfg) {
			RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
		}
	}

	// now check that all are configured

	for (const auto &reg_cfg : _register_cfg) {
		if (_skip_microcode && (is_fifo_or_int_register(reg_cfg.reg) || reg_cfg.reg == Register::PWR_CONF)) {
			// FIFO/INT 地址在本片上不存在；PWR_CONF(0x7C) 本片也没有、兼容件分支不再写它。
			// 对不存在的寄存器做回读核对没有意义，跳过（免得 _bad_register_perf 白涨）。
			continue;
		}

		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	SetAccelScaleAndRange();
	SetGyroScale();

	success = true;
	return success;
}

// ★ rm2027/fmu-h743（2026-09-30）兼容件专用：按 _register_cfg 里登记的"期望位/清除位"
// 写【单个】寄存器。手册时序路径要在不同时刻分别写不同寄存器（见 Configure 的 §5.39/§5.45），
// 用这个入口可以继续复用 _register_cfg 这一份配置常量，不必在别处再抄一份。
void BMI270::ApplyRegisterConfig(Register reg)
{
	for (const auto &reg_cfg : _register_cfg) {
		if (reg_cfg.reg == reg) {
			RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
			return;
		}
	}
}

// ★ rm2027/fmu-h743（2026-09-30）兼容件专用配置自查。
//
// 依据：厂商手册（华轩阳 HXY「ICM-42688P」）§5.8 DATA_STAT(0x0B)：
//   B5 GYR_CONF_ERR、B4 ACC_CONF_ERR —— 芯片自己在说"你给我的 ODR/BWP 组合我不接受"
//   （§5.37：非法组合会自动回落到缺省值 0xA8 并置 ACC_CONF_ERR；§5.39 陀螺同理）。
//
// 为什么值得每次配置后都读：本片已经观察到"配置回读是影子寄存器"（回读恒等于写入值，
// 但真实生效值可能不是），所以回读核对并不能证明配置生效；而这两个 ERR 位是【器件侧】的
// 判定。上板后如果 ACC_CONF_ERR/GYR_CONF_ERR 被置起来，就说明"静止 1g 只读到 ~1185 counts
// （手册应是 2048）"是【配置没生效】造成的，而不是器件增益/零偏问题 —— 那就不该动标定系数，
// 而应该先修配置时序。
void BMI270::CheckCompatibleDataStat()
{
	const uint8_t data_stat = RegisterRead(Register::DATA_STAT);
	_compat_data_stat = data_stat;

	const bool acc_err = (data_stat & DATA_STAT_BIT::acc_conf_err) != 0;
	const bool gyr_err = (data_stat & DATA_STAT_BIT::gyr_conf_err) != 0;

	if (acc_err || gyr_err) {
		// 配置没被器件接受：计入 _bad_register_perf（会在 sensor_*_status 的 error_count 里体现）
		perf_count(_bad_register_perf);
	}

	// 首次用 INFO（保证在 console 里看得见），之后降为 DEBUG，避免
	// "读失败 → Reset → 重新配置" 这种循环把 console 环形缓冲区刷满。
	if (!_compat_status_logged) {
		PX4_INFO("compatible part: DATA_STAT(0x0B)=0x%02x ACC_CONF_ERR=%d GYR_CONF_ERR=%d",
			 (unsigned)data_stat, (int)acc_err, (int)gyr_err);
		_compat_status_logged = true;

	} else {
		PX4_DEBUG("compatible part: DATA_STAT(0x0B)=0x%02x ACC_CONF_ERR=%d GYR_CONF_ERR=%d",
			  (unsigned)data_stat, (int)acc_err, (int)gyr_err);
	}
}

int BMI270::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	PX4_DEBUG("actual data ready interrupt called");
	static_cast<BMI270 *>(arg)->DataReady();
	return 0;
}

void BMI270::DataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool BMI270::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	return px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &DataReadyInterruptCallback, this) == 0;
}

bool BMI270::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

bool BMI270::RegisterCheck(const register_config_t &reg_cfg)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	PX4_DEBUG("0x%02hhX: 0x%02hhX", (uint8_t)reg_cfg.reg, reg_value);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	return success;
}

// ★★ rm2027/fmu-h743 板级补丁（2026-09-30）：本板器件的 SPI 读【无 dummy 字节】。
// 详见 BMI270.hpp 中该函数的完整说明，实测记录在 work/spi-runtime/。
//
// 线上读帧实际是：[cmd(0x80|reg)] [data0] [data1] ...
// 而本驱动按 BMI270 手册假设：[cmd] [dummy] [data0] [data1] ...
// 所以这里把收到的字节整体右移一格、dummy 位置填 0 ——
// 时钟数不变（send/recv/len 原样交给基类），只重排结果，
// 于是 RegisterRead / FIFOReadCount / FIFORead / UpdateTemperature 四条读路径全部无需改动。
int BMI270::transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	// 只拦截"接收 + 读操作"的调用；写操作（bit7=0）和只发不收（recv==nullptr）原样透传。
	// ★ 必须在调用基类之前判断 send[0]：in-place 调用时 send 就是 recv，会被结果覆盖。
	const bool is_read_transfer = (send != nullptr) && (recv != nullptr) && (len >= 2)
				      && ((send[0] & DIR_READ) != 0);

	if (!is_read_transfer) {
		return SPI::transfer(send, recv, len);
	}

	const int ret = SPI::transfer(send, recv, len);

	// [b0, d0, d1, ..., d(n-2)]  →  [b0, 0(dummy), d0, d1, ..., d(n-2)]
	// 倒序搬移，避免覆盖尚未读取的字节。
	for (int i = (int)len - 1; i >= 2; i--) {
		recv[i] = recv[i - 1];
	}

	recv[1] = 0;

	return ret;
}

// ★★ rm2027/fmu-h743 板级补丁（2026-09-30）：兼容件专用 —— 一次性突发读 0x0C~0x17 并发布。
//
// 字节序 / 轴向 / 量程的实测结论（依据 work/spi-runtime/ 下的 datadump.tcl、rangetest.tcl、
// cfgtest.tcl，全部是 ST-Link 手工驱动 SPI1 在 CPU halted 下读到的真机数据）：
//
// 1) 字节序 = 【高字节在前 MSB-first】，与 Bosch 手册写的 LSB-first 相反。
//    判定方法：同一次 CS 内从 0x0C / 0x0D / 0x0E 各起一个窗口读，只有"每轴高字节在前"
//    这一种解释能让三个窗口都得到物理自洽的小数值（Y≈0、Z≈+1180）。
//    LSB-first 解释会得到 ±2~3 万的数（>16g，静止不可能）。
//    例：A(0x0C,12)=FF CB 00 08 04 AD 00 00 00 1E FF FA
//        B(0x0D,11)=CD 00 5F 04 94 00 01 00 18 FF FC
//        ⇒ A: X=-53 Y=8 Z=1197 / B: X_LSB=CD Y=95 Z=1172（两窗口同构，自洽）
//
// 2) 轴序 = 与 BMI270 寄存器图一致：0x0C..0x11 = ACC X/Y/Z，0x12..0x17 = GYR X/Y/Z
//    （0x12..0x17 静止时三轴都只有个位数~几十 counts，明显是陀螺仪的零偏噪声，
//      若是加速度计应该是 ~1180 counts 量级的重力 ⇒ 轴序确认）。
//
// 3) 轴向/符号 = 原驱动约定：静止平放时第 3 对（0x10/0x11）读到 +1180 counts ≈ +1g
//    ⇒ 芯片 +Z 朝上。故这里和原 ProcessAccel/ProcessGyro 一样做 FLU→FRD 的 y、z 取反，
//    传 (x, -y, -z)。update() 内部还会做 rotate_3f(_rotation, ...)（与 updateFIFO 一致），
//    再乘 _scale 发布，所以这里【传原始码值】，不要自己乘 scale。
//    ⚠️ 前提：板子水平正放。若 QGC 里看到的静止 sensor_accel.z 是 +9.8 而不是 -9.8，
//       说明板/芯片实际是倒放的，需要把符号翻转（见笔记"遗留问题"）。
//
// 4) 量程 = ACC_RANGE=0x03（名义 16g）实际只有 ~1185 counts/g，已在
//    SetAccelScaleAndRange() 里用 ACCEL_COMPAT_GAIN_CORRECTION 做了单点增益标定。
bool BMI270::DataReadPublish(const hrt_abstime &timestamp_sample)
{
	DataReadBuffer buffer{};

	// 注意 len 必须传 sizeof(buffer) == 14（多读 1 个字节），原因见 DataReadBuffer 的声明注释：
	// 线上返回 13 字节(0x0C..0x18)，transfer() 补丁整体右移一格后 data[0..11] 才是 0x0C..0x17。
	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, sizeof(buffer)) != PX4_OK) {
		PX4_DEBUG("data register burst transfer failed");
		perf_count(_bad_transfer_perf);
		return false;
	}

	// 0x0C..0x11 = ACC X/Y/Z（高字节在前），0x12..0x17 = GYR X/Y/Z
	const int16_t accel_x = combine(buffer.data[0], buffer.data[1]);
	const int16_t accel_y = combine(buffer.data[2], buffer.data[3]);
	const int16_t accel_z = combine(buffer.data[4], buffer.data[5]);

	const int16_t gyro_x = combine(buffer.data[6], buffer.data[7]);
	const int16_t gyro_y = combine(buffer.data[8], buffer.data[9]);
	const int16_t gyro_z = combine(buffer.data[10], buffer.data[11]);

	// ★ rm2027/fmu-h743 板级保险（2026-09-30）：**重复样本抑制**。
	//
	// 为什么需要：轮询是【定时 1 kHz】的，而器件真实的有效 ODR 未必等于我们写的
	// ACC_CONF/GYR_CONF —— 本片已观测到"配置回读是影子寄存器、真实生效值不等于回读值"
	// （ACC_CONF 改 bwp 会让读数跳 2x；ACC_RANGE 在某些配置下看起来完全失效）。
	// 若 ODR 实际停在缺省档（本片 ACC_CONF 缺省 0xA8 主频 100Hz / GYR_CONF 缺省 0xA9 200Hz），
	// 1 kHz 轮询就会把【同一个样本重复发布 10 次】。
	// 对 EKF2 来说，这等于"dt>0 但测量值一个字节都不变" ⇒ 被当成零角速度/恒定加速度，
	// 会把陀螺零偏估计和姿态积分带偏 —— 比 1.73x 增益问题更危险。
	//
	// 做法：12 个原始字节与上一次**完全相同**就不发布。这样无论真实 ODR 是多少，
	// 发布频率都自动跟随器件实际更新率，永不重复；代价仅是极端静止时可能少发一两个样本。
	if ((buffer.data[0] == _last_data[0]) && (buffer.data[1] == _last_data[1]) &&
	    (buffer.data[2] == _last_data[2]) && (buffer.data[3] == _last_data[3]) &&
	    (buffer.data[4] == _last_data[4]) && (buffer.data[5] == _last_data[5]) &&
	    (buffer.data[6] == _last_data[6]) && (buffer.data[7] == _last_data[7]) &&
	    (buffer.data[8] == _last_data[8]) && (buffer.data[9] == _last_data[9]) &&
	    (buffer.data[10] == _last_data[10]) && (buffer.data[11] == _last_data[11])) {

		perf_count(_duplicate_sample_perf);
		return true;   // 不是错误：只是这一拍器件还没更新
	}

	for (int i = 0; i < 12; i++) {
		_last_data[i] = buffer.data[i];
	}

	// FLU → FRD（y、z 取反），与原 ProcessAccel / ProcessGyro 完全一致。
	// update() 接收的是【原始码值】：内部做 rotate_3f(_rotation,...) 再乘 _scale 发布。
	_px4_accel.update(timestamp_sample, (float)accel_x, (float)negate_clamped(accel_y), (float)negate_clamped(accel_z));
	_px4_gyro.update(timestamp_sample, (float)gyro_x, (float)negate_clamped(gyro_y), (float)negate_clamped(gyro_z));

	return true;
}

uint8_t BMI270::RegisterRead(Register reg)
{
	// 6.1.2 SPI interface of accelerometer part
	//
	// In case of read operations of the accelerometer part, the requested data
	// is not sent immediately, but instead first a dummy byte is sent, and
	// after this dummy byte the actual requested register content is transmitted.
	//   ⚠️ 本板器件【没有】这个 dummy 字节 —— 由上面的 BMI270::transfer() 补丁统一补齐，
	//      所以这里保持原样即可（cmd[2] 现在就是正确的寄存器内容）。
	uint8_t cmd[3] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	// cmd[1] dummy byte
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[2];
}

void BMI270::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void BMI270::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}


// Checks how many bytes are in the FIFO
uint16_t BMI270::FIFOReadCount()
{
	CheckErrorRegister();
	PX4_DEBUG("Attempting to determine FIFO fill level");

	// FIFO length registers FIFO_LENGTH_1 and FIFO_LENGTH_0 contain the 14 bit FIFO byte
	FIFOLengthReadBuffer buffer {};

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, sizeof(buffer)) != PX4_OK) {
		PX4_DEBUG("Bad transfer");
		perf_count(_bad_transfer_perf);
		return 0;
	}

	uint16_t fifo_fill_level = combine(buffer.FIFO_LENGTH_1 & 0x3F, buffer.FIFO_LENGTH_0);

	return fifo_fill_level;
}


// writes a gyro frame into the FIFO buffer the first argument points to
void BMI270::ProcessGyro(sensor_gyro_fifo_s *gyro, FIFO::Data *frame)
{
	const uint8_t samples = gyro->samples;

	const int16_t gyro_x = combine(frame->x_msb, frame->x_lsb);
	const int16_t gyro_y = combine(frame->y_msb, frame->y_lsb);
	const int16_t gyro_z = combine(frame->z_msb, frame->z_lsb);

	// Rotate from FLU to NED
	gyro->x[samples] = gyro_x;
	gyro->y[samples] = (gyro_y == INT16_MIN) ? INT16_MAX : -gyro_y;
	gyro->z[samples] = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;

	gyro->samples++;
}

// writes an accelerometer frame into the FIFO buffer the first argument points to
void BMI270::ProcessAccel(sensor_accel_fifo_s *accel, FIFO::Data *frame)
{

	const uint8_t samples = accel->samples;

	const int16_t accel_x = combine(frame->x_msb, frame->x_lsb);
	const int16_t accel_y = combine(frame->y_msb, frame->y_lsb);
	const int16_t accel_z = combine(frame->z_msb, frame->z_lsb);

	// Rotate from FLU to NED
	accel->x[samples] = accel_x;
	accel->y[samples] = (accel_y == INT16_MIN) ? INT16_MAX : -accel_y;
	accel->z[samples] = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;

	accel->samples++;
}


bool BMI270::FIFORead(const hrt_abstime &timestamp_sample, uint16_t fifo_bytes)
{

	uint8_t err = RegisterRead(Register::ERR_REG);

	if ((err >> 6 & 1) == 1) {
		FIFOReset();
		return false;
	}

	FIFOReadBuffer buffer{};

	// Reads from the FIFO_DATA register as much data as is available,
	// plus 2 bytes for the sent command and dummy byte.
	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, fifo_bytes + 2) != PX4_OK) {
		PX4_DEBUG("buffer transfer failed");
		perf_count(_bad_transfer_perf);
		return false;
	}

	sensor_accel_fifo_s accel_buffer{};
	accel_buffer.timestamp_sample = timestamp_sample;
	accel_buffer.dt = FIFO_SAMPLE_DT;

	sensor_gyro_fifo_s gyro_buffer{};
	gyro_buffer.timestamp_sample = timestamp_sample;
	gyro_buffer.dt = FIFO_SAMPLE_DT;

	uint8_t *data_buffer = (uint8_t *)&buffer.f[0];
	unsigned fifo_buffer_index = 0; // start of buffer

	while (fifo_buffer_index < fifo_bytes) {
		// look for header signature (first 6 bits) followed by two bits indicating the status of INT1 and INT2
		switch (data_buffer[fifo_buffer_index]) {
		case ((uint8_t)FIFO::Header::sensor_accel_frame | (uint8_t)FIFO::Header::sensor_gyro_frame): {
				// Move past header and padding for Aux
				fifo_buffer_index += 1;
				BMI270::ProcessGyro(&gyro_buffer, (FIFO::Data *)&data_buffer[fifo_buffer_index]);
				// Move forward to next record
				fifo_buffer_index += 6;
				BMI270::ProcessAccel(&accel_buffer, (FIFO::Data *)&data_buffer[fifo_buffer_index]);
				// Move forward to next record
				fifo_buffer_index += 6;
			}
			break;

		case (uint8_t)FIFO::Header::sensor_gyro_frame: {
				// Move past header.
				fifo_buffer_index += 1;
				BMI270::ProcessGyro(&gyro_buffer, (FIFO::Data *)&data_buffer[fifo_buffer_index]);
				// Move forward to next record
				fifo_buffer_index += 6;
			}
			break;

		case (uint8_t)FIFO::Header::sensor_accel_frame: {
				// Move past header.
				fifo_buffer_index += 1;
				BMI270::ProcessAccel(&accel_buffer, (FIFO::Data *)&data_buffer[fifo_buffer_index]);
				// Move forward to next record
				fifo_buffer_index += 6;
			}
			break;

		case (uint8_t)FIFO::Header::skip_frame:
			// Skip Frame
			// Frame length: 2 bytes (1 byte header + 1 byte payload)
			PX4_DEBUG("Skip Frame");
			fifo_buffer_index += 2;
			break;

		case (uint8_t)FIFO::Header::sensor_time_frame:
			// Sensortime Frame
			// Frame length: 4 bytes (1 byte header + 3 bytes payload)
			PX4_DEBUG("Sensortime Frame");
			fifo_buffer_index += 4;
			break;

		case (uint8_t)FIFO::Header::FIFO_input_config_frame:
			// FIFO input config Frame
			// Frame length: 2 bytes (1 byte header + 1 byte payload)
			PX4_DEBUG("FIFO input config Frame");
			fifo_buffer_index += 2;
			break;

		case (uint8_t)FIFO::Header::sample_drop_frame:
			// Sample drop Frame
			// Frame length: 2 bytes (1 byte header + 1 byte payload)
			PX4_DEBUG("Sample drop Frame");
			fifo_buffer_index += 2;
			break;

		default:
			fifo_buffer_index++;
			break;
		}
	}

	_px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				   perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));


	if ((accel_buffer.samples == 0) && (gyro_buffer.samples == 0)) {

		return false;

	} else {
		if (accel_buffer.samples > 0) {
			_px4_accel.updateFIFO(accel_buffer);
		}

		if (gyro_buffer.samples > 0) {
			_px4_gyro.updateFIFO(gyro_buffer);
		}

		return true;
	}

}



void BMI270::FIFOReset()
{
	PX4_DEBUG("Resetting FIFO...");
	perf_count(_fifo_reset_perf);

	// ACC_SOFTRESET: trigger a FIFO reset by writing 0xB0 to ACC_SOFTRESET (register 0x7E).
	RegisterWrite(Register::CMD, 0xB0);

	// reset while FIFO is disabled
	_drdy_timestamp_sample.store(0);
}

void BMI270::UpdateTemperature()
{
	uint8_t temperature_buf[4] {};
	temperature_buf[0] = static_cast<uint8_t>(Register::TEMP_MSB) | DIR_READ;

	if (transfer(&temperature_buf[0], &temperature_buf[0], sizeof(temperature_buf)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return;
	}

	// ★ rm2027/fmu-h743 板级更正（2026-09-30）：厂商手册 5.22 TEMP_H(22h)/TEMP_L(23h) 写明
	//   `T = (signed short)((X_H << 8) | X_L) / 512 + 23℃`
	//   —— 高字节在前。upstream 这里写成了 `buf[2] | (buf[3] << 8)`，高低字节反了
	//   （正品 BMI270 恰好也是高字节在前，所以这是 upstream 既有 bug，不是本板特有）。
	//   注：temperature_buf[2]/[3] 分别是 0x22/0x23 的内容，这一步已经被 BMI270::transfer()
	//   的无-dummy 补丁处理过，所以下标 2/3 对应寄存器 0x22/0x23 是对的。
	const uint16_t temp = (temperature_buf[2] << 8) | temperature_buf[3];

	float temperature;

	if (temp == 0x8000) {
		// invalid
		temperature = NAN;

	} else {
		constexpr float lsb = 0.001953125f; // 1/2^9
		temperature = 23.0f + (int16_t)temp * lsb;
	}

	_px4_accel.set_temperature(temperature);
	_px4_gyro.set_temperature(temperature);
}
