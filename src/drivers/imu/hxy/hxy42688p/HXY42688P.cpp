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
 * @file HXY42688P.cpp
 *
 * 华轩阳 (HuaXuanYang / HXY) "ICM-42688P" 六轴 IMU 模块驱动（SPI）。
 *
 * ============================================================================
 * 唯一权威基准：厂商手册
 *   《华轩阳HXY_ICM-42688P_六轴传感器模块_手册》
 *   projects/competition/rm2027-drone/references/硬件手册/
 *     华轩阳HXY_ICM-42688P_六轴传感器模块_手册.pdf
 *     （文字提取版：华轩阳HXY_ICM-42688P_手册_文字提取.txt）
 * 本文件每条硬件行为后面都标了手册章节号（§x.y）。不用 BMI270 寄存器图做判断。
 * ============================================================================
 *
 * 【初始化流程（全部按手册，逐条出处）】
 *   §5.42 SOFT_RST(0x4A)=0xA5「复位整个电路，数据清零」 → 等 50ms
 *   §5.45 PWR_CTRL(0x7D)=0x0E（B3 TEMP_EN|B2 ACC_EN|B1 GYR_EN）→ 等 10ms
 *   §5.44 步骤1「使能陀螺仪 GYR_EN=1，并延时 250ms」；§电气参数 Ton@ODR=100Hz = 50ms
 *         ⇒ 通电稳定等 250ms（取手册里的上界）
 *   §5.39 备注「GYR_NOISE_PERF 在切换配置时需要先将加速度计和陀螺仪关闭再进行配置，
 *         然后再打开」⇒ 关断 → 写 GYR_CONF → 打开(0x0E) → 10ms
 *   §5.43 步骤3「配置 ACC_CONF 后延时 2ms」、§5.44 步骤3「配置 GYR_CONF(0x42) 后延时 2ms」
 *   §5.45 / §4.1 备注「配置完寄存器后建议延时 1ms 再进行寄存器读取操作」
 *   §5.37/§5.38 ACC_CONF=0x8C / ACC_RANGE=0x01（±4g）
 *   §5.39/§5.40 GYR_CONF=0xCC / GYR_RANGE=0x00（±2000dps）
 *   §2.4 目标工作模式 = 高性能模式（ACC_EN=1, GYR_EN=1, ACC_FILTER_PERF=1,
 *         GYR_FILTER_PERF=1, GYR_NOISE_PERF=1）
 *   §5.8 读 DATA_STAT(0x0B) 校验 B5 GYR_CONF_ERR / B4 ACC_CONF_ERR == 0
 *
 * 【取数】查询方式（手册 §3.2.1「SPI 读」+ §5.8 DRDY 状态位）：轮询 0x0B~0x17，
 *   用 DRDY_GYR/DRDY_ACC 判"三轴新数据全都转换完成"。不用 FIFO、不用中断。
 */

#include "HXY42688P.hpp"

#include <inttypes.h>
#include <math.h>
#include <string.h>

using namespace time_literals;

// 每轴高字节在前（手册 §5.9~§5.14 的 X_H/X_L 拼接方式）
static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

// FLU→FRD（y、z 取反）时防 INT16_MIN 溢出
static constexpr int16_t negate_clamped(int16_t v)
{
	return (v == INT16_MIN) ? INT16_MAX : -v;
}

HXY42688P::HXY42688P(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_px4_accel(get_device_id(), config.rotation, config.external),
	_px4_gyro(get_device_id(), config.rotation, config.external)
{
}

HXY42688P::~HXY42688P()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_duplicate_sample_perf);
	perf_free(_no_new_data_perf);
	perf_free(_stale_sample_perf);
	perf_free(_invalid_temperature_perf);
}

int HXY42688P::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool HXY42688P::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void HXY42688P::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();
}

void HXY42688P::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("query mode: polling 0x0B-0x17 every %u us (%u Hz); new-sample gate = DATA_STAT(0x0B) DRDY_GYR|DRDY_ACC",
		 (unsigned)POLL_INTERVAL_US, (unsigned)SAMPLE_RATE_HZ);

	PX4_INFO("DATA_STAT(0x0B)=0x%02x DRDY_ACC=%d DRDY_GYR=%d DRDY_TMP=%d ACC_CONF_ERR=%d GYR_CONF_ERR=%d",
		 (unsigned)_data_stat,
		 (int)((_data_stat & DATA_STAT_BIT::drdy_acc) != 0),
		 (int)((_data_stat & DATA_STAT_BIT::drdy_gyr) != 0),
		 (int)((_data_stat & DATA_STAT_BIT::drdy_tmp) != 0),
		 (int)((_data_stat & DATA_STAT_BIT::acc_conf_err) != 0),
		 (int)((_data_stat & DATA_STAT_BIT::gyr_conf_err) != 0));

	// 逐个回读"应当已被写上"的配置寄存器 —— 这是"配置到底进没进去"的第一手证据
	for (uint8_t i = 0; i < size_register_cfg; i++) {
		const register_config_t &reg_cfg = _register_cfg[i];
		const uint8_t reg_value = RegisterRead(reg_cfg.reg);

		const bool set_ok = (reg_value != 0xFF)
				    && ((reg_cfg.set_bits == 0) || ((reg_value & reg_cfg.set_bits) == reg_cfg.set_bits));

		// 判据只用 set_bits；clear_bits 仅表示"写入前先清掉这些位"，不能当读回判据
		// （见 CheckRegisterConfig() 的详细说明：0x41/0x40 曾被它误判）。
		PX4_INFO("0x%02x = 0x%02x  (want set 0x%02x; write used clear 0x%02x)  %s  [readback failures: %d]",
			 (unsigned)(uint8_t)reg_cfg.reg, (unsigned)reg_value,
			 (unsigned)reg_cfg.set_bits, (unsigned)reg_cfg.clear_bits,
			 set_ok ? "OK" : "MISMATCH",
			 (int)_reg_fail_count[i]);
	}

	PX4_INFO("failure_count=%d consecutive_stale=%u (limit %u polls = %u ms)",
		 _failure_count, (unsigned)_consecutive_stale, (unsigned)STALE_SAMPLE_LIMIT,
		 (unsigned)(STALE_SAMPLE_LIMIT * POLL_INTERVAL_US / 1000));

	// ★ 临时诊断：Configure() 里 GYR_CONF 那一步的完整过程
	PX4_INFO("GYR_CONF step: set=0x%02x clear=0x%02x | PWR_CTRL before=0x%02x after=0x%02x"
		 " | orig=0x%02x wrote=0x%02x | readback immediate=0x%02x late=0x%02x"
		 " | 补写 wrote2=0x%02x final=0x%02x",
		 (unsigned)_dbg_gyr_set, (unsigned)_dbg_gyr_clr,
		 (unsigned)_dbg_pwr_before, (unsigned)_dbg_pwr_after,
		 (unsigned)_dbg_gyr_orig, (unsigned)_dbg_gyr_wrote,
		 (unsigned)_dbg_gyr_rd_imm, (unsigned)_dbg_gyr_rd_late,
		 (unsigned)_dbg_gyr_wrote2, (unsigned)_dbg_gyr_rd_final);

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_duplicate_sample_perf);
	perf_print_counter(_no_new_data_perf);
	perf_print_counter(_stale_sample_perf);
	perf_print_counter(_invalid_temperature_perf);
}

int HXY42688P::probe()
{
	// 第一次寄存器访问（结果忽略）：与 PX4 其它 IMU 驱动一致，先发一次读命中总线/唤醒接口。
	RegisterRead(Register::WHO_AM_I);

	// 器件上电/接口就绪的余量时间（手册未规定具体值，这里取 PX4 同类驱动的 200us）
	px4_usleep(200);

	// 手册 §5.1 / §4.1：WHO_AM_I(0x01)，缺省值 0x6A。
	// ⚠️ 手册 §4.1 的通用寄存器表从 0x01 开始，0x00【不是】寄存器
	//    （BMI270 的 CHIP_ID=0x00 在本片上不存在，不能拿它做识别）。
	const uint8_t who_am_i = RegisterRead(Register::WHO_AM_I);

	if (who_am_i != WHO_AM_I_VALUE) {
		DEVICE_DEBUG("unexpected WHO_AM_I(0x01) 0x%02x (expected 0x%02x)", who_am_i, WHO_AM_I_VALUE);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void HXY42688P::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// 手册 §5.42：SOFT_RST(0x4A) 写入 0xA5 ——「复位整个电路，数据清零」
		RegisterWrite(Register::SOFT_RST, SOFT_RESET_CMD);
		_reset_timestamp = now;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(50_ms);	// 复位后等 50ms 再访问寄存器
		break;

	case STATE::WAIT_FOR_RESET: {
			// 手册 §5.1：复位完成后 WHO_AM_I(0x01) 应为 0x6A
			const uint8_t who_am_i = RegisterRead(Register::WHO_AM_I);

			if (who_am_i == WHO_AM_I_VALUE) {
				_state = STATE::POWER_ON;
				ScheduleDelayed(1_ms);

			} else if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				// 明确报出来（不用 DEBUG 掩盖）
				PX4_ERR("WHO_AM_I(0x01)=0x%02x after soft reset, expected 0x%02x - reset failed, retrying",
					(unsigned)who_am_i, (unsigned)WHO_AM_I_VALUE);
				_state = STATE::RESET;
				ScheduleDelayed(100_ms);

			} else {
				ScheduleDelayed(10_ms);
			}

			break;
		}

	case STATE::POWER_ON:
		// 手册 §5.45 备注：「0x7D 要先写入 0x0E 并延时 10ms 之后再进行其他配置」
		RegisterWrite(Register::PWR_CTRL, PWR_CTRL_BIT::accel_gyr_temp_en);
		_state = STATE::STABILIZE;
		ScheduleDelayed(10_ms);
		break;

	case STATE::STABILIZE:
		// 手册 §5.44 步骤1：「使能陀螺仪 GYR_EN = 1，并延时 250ms」
		// 手册 电气参数：开启时间 Ton @ODR=100HZ = 50ms
		// ⇒ 通电稳定取 250ms（手册给出的两种时间里的上界）
		_state = STATE::CONFIGURE;
		ScheduleDelayed(250_ms);
		break;

	case STATE::CONFIGURE: {
			const bool configured = Configure();

			_consecutive_stale = 0;
			_checked_register = 0;
			_last_config_check_timestamp = now;

			if (!configured) {
				// Configure() 内部已逐条用 PX4_ERR 打出差异；这里只说明后面的动作，
				// 仍然进入取数状态，好让现场能同时看到"配置回读值"和"实际数据"。
				PX4_ERR("configuration not fully accepted by the device - polling anyway for diagnosis");
			}

			_state = STATE::DATA_READ;
			ScheduleOnInterval(POLL_INTERVAL_US, POLL_INTERVAL_US);
			break;
		}

	case STATE::DATA_READ: {
			const bool published = DataReadPublish(now);

			if (published) {
				_consecutive_stale = 0;

				if (_failure_count > 0) {
					_failure_count--;
				}

				// 约 1 Hz 抽查一个配置寄存器（循环），确认配置没被器件丢掉
				if (hrt_elapsed_time(&_last_config_check_timestamp) > 1_s) {
					_last_config_check_timestamp = now;
					CheckRegisterConfig(_checked_register % size_register_cfg);
					_checked_register++;
				}

			} else {
				// 活体保护：这一拍没拿到新样本（DRDY 未同时置起，或数据与上一帧完全相同）。
				// 连续 STALE_SAMPLE_LIMIT 拍都没新样本 ⇒ 器件已停止转换 / 卡死在同一帧上。
				// ⚠️ 旧 bmi270 兼容补丁在这里是"重复就 return true"，结果把"器件死了"
				//    伪装成"器件很安静"（duplicate sample 累计 400 万次、error_count 恒 0、
				//    Reset() 永不触发）—— 本驱动不再这么做。
				_consecutive_stale++;

				if (_consecutive_stale > STALE_SAMPLE_LIMIT) {
					perf_count(_stale_sample_perf);
					_failure_count++;

					PX4_ERR("no new sample for %u consecutive polls (%u ms @ %u Hz): device stopped converting"
						" - re-initializing (cumulative stalls: %" PRIu64 ")",
						(unsigned)_consecutive_stale,
						(unsigned)(_consecutive_stale * POLL_INTERVAL_US / 1000),
						(unsigned)SAMPLE_RATE_HZ,
						perf_event_count(_stale_sample_perf));

					_px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf)
								   + perf_event_count(_stale_sample_perf));
					_px4_gyro.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf)
								  + perf_event_count(_stale_sample_perf));

					Reset();
					return;
				}
			}

			_px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf)
						   + perf_event_count(_stale_sample_perf));
			_px4_gyro.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf)
						  + perf_event_count(_stale_sample_perf));

			// 约 1 Hz 更新温度（手册 §5.22）
			if (hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {
				UpdateTemperature();
				_temperature_update_timestamp = now;
			}

			break;
		}
	}
}

// ============================================================================
// 配置流程（手册 §5.37~§5.45 + §2.4）
// ============================================================================
bool HXY42688P::Configure()
{
	// ---- 1) 手册 §5.39 备注：GYR_NOISE_PERF 必须在【加速度计和陀螺仪都关闭】时切换 ----
	//  「GYR_NOISE_PERF 在切换配置时需要先将加速度计和陀螺仪关闭再进行配置，然后再打开
	//    加速度计和陀螺仪，就可以将 GYR_NOISE_PERF 配置成功。」
	RegisterWrite(Register::PWR_CTRL, 0x00);	// ACC_EN=0, GYR_EN=0, TEMP_EN=0
	px4_usleep(1_ms);				// §5.45：配置完寄存器后建议延时 1ms 再进行后续操作

	// ★ 临时诊断：把这一步的"读-改-写"全过程记录下来（见 HXY42688P.hpp 的 _dbg_* 注释）
	{
		const register_config_t &cfg = RegisterConfig(Register::GYR_CONF);
		_dbg_gyr_set   = cfg.set_bits;
		_dbg_gyr_clr   = cfg.clear_bits;
		_dbg_pwr_before = RegisterRead(Register::PWR_CTRL);
		_dbg_gyr_orig  = RegisterRead(Register::GYR_CONF);
		_dbg_gyr_wrote = (uint8_t)((_dbg_gyr_orig & ~cfg.clear_bits) | cfg.set_bits);
		RegisterWrite(Register::GYR_CONF, _dbg_gyr_wrote);
		_dbg_gyr_rd_imm = RegisterRead(Register::GYR_CONF);
	}

	// ---- 2) 手册 §5.45：先把 PWR_CTRL 写成 0x0E，延时 10ms，之后再进行其他配置 ----
	RegisterWrite(Register::PWR_CTRL, PWR_CTRL_BIT::accel_gyr_temp_en);
	px4_usleep(10_ms);
	_dbg_pwr_after = RegisterRead(Register::PWR_CTRL);
	_dbg_gyr_rd_late = RegisterRead(Register::GYR_CONF);

	// ---- 3) 加速度计：手册 §5.43 步骤 2/3「设置量程 → 设置 ACC_CONF → 延时 2ms」----
	ApplyRegisterConfig(Register::ACC_RANGE);	// ±4g（§5.38 FS[1:0]=01）
	ApplyRegisterConfig(Register::ACC_CONF);	// 0x8C 高性能/1600Hz（§5.37 + §2.4）
	px4_usleep(2_ms);				// §5.43 步骤3

	// ---- 4) 陀螺仪量程：手册 §5.44 步骤2「设置陀螺仪量程，通常设置为 ±2000dps」----
	ApplyRegisterConfig(Register::GYR_RANGE);	// FS[2:0]=000（§5.40），B4 OIS_RANGE=0
	px4_usleep(2_ms);				// §5.44 步骤3

	// ---- 4b) ★ GYR_CONF 补写一手：在【所有其它配置都写完之后】再写一次 GYR_CONF ----
	//   为什么要有这一手（2026-09-30 深夜，ST-Link 实测）：
	//     · `gyrconf-timing.tcl` 把驱动的原时序（PWR_CTRL=0 → 写 0x42=0xCC → PWR_CTRL=0x0E）
	//       在芯片上手工重放，0x42 **存住了 0xCC**（变体 A/B/C/D 全部 KEEP）
	//       ⇒ "关断中写会被上电吃掉"这个假说**已被实验否证**。
	//     · 但驱动跑完 Configure() 后 0x42 恒为缺省 0xA9，而 0x40/0x41/0x7D 都写进去了。
	//     · ST-Link 变体 D 证明：PWR_CTRL 已是 0x0E 时写 0x42=0xCC 能稳稳存住。
	//   ⇒ 保留手册 §5.39 要求的"关断中配置 GYR_NOISE_PERF"，同时补这一手做兜底；
	//     两手的写值与读回都记进 _dbg_*，下一次 `status` 就能判决是哪一手生效。
	{
		const register_config_t &cfg = RegisterConfig(Register::GYR_CONF);
		const uint8_t orig = RegisterRead(Register::GYR_CONF);
		_dbg_gyr_wrote2 = (uint8_t)((orig & ~cfg.clear_bits) | cfg.set_bits);
		RegisterWrite(Register::GYR_CONF, _dbg_gyr_wrote2);
		px4_usleep(1_ms);
		_dbg_gyr_rd_final = RegisterRead(Register::GYR_CONF);
	}

	// ---- 5) 手册 §5.45/§4.1：配置完寄存器后延时 1ms 再进行寄存器读取操作 ----
	px4_usleep(1_ms);

	// ---- 6) 逐条回读校验（set_bits 必须全置位、clear_bits 必须全清）----
	const bool registers_ok = VerifyRegisters();

	// ---- 7) 读 DATA_STAT(0x0B) 校验 ACC_CONF_ERR / GYR_CONF_ERR（手册 §5.8）----
	const bool data_stat_ok = CheckDataStat();

	SetAccelScaleAndRange();
	SetGyroScale();

	const bool ok = registers_ok && data_stat_ok;
	_verify_failed = !ok;

	// ★ 临时诊断：把 GYR_CONF 那一手的全过程【直接打进启动日志】——
	//   `hxy42688p status` 也会打同一条，但现场 QGC 会持续抽干 console 未读窗口，
	//   而 SWD 抓 dmesg 环形缓冲区（checklog.sh）只能看到"写进去过的字节"。
	//   ⇒ 在 Configure() 里打一次，抓一次缓冲区就能看到，不必再依赖人工敲 status。
	PX4_ERR("GYR_CONF step: set=0x%02x clear=0x%02x | PWR_CTRL before=0x%02x after=0x%02x"
		" | orig=0x%02x wrote=0x%02x | readback immediate=0x%02x late=0x%02x"
		" | 补写 wrote2=0x%02x final=0x%02x",
		(unsigned)_dbg_gyr_set, (unsigned)_dbg_gyr_clr,
		(unsigned)_dbg_pwr_before, (unsigned)_dbg_pwr_after,
		(unsigned)_dbg_gyr_orig, (unsigned)_dbg_gyr_wrote,
		(unsigned)_dbg_gyr_rd_imm, (unsigned)_dbg_gyr_rd_late,
		(unsigned)_dbg_gyr_wrote2, (unsigned)_dbg_gyr_rd_final);

	if (!ok) {
		perf_count(_bad_register_perf);
		PX4_ERR("device did NOT accept the configuration (register readback %s, DATA_STAT(0x0B)=0x%02x)"
			" - see the per-register errors above / `hxy42688p status`",
			registers_ok ? "ok" : "FAILED", (unsigned)_data_stat);
	}

	return ok;
}

const HXY42688P::register_config_t &HXY42688P::RegisterConfig(Register reg) const
{
	for (const auto &reg_cfg : _register_cfg) {
		if (reg_cfg.reg == reg) {
			return reg_cfg;
		}
	}

	return _register_cfg[0];
}

// 读-改-写一个寄存器：val = (原值 & ~clear_bits) | set_bits，并且【总是写下去】
// （不像 upstream 那样"值没变就不写"——这里要的就是"每个该写的寄存器确实被写过"）。
void HXY42688P::ApplyRegisterConfig(Register reg)
{
	const register_config_t &reg_cfg = RegisterConfig(reg);
	const uint8_t orig_val = RegisterRead(reg);
	const uint8_t val = (orig_val & ~reg_cfg.clear_bits) | reg_cfg.set_bits;

	RegisterWrite(reg, val);
}

bool HXY42688P::VerifyRegisters()
{
	bool success = true;

	for (uint8_t i = 0; i < size_register_cfg; i++) {
		if (!CheckRegisterConfig(i)) {
			success = false;
		}
	}

	return success;
}

bool HXY42688P::CheckRegisterConfig(uint8_t index)
{
	const register_config_t &reg_cfg = _register_cfg[index];
	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	// ⚠️ clear_bits【不能】当作读回判据 —— 它的语义只是"写入前先清掉这些位"：
	//    (a) 同一位可以同时在 set_bits 和 clear_bits 里。例如 ACC_RANGE：set=0x01 / clear=0x03
	//        （选 FS[1:0]=01 之前要连 bit1 一起清），写对了也必然 (value & clear) != 0；
	//    (b) clear_bits 可能是 0xff（"整寄存器重写"），那任何非零值都会误报失败。
	//    实测（2026-09-30，rm2027 板，hxy42688p status）：
	//      0x41 = 0x01  ← 正是目标值，却被旧判据判成 MISMATCH（因为 0x01 & 0x03 != 0）
	//      0x40 = 0x8c  ← 正是目标值，却被判成 MISMATCH（因为 clear=0xff）
	//      ⇒ 旧判据把 5 条里的 3 条全判错，只有 0x42 是真的失败。
	//    正确判据 = set_bits 必须全部置上。
	// ⚠️ 再补一条（2026-09-30 实测）：读到 **0xFF** 时不能算通过 —— 0xFF 是“MISO 恒高 /
	//    读不到器件”的典型特征，而 `0xff & set_bits == set_bits` 恒成立，会把总线故障
	//    伪装成 OK（实测出现过 `0x42 = 0xff ... OK`、`0x43 = 0xff ... OK`）。
	const bool bus_error = (reg_value == 0xFF);	// MISO 恒高 ⇒ 这一拍没读到器件
	const bool set_ok = !bus_error
			    && ((reg_cfg.set_bits == 0) || ((reg_value & reg_cfg.set_bits) == reg_cfg.set_bits));

	if (set_ok) {
		return true;
	}

	_reg_fail_count[index]++;

	// ⚠️ 明确报出来，不用 DEBUG 掩盖 —— 已实测过 GYR_CONF 回读恒为缺省 0xA9（B6 没置上）
	//    并静默累计约 3000 次失败的教训。这里前 3 次每次都报，之后每 100 次报一条，
	//    消息里始终带着累计失败次数（计数不受限，信息不丢失）。
	const uint32_t failures = _reg_fail_count[index];

	if (failures <= 3 || (failures % 100) == 0) {
		PX4_ERR("0x%02x read back 0x%02x, wanted set 0x%02x (write used clear 0x%02x) (failures: %" PRIu32 ")%s",
			(unsigned)(uint8_t)reg_cfg.reg, (unsigned)reg_value,
			(unsigned)reg_cfg.set_bits, (unsigned)reg_cfg.clear_bits, failures,
			bus_error ? "  <== 0xFF: 这一拍根本没读到器件（总线故障，不是配置被拒）" : "");

		if (reg_cfg.reg == Register::GYR_CONF && !bus_error) {
			PX4_ERR("GYR_CONF(0x42) not accepted: 手册 §5.39 备注要求先关闭 ACC+GYR 再写 GYR_CONF"
				"（本驱动已按该顺序写，并在所有配置写完后补写一手），若仍失败说明器件不接受该组合");
		}
	}

	return false;
}

bool HXY42688P::CheckDataStat()
{
	// 手册 §5.8 DATA_STAT(0x0B)：B5 GYR_CONF_ERR / B4 ACC_CONF_ERR（0 = 初始化无误，1 = 初始化错误）
	const uint8_t data_stat = RegisterRead(Register::DATA_STAT);
	_data_stat = data_stat;

	const bool acc_err = (data_stat & DATA_STAT_BIT::acc_conf_err) != 0;
	const bool gyr_err = (data_stat & DATA_STAT_BIT::gyr_conf_err) != 0;

	if (acc_err || gyr_err || !_data_stat_logged) {
		PX4_ERR("DATA_STAT(0x0B)=0x%02x ACC_CONF_ERR=%d GYR_CONF_ERR=%d DRDY_ACC=%d DRDY_GYR=%d",
			(unsigned)data_stat, (int)acc_err, (int)gyr_err,
			(int)((data_stat & DATA_STAT_BIT::drdy_acc) != 0),
			(int)((data_stat & DATA_STAT_BIT::drdy_gyr) != 0));
		_data_stat_logged = true;

	} else {
		PX4_DEBUG("DATA_STAT(0x0B)=0x%02x ACC_CONF_ERR=0 GYR_CONF_ERR=0", (unsigned)data_stat);
	}

	return !(acc_err || gyr_err);
}

// ============================================================================
// 取数（查询方式）
// ============================================================================
bool HXY42688P::DataReadPublish(const hrt_abstime &timestamp_sample)
{
	DataReadBuffer buffer{};

	// 一次突发读 0x0B~0x17（DATA_STAT + ACC X/Y/Z + GYR X/Y/Z）。
	// len 必须是 sizeof(buffer)==15（见头文件 DataReadBuffer 说明：transfer() 会整体右移一格）。
	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, sizeof(buffer)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	const uint8_t data_stat = buffer.data[0];
	_data_stat = data_stat;

	// 手册 §5.8：B1 DRDY_GYR / B0 DRDY_ACC ——「三个轴新的数据全都转换完成」才为 1
	const bool drdy_acc = (data_stat & DATA_STAT_BIT::drdy_acc) != 0;
	const bool drdy_gyr = (data_stat & DATA_STAT_BIT::drdy_gyr) != 0;

	// 防"同一帧重发"：12 字节与上一帧完全相同 ⇒ 不是新样本。
	// ⚠️ 关键：这里【不再】像旧 bmi270 兼容补丁那样"重复就返回成功"，
	//    否则"器件死了"会被伪装成"器件很安静"。返回 false 会让活体保护开始计时。
	if (memcmp(&buffer.data[1], _last_data, sizeof(_last_data)) == 0) {
		perf_count(_duplicate_sample_perf);
		return false;
	}

	if (!drdy_acc || !drdy_gyr) {
		// 手册 §5.8：两个 DRDY 位同时为 1 才代表 accel+gyro 三轴新数据全都转换完成
		perf_count(_no_new_data_perf);
		return false;
	}

	memcpy(_last_data, &buffer.data[1], sizeof(_last_data));

	// 手册 §5.9~§5.14：每轴高字节在前，2 的补码
	const int16_t accel_x = combine(buffer.data[1], buffer.data[2]);
	const int16_t accel_y = combine(buffer.data[3], buffer.data[4]);
	const int16_t accel_z = combine(buffer.data[5], buffer.data[6]);

	const int16_t gyro_x = combine(buffer.data[7], buffer.data[8]);
	const int16_t gyro_y = combine(buffer.data[9], buffer.data[10]);
	const int16_t gyro_z = combine(buffer.data[11], buffer.data[12]);

	// FLU → FRD（y、z 取反）；传入的是【原始码值】，缩放由 PX4Accelerometer/PX4Gyroscope
	// 在 update() 内部按 _scale 完成（与 PX4 其它 IMU 驱动一致）。
	_px4_accel.update(timestamp_sample, (float)accel_x, (float)negate_clamped(accel_y), (float)negate_clamped(accel_z));
	_px4_gyro.update(timestamp_sample, (float)gyro_x, (float)negate_clamped(gyro_y), (float)negate_clamped(gyro_z));

	return true;
}

// ============================================================================
// 量程 / 缩放（手册 §5.38 / §5.40）
// ============================================================================
void HXY42688P::SetAccelScaleAndRange()
{
	// 手册 §5.38：FS[1:0] 00=±2G 01=±4G 10=±8G 11=±16G
	const uint8_t ACC_RANGE = RegisterRead(Register::ACC_RANGE) & ACC_RANGE_BIT::acc_range_mask;

	switch (ACC_RANGE) {
	case ACC_RANGE_BIT::acc_range_2g:
		_px4_accel.set_scale(2.f * CONSTANTS_ONE_G / 32768.f);
		_px4_accel.set_range(2.f * CONSTANTS_ONE_G);
		break;

	case ACC_RANGE_BIT::acc_range_4g:
		_px4_accel.set_scale(4.f * CONSTANTS_ONE_G / 32768.f);
		_px4_accel.set_range(4.f * CONSTANTS_ONE_G);
		break;

	case ACC_RANGE_BIT::acc_range_8g:
		_px4_accel.set_scale(8.f * CONSTANTS_ONE_G / 32768.f);
		_px4_accel.set_range(8.f * CONSTANTS_ONE_G);
		break;

	case ACC_RANGE_BIT::acc_range_16g:
	default:
		_px4_accel.set_scale(16.f * CONSTANTS_ONE_G / 32768.f);
		_px4_accel.set_range(16.f * CONSTANTS_ONE_G);
		break;
	}
}

void HXY42688P::SetGyroScale()
{
	// 手册 §5.40：FS[2:0] 000=±2000dps 001=±1000 010=±500 011=±250 100=±125
	const uint8_t GYR_RANGE = RegisterRead(Register::GYR_RANGE) & GYR_RANGE_BIT::gyr_fs_mask;

	float range_rad_s = math::radians(2000.f);	// 缺省 000 = ±2000dps

	switch (GYR_RANGE) {
	case GYR_RANGE_BIT::gyr_fs_1000dps:
		range_rad_s = math::radians(1000.f);
		break;

	case GYR_RANGE_BIT::gyr_fs_500dps:
		range_rad_s = math::radians(500.f);
		break;

	case GYR_RANGE_BIT::gyr_fs_250dps:
		range_rad_s = math::radians(250.f);
		break;

	case GYR_RANGE_BIT::gyr_fs_125dps:
		range_rad_s = math::radians(125.f);
		break;

	case GYR_RANGE_BIT::gyr_fs_2000dps:
	default:
		break;
	}

	_px4_gyro.set_range(range_rad_s);
	_px4_gyro.set_scale(range_rad_s / 32768.f);
}

// ============================================================================
// 温度（手册 §5.22）
// ============================================================================
void HXY42688P::UpdateTemperature()
{
	TemperatureReadBuffer buffer{};

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, sizeof(buffer)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return;
	}

	// 手册 §5.22：TEMP_H(0x22)/TEMP_L(0x23)，高字节在前，
	//            T = (signed short)((X_H << 8) | X_L) / 512 + 23℃
	const uint16_t temp = (buffer.TEMP_H << 8) | buffer.TEMP_L;

	float temperature;

	if (temp == 0x8000) {
		// 手册未定义"无效值"编码；这里沿用 PX4 惯例：0x8000 视为无效（置 NAN 并计数）
		perf_count(_invalid_temperature_perf);
		temperature = NAN;

	} else {
		temperature = 23.0f + (float)(int16_t)temp * (1.0f / 512.0f);
	}

	_px4_accel.set_temperature(temperature);
	_px4_gyro.set_temperature(temperature);
}

// ============================================================================
// 底层寄存器访问
// ============================================================================
/**
 * rm2027/fmu-h743 板级 SPI 读适配：本器件读时序【没有 dummy 字节】。
 *
 * 手册 §3.2.1「SPI 读」：SPI 读命令以 16 个时钟完成 —— Bit0 读写控制位(=1)，
 * Bit1-7 地址 AD(6:0)，Bit8-15 数据 DO(7:0)（MSB 先发送）。
 * ⇒ 地址字节之后【立刻】就是数据：线上是 [cmd] [data0] [data1] ...
 * 而 PX4 的 SPI 读路径按"先 dummy 再数据"取值。
 *
 * 处理：发出的时钟数不变（send/recv/len 原样交给基类），只把收到的字节
 * 整体右移一格、dummy 位置填 0。于是上方所有读路径（单寄存器 / 突发 / 温度）
 * 都按 [cmd][dummy][data...] 的布局取值即可。
 *
 * ⚠️ 必须在调用基类【之前】读 send[0]：in-place 调用时 send 就是 recv，会被结果覆盖。
 */
int HXY42688P::transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	const bool is_read_transfer = (send != nullptr) && (recv != nullptr) && (len >= 2)
				      && ((send[0] & DIR_READ) != 0);

	if (!is_read_transfer) {
		return SPI::transfer(send, recv, len);
	}

	const int ret = SPI::transfer(send, recv, len);

	// [b0, d0, d1, ..., d(n-2)] → [b0, 0(dummy), d0, d1, ..., d(n-2)]
	// 倒序搬移，避免覆盖尚未读取的字节。
	for (int i = (int)len - 1; i >= 2; i--) {
		recv[i] = recv[i - 1];
	}

	recv[1] = 0;

	return ret;
}

uint8_t HXY42688P::RegisterRead(Register reg)
{
	// 布局 [cmd][dummy][data] —— dummy 由 transfer() 填入，cmd[2] 就是寄存器内容
	uint8_t cmd[3] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[2];
}

void HXY42688P::RegisterWrite(Register reg, uint8_t value)
{
	// 写操作：bit7 = 0（手册 §3.2.1）
	uint8_t cmd[2] { static_cast<uint8_t>(reg), value };
	transfer(cmd, cmd, sizeof(cmd));
}
