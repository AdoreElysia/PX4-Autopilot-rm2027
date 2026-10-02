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
 * @file HXY42688P.hpp
 *
 * Driver for the 华轩阳 (HuaXuanYang / HXY) "ICM-42688P" six-axis IMU module
 * connected via SPI.
 *
 * 唯一权威基准 = 厂商手册：
 *   《华轩阳HXY_ICM-42688P_六轴传感器模块_手册》
 *   projects/competition/rm2027-drone/references/硬件手册/
 *     华轩阳HXY_ICM-42688P_六轴传感器模块_手册.pdf（+ …_手册_文字提取.txt）
 *
 * 这是【独立的新驱动】，不是 Bosch BMI270 驱动的兼容分支：
 *   - 不共用 BMI270 的目录/类/寄存器图，也不含微码（microcode）
 *   - 器件识别：WHO_AM_I(0x01) == 0x6A（手册 §5.1；0x00 不是寄存器）
 *   - 初始化完全按手册时序（SOFT_RST → PWR_CTRL=0x0E → 稳定 → 量程/ODR），见 Configure()
 *   - 取数：查询方式轮询 0x0B~0x17，靠 DATA_STAT(0x0B) 的 DRDY 位判"新样本"（手册 §5.8）
 */

#pragma once

#include "HXY42688P_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace HXY42688P_REGS;

class HXY42688P : public device::SPI, public I2CSPIDriver<HXY42688P>
{
public:
	HXY42688P(const I2CSPIDriverConfig &config);
	~HXY42688P() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	void exit_and_cleanup() override;

	// 手册 §5.37/§5.39：目标工作模式 = 高性能模式（§2.4：ACC_EN=1, GYR_EN=1,
	// ACC_FILTER_PERF=1, GYR_FILTER_PERF=1, GYR_NOISE_PERF=1）；
	// ODR 取 1600Hz（accel 与 gyro 都合法：§5.37 BWP=000 允许 12.5Hz~1.6kHz；
	// §5.39 BWP=00 无保留限制；§5.44 陀螺自检步骤 3 也推荐 0x42=0x8C@1600Hz）。
	static constexpr uint32_t POLL_INTERVAL_US{1000};	// 1 kHz 轮询 = PX4 的 sensor_* 标准发布率
	static constexpr uint32_t SAMPLE_RATE_HZ{1000000 / POLL_INTERVAL_US};

	// 活体保护（取数路径）：连续这么多拍（拍 = 一次轮询 = 1 ms）没有拿到"新样本"，
	// 就判定器件已经停止转换 / 卡在同一帧上 ⇒ 计错误 + 递增 _failure_count + Reset() 重初始化。
	// 100 拍 @1kHz = 100 ms。
	static constexpr uint16_t STALE_SAMPLE_LIMIT{100};

	// 配置寄存器表：每一项给出"必须置 1 的位"和"必须清 0 的位"（RegisterCheck 语义）。
	// ⚠️ 所有值都来自手册，逐条注释了出处。
	static constexpr uint8_t size_register_cfg{5};

	struct register_config_t {
		Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	const register_config_t _register_cfg[size_register_cfg] {
		// 手册 §5.45：PWR_CTRL(0x7D) = B3 TEMP_EN | B2 ACC_EN | B1 GYR_EN = 0x0E（先写 0x0E 再延时 10ms）
		{ Register::PWR_CTRL,  PWR_CTRL_BIT::accel_gyr_temp_en, 0 },
		// 手册 §5.38：ACC_RANGE(0x41) = FS[1:0]；B7:B2 保留 ⇒ 必须先把 B1:B0 清掉再写 FS
		{ Register::ACC_RANGE, ACC_RANGE_BIT::acc_range_4g, ACC_RANGE_BIT::acc_range_mask },
		// 手册 §5.37 + §2.4：ACC_CONF(0x40) = 0x8C（高性能 / BWP=000 / ODR=1600Hz），8 位全部有定义
		{ Register::ACC_CONF,  ACC_CONF_BIT::acc_conf_high_perf_1600hz, 0xFF },
		// 手册 §5.40 + §5.44 步骤 2：GYR_RANGE(0x43) = FS[2:0]=000（±2000dps），且 B4 OIS_RANGE 必须为 0
		{ Register::GYR_RANGE, GYR_RANGE_BIT::gyr_fs_2000dps, GYR_RANGE_BIT::gyr_range_mask },
		// 手册 §5.39 + §2.4：GYR_CONF(0x42) = 0xCC（高性能 / 噪声优化 / BWP=00 / ODR=1600Hz），8 位全部有定义
		{ Register::GYR_CONF,  GYR_CONF_BIT::gyr_conf_high_perf_1600hz, 0xFF },
	};

	// 轮询读 0x0B~0x17（DATA_STAT + ACC X/Y/Z + GYR X/Y/Z）。
	//
	// 手册 §3.2.1「SPI 读」：读命令 16 个时钟，Bit8-15 就是 DO(7:0)（从 bit8 起器件驱动 SDO）
	// ⇒ 地址字节之后【立刻】是数据，线上没有 dummy 字节。而本驱动按 PX4 SPI 惯例在缓冲里
	// 声明一个 dummy 位置，由 transfer() 统一做"右移一格、dummy 填 0"的适配
	// （该适配已在实机验证：bad transfer = 0，读回值与手册缺省值一致）。
	//
	// 线上 = [cmd(0x8B)] [0x0B] [0x0C] … [0x17] [0x18]（15 字节），右移后：
	//   data[0]    = 0x0B DATA_STAT
	//   data[1..12] = 0x0C~0x17（每轴高字节在前，手册 §5.9~§5.14）
	struct DataReadBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::DATA_STAT) | DIR_READ};
		uint8_t dummy{0};
		uint8_t data[13] {};
	};

	static_assert(sizeof(DataReadBuffer) == 15);

	// 温度：手册 §5.22 TEMP_H(0x22)/TEMP_L(0x23)，高字节在前，T = (int16)raw/512 + 23℃
	struct TemperatureReadBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::TEMP_H) | DIR_READ};
		uint8_t dummy{0};
		uint8_t TEMP_H{0};
		uint8_t TEMP_L{0};
	};

	static_assert(sizeof(TemperatureReadBuffer) == 4);

	/**
	 * rm2027/fmu-h743 板级 SPI 适配：本器件【没有 dummy 字节】。
	 *
	 * 手册 §3.2.1：读命令 Bit8-15 就是数据 ⇒ [cmd] [data0] [data1] …
	 * 而 PX4 的 SPI 读路径按"先 dummy 再数据"取值，所以这里把收到的字节整体
	 * 右移一格、dummy 位置填 0。发出的时钟数与原来完全一致（不改线上时序），
	 * 只重排结果 ⇒ RegisterRead / 突发读 / 温度读三条路径都无需再各自处理。
	 * 写操作（bit7=0）与只发不收（recv==nullptr）原样透传。
	 *
	 * 注：device::SPI::transfer() 不是 virtual（src/lib/drivers/device/nuttx/SPI.hpp:119），
	 * 所以这里用同名成员函数隐藏基类版本 —— 驱动内部所有 transfer() 调用都解析到本函数，
	 * 线上时序仍然由基类实现发出（本函数只是先调基类、再重排结果）。
	 */
	int transfer(uint8_t *send, uint8_t *recv, unsigned len);

	int probe() override;

	bool Reset();

	// 手册标准配置流程（见 .cpp 顶部与 Configure() 内的逐条出处）
	bool Configure();

	// 按 _register_cfg 里登记的"期望位/清除位"写【单个】寄存器（读-改-写，总是写下去）
	void ApplyRegisterConfig(Register reg);

	// 按 _register_cfg 逐条回读核对（set_bits 必须全置位、clear_bits 必须全清）
	// 返回 false 表示至少一项不符，并把每一项的差异用 PX4_ERR 打到日志里
	bool VerifyRegisters();

	// 回读核对 _register_cfg[index] 一项；不符时 PX4_ERR + 计数（前 3 次每次都报，之后每 100 次一条）
	bool CheckRegisterConfig(uint8_t index);

	// 读 DATA_STAT(0x0B)（手册 §5.8），校验 B5 GYR_CONF_ERR / B4 ACC_CONF_ERR 是否为 0 并打印
	bool CheckDataStat();

	// 查询方式取数：一次突发读 0x0B~0x17，按 DATA_STAT 的 DRDY 位判新样本后发布
	bool DataReadPublish(const hrt_abstime &timestamp_sample);

	void SetAccelScaleAndRange();
	void SetGyroScale();

	void UpdateTemperature();

	uint8_t RegisterRead(Register reg);
	void RegisterWrite(Register reg, uint8_t value);

	const register_config_t &RegisterConfig(Register reg) const;

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	// 同一帧重发（12 字节数据与上一帧完全相同）——器件停转的现场证据；配合活体保护使用
	perf_counter_t _duplicate_sample_perf{perf_alloc(PC_COUNT, MODULE_NAME": duplicate sample")};
	// DATA_STAT 的 DRDY 位没同时置起（这一拍器件还没产出完整的新 accel+gyro 帧）
	perf_counter_t _no_new_data_perf{perf_alloc(PC_COUNT, MODULE_NAME": no new data")};
	// 连续 STALE_SAMPLE_LIMIT 拍没拿到新样本 ⇒ 判定器件停转
	perf_counter_t _stale_sample_perf{perf_alloc(PC_COUNT, MODULE_NAME": stale sample (device stalled)")};
	perf_counter_t _invalid_temperature_perf{perf_alloc(PC_COUNT, MODULE_NAME": invalid temperature")};

	hrt_abstime _temperature_update_timestamp{0};
	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};

	int _failure_count{0};
	uint16_t _consecutive_stale{0};	///< 连续"没有新样本"的轮询拍数（活体保护）

	uint8_t _last_data[12] {};	///< 上一次发布的 0x0C~0x17 原始字节（防同一帧重发）
	uint8_t _data_stat{0};		///< 最近一次读到的 DATA_STAT(0x0B)（手册 §5.8）

	uint8_t _checked_register{0};
	uint32_t _reg_fail_count[size_register_cfg] {};	///< 每个配置寄存器的累计回读失败次数
	bool _data_stat_logged{false};			///< DATA_STAT 首次打印用 PX4_ERR（保证可见），之后无错就降 DEBUG
	bool _verify_failed{false};			///< 最近一次 Configure() 是否有寄存器没写进去

	// ★ 临时诊断（GYR_CONF 写不进去的定位用）—— `hxy42688p status` 会打印这几个值。
	//   背景：ST-Link 手工写 0x42 任何值（含 0xCC）都能存住；但驱动跑完 Configure() 后
	//   0x42 仍是缺省 0xA9，而同一次 Configure() 里的 0x40/0x41/0x7D 都写成功。
	//   ⇒ 需要把"驱动到底写下去了什么"直接抓出来。
	uint8_t _dbg_gyr_set{0};
	uint8_t _dbg_gyr_clr{0};
	uint8_t _dbg_gyr_orig{0};
	uint8_t _dbg_gyr_wrote{0};
	uint8_t _dbg_gyr_rd_imm{0};
	uint8_t _dbg_gyr_rd_late{0};
	uint8_t _dbg_pwr_before{0};
	uint8_t _dbg_pwr_after{0};
	// ★ 追加（2026-09-30 深夜）：在"所有其它配置都写完之后"再补写一手 GYR_CONF，
	//   记录这一次写了什么、读回什么。ST-Link 实验（gyrconf-timing.tcl 变体 D）已证：
	//   PWR_CTRL 已是 0x0E 时写 0x42=0xCC 能稳稳存住。
	uint8_t _dbg_gyr_wrote2{0};
	uint8_t _dbg_gyr_rd_final{0};

	enum class STATE : uint8_t {
		RESET,			// 写 SOFT_RST(0x4A)=0xA5
		WAIT_FOR_RESET,		// 等 50ms，然后确认 WHO_AM_I(0x01)=0x6A
		POWER_ON,		// 写 PWR_CTRL(0x7D)=0x0E
		STABILIZE,		// 等 ≥250ms（陀螺通电稳定）
		CONFIGURE,		// 量程/ODR/滤波器配置 + 回读校验 + DATA_STAT 自查
		DATA_READ,		// 查询方式轮询取数
	} _state{STATE::RESET};
};
