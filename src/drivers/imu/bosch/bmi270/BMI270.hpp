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

/**
 * @file BMI270.hpp
 *
 * Driver for the Bosch BMI270 connected via SPI.
 *
 */

#pragma once

#include "Bosch_BMI270_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace Bosch_BMI270;

class BMI270 : public device::SPI, public I2CSPIDriver<BMI270>
{
public:
	BMI270(const I2CSPIDriverConfig &config);
	~BMI270() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	void exit_and_cleanup() override;

	// Sensor Configuration
	static constexpr uint32_t RATE{1600}; // 1600 Hz
	static constexpr float FIFO_SAMPLE_DT{1e6f / RATE};

	// ★★ rm2027/fmu-h743 板级补丁（2026-09-30）：兼容件轮询读数据寄存器的周期 = 1000 us（1 kHz）。
	//
	// 为什么是 1 kHz：
	//   - 芯片 ODR 固定为 accel 1600 Hz / gyro 1600 Hz
	//     （见 _register_cfg 里的 ACC_CONF.acc_odr_1600 / GYR_CONF.gyr_odr_1k6）。
	//   - 1 ms = 【PX4 想要的输出频率】（sensor_gyro / sensor_accel 的标准 1 kHz 发布率），
	//     不是"随手硬编的 1 ms"。
	//   - 轮询周期(1000 us) ≥ 芯片 ODR 周期(625 us) ⇒ 每两次读之间芯片一定产出过新样本：
	//     永远不会重复读到同一个样本；因为比 ODR 慢，偶尔会【丢一个样本】。
	//     丢样本可接受（EKF 只看到 dt 略大），重复样本有害（dt>0 但数值不变 ⇒ 虚假的零角速度）。
	//   - 【不能】用 _fifo_empty_interval_us(1250 us)：那是 FIFO 批量传输的周期，
	//     "每周期读一批样本"的语义与"每周期读一个样本"不同。
	//   - 本驱动所在版本没有 PX4Gyroscope::get_required_data_rate()（只有 get_max_rate_hz()），
	//     所以不做动态跟随；1 kHz 是保守且够用的选择。
	static constexpr uint32_t POLL_INTERVAL_US{1000};

	// ★★ rm2027/fmu-h743 板级补丁（2026-09-30）：本板兼容件的加速度计【实测灵敏度修正】。
	//
	// 实测（work/spi-runtime/datadump.tcl + rangetest.tcl，板子静止平放、CPU halted）：
	//   ACC_RANGE 编码 → 静止 1g 对应的原始码值（|a| 均值）：
	//     0x00 → 9289.5        0x01 → 4648.0
	//     0x02 → 2316.0        0x03 → 1185.0（PX4 驱动用的就是 0x03 = 名义 16g，手册 2048 counts/g）
	//   严格 8:4:2:1 阶梯 ⇒ 量程寄存器本身工作正常，只是【绝对灵敏度比手册低 ~1.73x】。
	//   （1g 静止单点测量无法区分"增益低"与"零偏"——两者在单一加速度下数学等价；
	//     这里按增益做单点标定，使静止读数 = 1g。用 PX4 六面标定可进一步修偏移/增益。）
	// 修正系数 = 手册值 / 实测值 = 2048 / 1185 ≈ 1.7283。
	// 因为阶梯是严格 8:4:2:1，这个系数对 4 个量程都成立，故在 SetAccelScaleAndRange() 里统一相乘。
	// ⚠️ 只在 _skip_microcode（兼容件）时生效，正品 BMI270 走原公式，不受影响。
	static constexpr float ACCEL_COMPAT_GAIN_CORRECTION{2048.f / 1185.f};

	// _register_cfg 的前 5 项是本板兼容件【真正存在且能写】的寄存器
	// (PWR_CONF / PWR_CTRL / ACC_CONF / GYR_CONF / ACC_RANGE)；
	//
	// ⚠️ 2026-09-30 对照厂商数据手册（华轩阳 HXY「ICM-42688P」六轴惯性传感器模块手册，
	//    华轩阳电子，文件 F58A73F6B8D6319EEDDDDA2ED27CA8C9.pdf）后更正：
	//    本片寄存器表里【根本没有】0x46~0x49(FIFO_WTM/FIFO_CONFIG) 与 0x53/0x58(INT*)
	//    这几个地址 —— 本片的 FIFO 在 0x1C~0x21（FIFO_CFG0/1/2 = 0x1C/0x1D/0x1E、
	//    FIFO_STAT0/1 = 0x1F/0x20、FIFO_DATA = 0x21），中断配置在 INT_CFG1/2 = 0x06/0x07。
	//    这才是"0x48/0x49 写进去回读恒 0x00、0x24/0x25 恒 0"的真正原因：
	//    不是"这颗片子没有 FIFO"，而是原 BMI270 驱动写的是本片上【不存在的地址】。
	//    ⇒ 兼容件分支不再碰这些地址，也不参与周期性回读检查。
	static constexpr uint8_t size_register_cfg_compat{5};

	static constexpr uint8_t ID_088 = 0x1E;
	static constexpr uint8_t ID_090L = 0x1A;

	static constexpr int32_t FIFO_MAX_SAMPLES{math::min(FIFO::SIZE / sizeof(FIFO::Data), sizeof(sensor_accel_fifo_s::x) / sizeof(sensor_accel_fifo_s::x[0]))};


	hrt_abstime _temperature_update_timestamp{0};

	struct FIFOLengthReadBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::FIFO_LENGTH_0) | DIR_READ};
		uint8_t dummy{0};
		uint8_t FIFO_LENGTH_0{0};
		uint8_t FIFO_LENGTH_1{0};
	};

	struct FIFOReadBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::FIFO_DATA) | DIR_READ};
		uint8_t dummy{0};
		FIFO::Data f[FIFO_MAX_SAMPLES] {};
	};

	// ensure no struct padding
	static_assert(sizeof(FIFOReadBuffer) == (2 + FIFO_MAX_SAMPLES *sizeof(FIFO::Data)));

	// ★★ rm2027/fmu-h743 板级补丁（2026-09-30）：轮询读数据寄存器 0x0C~0x17 的突发读缓冲。
	//
	// ⚠️ 这里的 "dummy" 字节是【给 transfer() 补丁用的】，不是线上的真实时序：
	//   本板器件读时序无 dummy（线上是 [cmd][D0][D1]...），transfer() 补丁把收到的字节
	//   整体右移一格并把 recv[1] 置 0，所以本结构体必须按"cmd + dummy + 12 字节数据"声明。
	//
	// ⚠️ transfer 的 len 必须传 sizeof(DataReadBuffer) == 14，不能传 13：
	//   len=14：线上 = [cmd][0x0C..0x18]（13 字节），右移后 buffer[2..13] = 0x0C..0x17 ✓
	//           ⇒ data[0..11] 正好是 0x0C~0x17
	//   len=13：线上 = [cmd][0x0C..0x17]（12 字节），右移后 data[11] 被挤出丢掉、
	//           且 buffer[1] 变成补丁填的 0（dummy 而不是数据）⇒ 少读一个字节 ✗
	//   （这个 off-by-one 是新路最容易踩的坑，已在实机上验证过补丁的移位语义。）
	struct DataReadBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::DATA_0) | DIR_READ};
		uint8_t dummy{0};
		uint8_t data[12] {};  // 0x0C~0x17：ACC X/Y/Z + GYR X/Y/Z，每轴高字节在前
	};

	static_assert(sizeof(DataReadBuffer) == 14);

	struct register_config_t {
		Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	/**
	 * ★★ rm2027/fmu-h743 板级补丁（2026-09-30）：本板 IMU 的 SPI 读时序【没有 dummy 字节】。
	 *
	 * PX4 的 BMI270 驱动按 Bosch 数据手册实现读操作：
	 *     [cmd(0x80|reg)] [dummy] [data...]
	 * 所以 RegisterRead() 发 3 个字节、取第 3 个（FIFOLengthReadBuffer / FIFOReadBuffer /
	 * UpdateTemperature 里的 dummy 成员是同一个假设）。
	 *
	 * 本板器件的寄存器图与 BMI270 完全吻合（ACC_CONF=0x40 读 0xA8、GYR_CONF=0x42 读 0xA9、
	 * AUX_CONF=0x44、FIFO_DOWNS=0x45、SENSORTIME=0x18-0x1A、TEMP=0x22/0x23、PWR_CONF/CTRL/CMD
	 * = 0x7C/0x7D/0x7E，PWR_CTRL=0x0E 后 0x0C-0x17 出现实时数据），属于 BMI270 兼容件；
	 * 但它的读时序是【地址字节之后立刻就是数据】，没有 dummy：
	 *     [cmd] [data0] [data1] ...
	 *
	 * 实测证据（ST-Link 手工驱动 SPI1 外设寄存器，见 work/spi-fingerprint/run-align.log、
	 * work/spi-runtime/）：
	 *   1) 写 0x75=0x11 / 0x76=0x22 / 0x77=0x33，从 0x75 突发读回 → ff 11 22 33
	 *      ⇒ 第 2 个字节就是 0x75 的内容（若有 dummy，第 2 字节应是 dummy、第 3 字节才是 0x11）
	 *   2) 3 字节读 0x00 = 0x6a，而 2 字节读 0x01 也 = 0x6a
	 *      ⇒ 3 字节读的结果等于 R+1 的内容，即整串结果右移一格
	 *   3) 由此推出：RegisterRead(CHIP_ID=0x00) 实际取到的是 0x01 的内容(0x6a)，
	 *      probe() 必然失败 → 现象正是 "no instance started (no device on bus?)"。
	 *   4) 2026-09-30 拿到厂商数据手册（华轩阳 HXY「ICM-42688P」六轴惯性传感器模块手册）后
	 *      【正式确认】：手册 §3.2.1「SPI 读」写明读命令共 16 个时钟，Bit8-15 就是 DO(7:0)，
	 *      "如果是读，则电路会在 bit8 开始驱动 SDO" ⇒ 地址字节之后【立刻】是数据，
	 *      确实没有 dummy 字节。同时手册 §4.1 写明本片 WHO_AM_I 在 0x01、缺省值 0x6A
	 *      （0x00 不在手册寄存器表里）——上面第 2、3 条看到的 0x6A 就是 0x01 的内容，
	 *      所以这套"无 dummy + 右移补位"的补丁不是经验 hack，而是与手册时序一致。
	 *
	 * 这里【集中拦截读操作】：发出的时钟数与原来完全一致（不改变线上时序），
	 * 只把收到的字节数组右移一格、dummy 位置填 0。这样 BMI270.cpp 里的
	 * RegisterRead / FIFOReadCount / FIFORead / UpdateTemperature 四条读路径
	 * 一个字都不用改，写操作（bit7=0）与只发不收的调用原样透传。
	 */
	int transfer(uint8_t *send, uint8_t *recv, unsigned len);

	int probe() override;

	bool Reset();

	bool Configure();

	// ★ 兼容件：一次性突发读 0x0C~0x17 并发布 sensor_accel / sensor_gyro
	bool DataReadPublish(const hrt_abstime &timestamp_sample);

	void ProcessGyro(sensor_gyro_fifo_s *gyro, FIFO::Data *gyro_frame);
	void ProcessAccel(sensor_accel_fifo_s *accel, FIFO::Data *accel_frame);

	bool readAccelFrame(FIFO::Data *accel_frame);
	bool readGyroFrame(FIFO::Data *gyro_frame);

	void ConfigurePwr();
	void SetAccelScaleAndRange();
	void SetGyroScale();

	void CheckErrorRegister();

	void ConfigureFifo();
	void ConfigureSampleRate(int sample_rate = 0);
	void ConfigureFIFOWatermark(uint8_t samples);

	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();

	bool RegisterCheck(const register_config_t &reg_cfg);

	uint8_t RegisterRead(Register reg);
	void RegisterWrite(Register reg, uint8_t value);
	void RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits);

	uint16_t FIFOReadCount();
	//bool FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples);
	bool FIFORead(const hrt_abstime &timestamp_sample, uint16_t fifo_bytes);
	void FIFOReset();

	void UpdateTemperature();

	const spi_drdy_gpio_t _drdy_gpio;

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _fifo_empty_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO empty")};
	perf_counter_t _fifo_overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO overflow")};
	perf_counter_t _fifo_reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO reset")};
	// ★ rm2027/fmu-h743：轮询路径的重复样本抑制（见 DataReadPublish 的注释）。
	//   这个计数如果长期快速上涨，就说明器件真实 ODR 远低于 1 kHz 的轮询频率
	//   （即 ACC_CONF/GYR_CONF 没真正生效）—— 是个很好的现场诊断信号。
	perf_counter_t _duplicate_sample_perf{perf_alloc(PC_COUNT, MODULE_NAME": duplicate sample")};

	uint8_t _last_data[12] {};   ///< 上一次读到的 0x0C~0x17 原始字节，用于重复样本抑制

	perf_counter_t _drdy_missed_perf{nullptr};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	int _failure_count{0};

	px4::atomic<hrt_abstime> _drdy_timestamp_sample{0};
	bool _data_ready_interrupt_enabled{false};
	// ★ rm2027/fmu-h743 板级补丁（2026-09-30）：
	// 本板的 6 轴 IMU 是【BMI270 寄存器图兼容件】，CHIP_ID(0x00) 读出 0x00 而非正品 0x24。
	// 实测：它的寄存器图/功能与 BMI270 一致，但【不能】上传 329 字节微码 ——
	//       一旦上传，器件会锁死（读全 0xFF），只能断电恢复。
	// 故：仅当识别到这种"ID=0x00 的兼容件"时，跳过微码上传。
	// 依据：rm2027-drone/firmware/fmu-h743/开发日志/09-IMU寄存器指纹.md
	bool _skip_microcode{false};

	// ★ rm2027/fmu-h743 板级补丁（2026-09-30）：兼容件取数改走 STATE::DATA_READ
	// （定时轮询直接读 0x0C~0x17）——厂商手册明确写了"加速度计和陀螺仪测量数据
	//  以中断方式或【查询方式】获取"，所以轮询是被手册支持的用法。
	//
	// ⚠️ 注意（对照厂商手册后的更正）：本片其实【有】FIFO，只是地址与 BMI270 不同
	//   （FIFO_CFG0/1/2 = 0x1C/0x1D/0x1E，FIFO_STAT0/1 = 0x1F/0x20，FIFO_DATA = 0x21，
	//    共 2048 Byte = 341 组六轴数据；默认是 By-Pass 旁路模式）。
	//   BMI270 的 FIFO 地址(0x46~0x49 / 0x24~0x26)在本片上不存在 ⇒ 原 FIFO 路径
	//   拿不到数据是"地址不对"，不是"没有 FIFO"。
	//   FIFO 代码完整保留：换回正品 BMI270（CHIP_ID=0x24）时 _skip_microcode 为 false，
	//   自动走回原路径，不需要回滚代码。
	enum class STATE : uint8_t {
		WAIT_FOR_RESET,
		RESET,
		MICROCODE_LOAD,
		CONFIGURE,
		DATA_READ,   // ★ 兼容件专用：定时轮询读数据寄存器 0x0C~0x17（手册支持的"查询方式"）
		FIFO_READ,   // 正品 BMI270：原来的 FIFO 批量读路径（本板兼容件不走）
	} _state{STATE::RESET};

	// "Skipping BMI270 microcode upload" 只打印一次：
	// 兼容件每次 Reset 都会重新走到 MICROCODE_LOAD，如果每次都 PX4_INFO
	// 会把 console 环形缓冲区刷满（历史上正是这个刷屏现象掩盖了真正的报错）。
	bool _microcode_skip_logged{false};

	uint16_t _fifo_empty_interval_us{1250}; // default 1250 us / 800 Hz transfer interval
	int32_t _fifo_gyro_samples{static_cast<int32_t>(_fifo_empty_interval_us / (1000000 / RATE))}; // checks out to be 2 ...

	uint8_t _checked_register{0};
	static constexpr uint8_t size_register_cfg{11};
	register_config_t _register_cfg[size_register_cfg] {
		// Register                        | Set bits, Clear bits
		{ Register::PWR_CONF,          0, ACC_PWR_CONF_BIT::acc_pwr_save },
		{ Register::PWR_CTRL,          PWR_CTRL_BIT::accel_en | PWR_CTRL_BIT::gyr_en | PWR_CTRL_BIT::temp_en,  0 },
		{ Register::ACC_CONF,              ACC_CONF_BIT::acc_bwp_Normal | ACC_CONF_BIT::acc_odr_1600, Bit1 | Bit0 },
		{ Register::GYR_CONF,              GYR_CONF_BIT::gyr_odr_1k6 | GYR_CONF_BIT::gyr_flt_mode_normal | GYR_CONF_BIT::gyr_noise_hp | GYR_CONF_BIT::gyr_flt_hp, Bit0 | Bit1 | Bit4},
		{ Register::ACC_RANGE,             ACC_RANGE_BIT::acc_range_16g, 0 },
		{ Register::FIFO_WTM_0,            0, 0 },
		{ Register::FIFO_WTM_1,            0, 0 },
		{ Register::FIFO_CONFIG_0,         FIFO_CONFIG_0_BIT::BIT1_ALWAYS | FIFO_CONFIG_0_BIT::FIFO_mode, 0 },
		{ Register::FIFO_CONFIG_1,         FIFO_CONFIG_1_BIT::BIT4_ALWAYS | FIFO_CONFIG_1_BIT::Acc_en | FIFO_CONFIG_1_BIT::Gyr_en, 0 },
		{ Register::INT1_IO_CTRL,          INT1_IO_CONF_BIT::int1_out, 0 },
		{ Register::INT_MAP_DATA,    INT1_INT2_MAP_DATA_BIT::int1_fwm, 0},
	};
};
