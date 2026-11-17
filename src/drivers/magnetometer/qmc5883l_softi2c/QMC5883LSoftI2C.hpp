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
 * @file QMC5883LSoftI2C.hpp
 * @brief QMC5883L 磁罗盘，走**软件 I²C**（GPIO bit-bang）。
 *
 * 为什么单独写一个驱动，而不复用上游的 qmc5883l：
 *
 *   上游 src/drivers/magnetometer/qmc5883l/QMC5883L 继承 device::I2C，
 *   而 device::I2C 只认**硬件 I²C 总线号**（transfer() 最终落到 ioctl(I2CIOC_TRANSFER)）。
 *   本板（rm2027/fmu-h743）的 I²C4 全部焊点只有「IST8310 本体的 LGA-16 焊盘 +
 *   R20/R2 两颗 0402 电阻 + MCU 的 LQFP 引脚」，**没有任何测试点或接插件**
 *   （依据：原理图 fmu_2026-09-24.pdf 的 MAG 区块；
 *     work/FMU2-引脚识别-20261030/netlist-FMU_1.txt:113-114）。
 *   ⇒ 插座上接不了第二个硬件 I²C 器件。
 *
 *   而 PU-122Q GPS 模块自带的这颗 QMC5883L 只能从模块焊盘 C/D 引出来，
 *   落到某个空闲串口座上。串口座在 MCU 侧就是普通 GPIO，
 *   所以只能自己用 GPIO 造 I²C 时序。
 *
 *   这套路数与本仓库已有的 src/drivers/distance_sensor/vl53l0x_softi2c/ 完全一致，
 *   SoftI2C.{hpp,cpp} 直接复用（那个文件本来就是为"插座上做 I²C"写的）。
 *
 * QMC5883L 数据手册（QST QMC5883L Datasheet Rev A）给出的约束：
 *   - 支持标准模式 100 kHz / 快速模式 400 kHz，**没有最低频率要求**
 *     ⇒ bit-bang 慢一点完全合法。这里用 100 kHz（半个周期 5 µs）。
 *   - 上电必须先软复位（CNTL2 bit7 = SOFT_RST），等 POR 完成
 *     （手册 ≥ 5 ms，这里取 100 ms 余量，与上游驱动一致）。
 *   - 读数据前查 STATUS(0x06) 的 DRDY(bit0)。
 *   - SET/RESET Period(0x0B) 手册明确要求写 0x01。
 *   - CHIP_ID(0x0D) = 0xFF，**必须校验**（probe 里做），
 *     这样"起得来但认错芯片"这类问题不会重演（对比 af9838 不校验 ID 的坑）。
 *
 * 电气约定：
 *   两条线是**开漏 + 外部上拉**（模块板上自带 4.7 kΩ）。
 *   本驱动不开内部上拉 —— 也因此"空闲时能否读回高电平"就是接线自检判据。
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/softi2c/SoftI2C.hpp>
#include <drivers/drv_sensor.h>
#include "../qmc5883l/QST_QMC5883L_registers.hpp"

namespace rm2027
{

class QMC5883LSoftI2C : public px4::ScheduledWorkItem
{
public:
	enum class State : uint8_t {
		RESET = 0,
		WAIT_FOR_RESET,
		CONFIGURE,
		READ,
	};

	QMC5883LSoftI2C(SoftI2C &bus, enum Rotation rotation, bool external);
	~QMC5883LSoftI2C() override;

	/** 配器件：软复位 → 等 POR → 校验 CHIP_ID → 写配置。成功返回 PX4_OK。 */
	int init();

	/** 读一次三轴并发布到 sensor_mag。成功返回 PX4_OK。 */
	int read_and_publish();

	void print_status();

	State state() const { return _state; }

	static constexpr uint8_t I2C_ADDRESS = QST_QMC5883L::I2C_ADDRESS_DEFAULT;

	/** device_id 构造：软件 I²C 没有硬件总线号，用固定伪总线号 7 保证唯一。 */
	static uint32_t make_device_id();

	/** 采样率：QMC5883L 在 ODR=50Hz 档（CNTL1[3:2]=01）下输出 50 Hz。 */
	static constexpr uint32_t MEASUREMENT_RATE_HZ = 50;
	static constexpr hrt_abstime MEASUREMENT_INTERVAL_US = 1000000 / MEASUREMENT_RATE_HZ;

private:
	void Run() override;

	bool Reset();
	bool Configure();
	bool ReadRegisters(uint8_t first_reg, uint8_t *buf, size_t len);
	bool WriteRegister(uint8_t reg, uint8_t value);

	void error();

	SoftI2C		&_bus;
	PX4Magnetometer	_px4_mag;

	State		_state{State::RESET};
	hrt_abstime	_reset_timestamp{0};
	hrt_abstime	_last_publication{0};

	// perf 计数器：分别统计复位 / 坏寄存器 / 坏传输，便于上板定位
	perf_counter_t	_reset_perf;
	perf_counter_t	_bad_register_perf;
	perf_counter_t	_bad_transfer_perf;

	uint32_t	_failure_count{0};
};

} // namespace rm2027
