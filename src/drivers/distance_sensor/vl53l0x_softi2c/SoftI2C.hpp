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
 * @file SoftI2C.hpp
 * @brief 软件 I²C（bit-bang）主机 —— 只用两根普通 GPIO 就能读 I²C 器件。
 *
 * 为什么需要它：
 *   本板（rm2027/fmu-h743）**没有把任何一对可做 I²C 的引脚引到插座上**
 *   （唯一的 PC9 是 ESC4 的 DShot，且它的搭档 PA8 没引出）。
 *   要在"不焊芯片、只用现成插座"的前提下接 I²C 传感器，只能自己用 GPIO 造时序。
 *
 * 为什么这样做是安全的（VL53L0X 数据手册 DS11555 Rev6 Table 4）：
 *   - `FI2C` 工作频率 **最小 0 Hz** ⇒ 没有最低速度要求，慢没关系；
 *   - `tLOW ≥ 1.6 µs`、`tHIGH ≥ 0.6 µs` ⇒ 只有下限；
 *   - `tBUF`（两次传输之间的空闲）：L0X 手册写 **1.3 ms**，但同一芯片家族的 VL53L1X
 *     手册把**同一参数**写成 **1.3 µs**；而 ST 官方 API / 上游 PX4 驱动 / Pololu / Adafruit
 *     全都是"寄存器背靠背访问"且实测可用 ⇒ 判定为 L0X 手册的**单位笔误**。
 *     即使按 1.3 ms 解释也不影响本驱动：周期测量的轮询间隔是 5 ms。
 *   所以"软件拉线 + 忙等"完全落在器件允许的范围内，不需要定时器、不需要 DMA。
 *
 * 电气约定（重要）：
 *   两条线都配成 **开漏输出**（GPIO_OPENDRAIN）：写 1 = 释放（靠外部上拉拉高），
 *   写 0 = 拉低。**本驱动不开内部上拉** —— 上拉由传感器模块板载的 4.7 kΩ 提供。
 *   因此"空闲时能否读回高电平"同时就是"模块有没有插上/线有没有断"的自检判据
 *   （见 vl53l0x_softi2c test）。
 *
 * 可测试性：
 *   所有引脚操作都走 SoftI2CPins 抽象接口 ⇒ 主机侧单元测试可以塞一个
 *   mock（连一个假的 I²C 从机状态机）来验证时序，见 test_softi2c.cpp。
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace rm2027
{

/** 引脚抽象：真机实现走 px4_arch_configgpio / gpiowrite / gpioread。 */
class SoftI2CPins
{
public:
	virtual ~SoftI2CPins() = default;

	/** 一次性配置：两条线都配成开漏输出、初始释放（高）。 */
	virtual void configure() = 0;

	/** 写 1 = 释放（高电平由外部上拉决定），写 0 = 拉低。 */
	virtual void set_scl(bool high) = 0;
	virtual void set_sda(bool high) = 0;

	/** 读**引脚实际电平**（与方向无关，读的是 IDR）。 */
	virtual bool get_scl() = 0;
	virtual bool get_sda() = 0;

	/** 半个位周期。真机 = hrt 忙等；单测 = 空实现（不花时间）。 */
	virtual void delay_half_period() = 0;
};

class SoftI2C
{
public:
	explicit SoftI2C(SoftI2CPins &pins) : _pins(pins) {}

	SoftI2C(const SoftI2C &) = delete;
	SoftI2C &operator=(const SoftI2C &) = delete;

	/** 配引脚 + 总线释放检查。返回 PX4_OK / PX4_ERROR（PX4_ERROR 时 errors() 有原因）。 */
	int init();

	/** 是否有从机响应这个 7bit 地址（只发地址，不发数据）。 */
	bool probe(uint8_t address7);

	/** 写：START + addr+W + data[] + STOP */
	int write(uint8_t address7, const uint8_t *data, size_t len);

	/** 读：START + addr+R + data[] + STOP（最后一字节回 NACK） */
	int read(uint8_t address7, uint8_t *data, size_t len);

	/** 复合传输（读寄存器用）：START + addr+W + out[] + <重复 START> + addr+R + in[] + STOP */
	int write_read(uint8_t address7, const uint8_t *out, size_t out_len, uint8_t *in, size_t in_len);

	/** 释放总线（两条线都放开 + STOP），异常后复位用。 */
	void bus_release();

	/** 自检用：把两条线都拉低再放开，验证既能驱动、也能被外部上拉拉回。 */
	bool selftest_lines(bool *scl_ok, bool *sda_ok);

	/** 累计错误（NACK / 无应答 / SCL 被拉死）。 */
	uint32_t errors() const { return _errors; }
	uint32_t nacks() const { return _nacks; }

	/** 累计在线字节数（含地址字节与 ACK 拍之外的所有数据字节）—— 用来估软件 I²C 的 CPU 成本。 */
	uint32_t bytes() const { return _bytes; }

	/** 直接读两条线的实际电平（诊断用：不驱动总线、不产生波形） */
	bool scl_level() { return _pins.get_scl(); }
	bool sda_level() { return _pins.get_sda(); }

private:
	void    delay() { _pins.delay_half_period(); }
	void    start_condition();
	void    stop_condition();
	void    write_bit(bool bit);
	bool    read_bit();
	bool    write_byte(uint8_t byte);      /**< 返回 true = 收到 ACK */
	uint8_t read_byte(bool send_ack);      /**< send_ack=true 时本机回 ACK(0) */
	void    error();

	SoftI2CPins &_pins;
	uint32_t _errors{0};
	uint32_t _nacks{0};
	uint32_t _bytes{0};
};

} // namespace rm2027
