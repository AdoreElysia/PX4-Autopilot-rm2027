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
 * @file SoftI2C.cpp
 * @brief 软件 I²C（bit-bang）主机实现。本文件**不依赖任何 PX4 头**，
 *        以便主机侧单测直接编译（见 test_softi2c.cpp）。
 */

#include "SoftI2C.hpp"

namespace rm2027
{

void SoftI2C::error()
{
	_errors++;
}

void SoftI2C::start_condition()
{
	_pins.set_sda(true);
	delay();
	_pins.set_scl(true);
	delay();

	if (!_pins.get_scl()) {
		/* SCL 拉不上去：要么总线被从机拉住，要么上拉缺失/线断了 */
		error();
	}

	_pins.set_sda(false);   /* SCL 高时 SDA 高→低 = START */
	delay();
	_pins.set_scl(false);
	delay();
}

void SoftI2C::stop_condition()
{
	_pins.set_sda(false);
	delay();
	_pins.set_scl(true);
	delay();
	_pins.set_sda(true);    /* SCL 高时 SDA 低→高 = STOP */
	delay();
}

void SoftI2C::bus_release()
{
	_pins.set_sda(true);
	_pins.set_scl(true);
	delay();
}

void SoftI2C::write_bit(bool bit)
{
	_pins.set_sda(bit);
	delay();
	_pins.set_scl(true);
	delay();
	_pins.set_scl(false);
	delay();
}

bool SoftI2C::read_bit()
{
	_pins.set_sda(true);    /* 释放 SDA，交给从机驱动 */
	delay();
	_pins.set_scl(true);
	delay();

	const bool bit = _pins.get_sda();

	_pins.set_scl(false);
	delay();
	return bit;
}

bool SoftI2C::write_byte(uint8_t byte)
{
	_bytes++;

	for (int i = 7; i >= 0; i--) {
		write_bit(((byte >> i) & 0x1) != 0);
	}

	/* 第 9 拍：本机释放 SDA，从机拉低表示 ACK */
	_pins.set_sda(true);
	delay();
	_pins.set_scl(true);
	delay();

	const bool ack = !_pins.get_sda();

	_pins.set_scl(false);
	delay();
	return ack;
}

uint8_t SoftI2C::read_byte(bool send_ack)
{
	_bytes++;

	uint8_t byte = 0;

	_pins.set_sda(true);

	for (int i = 7; i >= 0; i--) {
		delay();
		_pins.set_scl(true);
		delay();
		byte = static_cast<uint8_t>((byte << 1) | (_pins.get_sda() ? 1 : 0));
		_pins.set_scl(false);
		delay();
	}

	/* 第 9 拍：本机回 ACK(拉低) 或 NACK(释放) */
	_pins.set_sda(!send_ack);
	delay();
	_pins.set_scl(true);
	delay();
	_pins.set_scl(false);
	delay();
	_pins.set_sda(true);

	return byte;
}

int SoftI2C::init()
{
	_pins.configure();

	/* 空闲检查：两条线都该被**模块上的上拉**拉高。
	 * 读不到高 ⇒ 模块没插 / 线断 / 被短路（本驱动刻意不开内部上拉）。 */
	delay();

	if (!_pins.get_scl() || !_pins.get_sda()) {
		error();
		return -1;
	}

	return 0;
}

bool SoftI2C::selftest_lines(bool *scl_ok, bool *sda_ok)
{
	/* SCL：先拉低看能否读到低，再放开看能否回到高 */
	_pins.set_scl(false);
	delay();
	const bool scl_low = !_pins.get_scl();
	_pins.set_scl(true);
	delay();
	const bool scl_high = _pins.get_scl();

	*scl_ok = scl_low && scl_high;

	_pins.set_sda(false);
	delay();
	const bool sda_low = !_pins.get_sda();
	_pins.set_sda(true);
	delay();
	const bool sda_high = _pins.get_sda();

	*sda_ok = sda_low && sda_high;

	return scl_low && scl_high && sda_low && sda_high;
}

bool SoftI2C::probe(uint8_t address7)
{
	start_condition();
	const bool ack = write_byte(static_cast<uint8_t>((address7 << 1) | 0x0));
	stop_condition();

	if (!ack) {
		_nacks++;
	}

	return ack;
}

int SoftI2C::write(uint8_t address7, const uint8_t *data, size_t len)
{
	start_condition();

	if (!write_byte(static_cast<uint8_t>((address7 << 1) | 0x0))) {
		_nacks++;
		error();
		stop_condition();
		return -1;
	}

	for (size_t i = 0; i < len; i++) {
		if (!write_byte(data[i])) {
			_nacks++;
			error();
			stop_condition();
			return -1;
		}
	}

	stop_condition();
	return 0;
}

int SoftI2C::read(uint8_t address7, uint8_t *data, size_t len)
{
	start_condition();

	if (!write_byte(static_cast<uint8_t>((address7 << 1) | 0x1))) {
		_nacks++;
		error();
		stop_condition();
		return -1;
	}

	for (size_t i = 0; i < len; i++) {
		data[i] = read_byte(i + 1 < len);   /* 最后一字节回 NACK */
	}

	stop_condition();
	return 0;
}

int SoftI2C::write_read(uint8_t address7, const uint8_t *out, size_t out_len, uint8_t *in, size_t in_len)
{
	start_condition();

	if (!write_byte(static_cast<uint8_t>((address7 << 1) | 0x0))) {
		_nacks++;
		error();
		stop_condition();
		return -1;
	}

	for (size_t i = 0; i < out_len; i++) {
		if (!write_byte(out[i])) {
			_nacks++;
			error();
			stop_condition();
			return -1;
		}
	}

	/* 重复 START（不发 STOP，保持对总线的占用） */
	_pins.set_sda(true);
	delay();
	_pins.set_scl(true);
	delay();

	if (!_pins.get_scl()) {
		error();
	}

	_pins.set_sda(false);
	delay();
	_pins.set_scl(false);
	delay();

	if (!write_byte(static_cast<uint8_t>((address7 << 1) | 0x1))) {
		_nacks++;
		error();
		stop_condition();
		return -1;
	}

	for (size_t i = 0; i < in_len; i++) {
		in[i] = read_byte(i + 1 < in_len);
	}

	stop_condition();
	return 0;
}

} // namespace rm2027
