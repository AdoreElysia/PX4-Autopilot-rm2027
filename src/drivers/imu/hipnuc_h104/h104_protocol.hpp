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
 * @file h104_protocol.hpp
 * @brief HiPNUC H104 (HI04M0) 串口二进制协议 HI91 的解析 + 单位/坐标换算
 *
 * ★ 本文件**刻意不依赖任何 PX4 头文件**，就是为了能在主机上单元测试 ——
 *   见 `test/test_h104_protocol.cpp`，用真实抓包（98 帧 0 坏）做回归，
 *   **不用上板就能验证解析器**。
 *
 * 协议依据：`work/spi-runtime/H104-帧格式规格.md`（已上板验证）
 *   SOF 5A A5 | LEN u16 LE(1..512, 不含头/长度/CRC) | CRC u16 LE (CRC-16/XMODEM)
 *   | 数据域：tag 开头，HI91 tag=0x91，定长 76，整帧 82
 *
 * HI91 数据域（偏移相对数据域起点，全部小端）：
 *   0  tag u8 = 0x91
 *   1  main_status u16   ⚠️ 实测**不是**状态位（是 system_time 的确定性函数），本驱动不使用
 *   3  temperature i8    °C
 *   4  air_pressure f32  Pa   ⚠️ HI04M0 恒 0（无气压计），不使用
 *   8  system_time u32   ms
 *   12 acc_b  f32×3      **G**     载体系 RFU（X 右 / Y 前 / Z 上）
 *   24 gyr_b  f32×3      **deg/s** 载体系 RFU
 *   36 mag_b  f32×3      μT  ⚠️ HI04M0 恒 0（KF_EN_MAG=0），不使用
 *   48 roll  f32 deg / 52 pitch f32 deg / 56 yaw f32 deg
 *   60 quat  f32×4 WXYZ
 *
 * 输出统一换算成 PX4 要的形态：
 *   加速度 m/s²、角速度 rad/s、机体系 **FRD**（X 前 / Y 右 / Z 下）
 *   实测依据：静止时模块 acc_b=(0.017, 0.002, 1.003) G（Z 朝天）
 *   ⇒ FRD = [y, x, −z] 后 acc_z ≈ −9.81 m/s²，正是 PX4 水平静止的期望值。
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace hipnuc_h104
{

static constexpr uint8_t  kSof0 = 0x5Au;
static constexpr uint8_t  kSof1 = 0xA5u;
static constexpr uint8_t  kTagHi91 = 0x91u;
static constexpr uint16_t kHi91PayloadLen = 76u;
static constexpr uint16_t kMaxPayload = 512u;
static constexpr uint16_t kHi91FrameLen = 6u + kHi91PayloadLen;   /* 82 */

static constexpr float kStandardGravity = 9.80665f;
static constexpr float kDegToRad = 0.017453292519943295f;

/** CRC-16/XMODEM：poly 0x1021，init 0x0000，无输入/输出反转，无最终异或 */
inline uint16_t crc16_xmodem(const uint8_t *p, size_t n)
{
	uint16_t crc = 0x0000;

	for (size_t i = 0; i < n; i++) {
		crc ^= static_cast<uint16_t>(p[i] << 8);

		for (int b = 0; b < 8; b++) {
			crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
			      : static_cast<uint16_t>(crc << 1);
		}
	}

	return crc;
}

inline float read_f32(const uint8_t *p)
{
	union { uint32_t u; float f; } v;
	v.u = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8)
	      | (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
	return v.f;
}

inline uint32_t read_u32(const uint8_t *p)
{
	return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8)
	       | (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

/** 一帧解析结果，已经换算成 PX4 机体系/单位 */
struct Sample {
	uint32_t system_time_ms;      /**< 模块毫秒时间戳（未对时=本地计数） */
	float    temperature_c;
	float    acc_m_s2[3];         /**< 机体系 FRD, m/s² */
	float    gyro_rad_s[3];       /**< 机体系 FRD, rad/s */
};

struct Stats {
	uint32_t bytes;
	uint32_t frames;      /**< LEN 合法的候选帧 */
	uint32_t crc_ok;
	uint32_t crc_bad;
	uint32_t len_bad;
	uint32_t resync;      /**< 为找 SOF 跳过的字节数 */
	uint32_t hi91_ok;
};

class Parser
{
public:
	using Callback = void (*)(void *ctx, const Sample &sample);

	void setCallback(Callback cb, void *ctx) { _cb = cb; _ctx = ctx; }

	const Stats &stats() const { return _st; }
	void resetStats() { _st = Stats{}; }

	/**
	 * 喂入任意长度的字节流。内部维护跨调用的残存缓冲，**允许帧被任意切开**，
	 * 也允许单次喂入远大于内部缓冲（会自动分片，不丢帧）。
	 * 每解析出一帧合法的 HI91 就回调一次。
	 */
	void feed(const uint8_t *data, size_t len)
	{
		_st.bytes += static_cast<uint32_t>(len);

		while (len > 0) {
			scan();                                     /* 先消费掉完整帧，只留半帧尾巴 */

			const size_t room = sizeof(_acc) - _acc_len;
			const size_t take = (len < room) ? len : room;
			memcpy(_acc + _acc_len, data, take);
			_acc_len += take;
			data += take;
			len -= take;
		}

		scan();
	}

private:
	/** 从 _acc 里尽可能多地解析完整帧，保留尾部半帧 */
	void scan()
	{
		size_t i = 0;

		while (i + 6 <= _acc_len) {
			if (_acc[i] != kSof0 || _acc[i + 1] != kSof1) {
				i++;
				_st.resync++;
				continue;
			}

			const uint16_t plen = static_cast<uint16_t>(_acc[i + 2])
					      | (static_cast<uint16_t>(_acc[i + 3]) << 8);

			if (plen < 1 || plen > kMaxPayload) {
				_st.len_bad++;
				i += 2;                 /* 别只 +1，防止在 SOF 里空转 */
				continue;
			}

			const size_t total = 6u + plen;

			if (i + total > _acc_len) {
				break;                  /* 半帧，等下一批 */
			}

			const uint16_t crc_rx = static_cast<uint16_t>(_acc[i + 4])
						| (static_cast<uint16_t>(_acc[i + 5]) << 8);
			uint16_t crc_calc = crc16_xmodem(_acc + i, 4);
			crc_calc = crc16_xmodem_cont(crc_calc, _acc + i + 6, plen);

			_st.frames++;

			if (crc_rx != crc_calc) {
				_st.crc_bad++;

			} else {
				_st.crc_ok++;
				const uint8_t *payload = _acc + i + 6;

				if (plen == kHi91PayloadLen && payload[0] == kTagHi91) {
					_st.hi91_ok++;
					publish(payload);
				}
			}

			i += total;
		}

		if (i > 0) {
			memmove(_acc, _acc + i, _acc_len - i);
			_acc_len -= i;
		}
	}

	static uint16_t crc16_xmodem_cont(uint16_t crc, const uint8_t *p, size_t n)
	{
		for (size_t i = 0; i < n; i++) {
			crc ^= static_cast<uint16_t>(p[i] << 8);

			for (int b = 0; b < 8; b++) {
				crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
				      : static_cast<uint16_t>(crc << 1);
			}
		}

		return crc;
	}

	void publish(const uint8_t *p)
	{
		if (!_cb) { return; }

		Sample s{};
		s.system_time_ms = read_u32(p + 8);
		s.temperature_c = static_cast<float>(static_cast<int8_t>(p[3]));

		/* 载体系 RFU (X右/Y前/Z上) → PX4 机体系 FRD (X前/Y右/Z下)：[y, x, −z] */
		const float ax = read_f32(p + 12);      /* 右, G */
		const float ay = read_f32(p + 16);      /* 前, G */
		const float az = read_f32(p + 20);      /* 上, G */
		s.acc_m_s2[0] = ay * kStandardGravity;
		s.acc_m_s2[1] = ax * kStandardGravity;
		s.acc_m_s2[2] = -az * kStandardGravity;

		const float gx = read_f32(p + 24);      /* deg/s */
		const float gy = read_f32(p + 28);
		const float gz = read_f32(p + 32);
		s.gyro_rad_s[0] = gy * kDegToRad;
		s.gyro_rad_s[1] = gx * kDegToRad;
		s.gyro_rad_s[2] = -gz * kDegToRad;

		_cb(_ctx, s);
	}

	static constexpr size_t kAccSize = 1024;   /* > kMaxPayload + 6 */
	uint8_t  _acc[kAccSize] {};
	size_t   _acc_len{0};
	Stats    _st{};
	Callback _cb{nullptr};
	void    *_ctx{nullptr};
};

}  // namespace hipnuc_h104
