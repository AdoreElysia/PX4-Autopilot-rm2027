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

#include "H104IMU.hpp"

#include <inttypes.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

/* 每轮读多少个字节 / 多久跑一次：
 * 115200 8N1 下 HI91@100Hz = 8200 B/s；NuttX 的 USART1 RX 缓冲是 600 字节（≈73 ms）。
 * 2 ms 一轮 ⇒ 最坏积压约 16 字节，余量 ~35 倍，不可能溢出。 */
static constexpr uint32_t kRunIntervalUs = 2000;
static constexpr int      kReadsPerRun   = 4;
static constexpr size_t   kReadChunk     = 256;

H104IMU::H104IMU(const char *port, uint32_t device_id, enum Rotation rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::INS0),
	_port(port),
	_device_id(device_id),
	_rotation(rotation),
	_px4_accel(device_id, rotation, true /* external：外挂模块 */),
	_px4_gyro(device_id, rotation, true /* external */),
	_perf_frames(perf_alloc(PC_COUNT, "h104_imu: frames")),
	_perf_crc_bad(perf_alloc(PC_COUNT, "h104_imu: crc bad")),
	_perf_len_bad(perf_alloc(PC_COUNT, "h104_imu: len bad")),
	_perf_resync(perf_alloc(PC_COUNT, "h104_imu: resync bytes")),
	_perf_read_errors(perf_alloc(PC_COUNT, "h104_imu: read errors")),
	_perf_reopens(perf_alloc(PC_COUNT, "h104_imu: port reopens")),
	_perf_overflow(perf_alloc(PC_COUNT, "h104_imu: rx overflow")),
	_start_time(hrt_absolute_time())
{
	_parser.setCallback(&H104IMU::SampleTrampoline, this);
}

H104IMU::~H104IMU()
{
	Stop();

	perf_free(_perf_frames);
	perf_free(_perf_crc_bad);
	perf_free(_perf_len_bad);
	perf_free(_perf_resync);
	perf_free(_perf_read_errors);
	perf_free(_perf_reopens);
	perf_free(_perf_overflow);
}

int H104IMU::OpenPort()
{
	int fd = ::open(_port, O_RDWR | O_NONBLOCK | O_NOCTTY);

	if (fd < 0) {
		_read_error_count++;
		return -errno;
	}

	struct termios t;

	if (tcgetattr(fd, &t) != 0) {
		::close(fd);
		return -errno;
	}

	/* ★ 与 NuttX H7 驱动对齐（stm32_serial.c 的 case TCSETS 硬校验）：
	 *   (c_cflag & CSIZE) 必须是 CS8，否则 tcsetattr 一律 -EINVAL。
	 *   曾经漏了 CS8 ⇒ 10 个波特率全 EINVAL，看起来像"H104 没在答"。 */
	cfsetspeed(&t, static_cast<speed_t>(115200));   /* NuttX 的 cfsetspeed 接受原始波特率整数 */
	t.c_cflag = CS8 | CLOCAL | CREAD;
	t.c_iflag = 0;
	t.c_oflag = 0;
	t.c_lflag = 0;
	t.c_cc[VMIN] = 1;
	t.c_cc[VTIME] = 0;

	tcflush(fd, TCIOFLUSH);

	if (tcsetattr(fd, TCSANOW, &t) != 0) {
		PX4_ERR("tcsetattr(%s @115200) 失败: %d (%s)", _port, errno, strerror(errno));
		::close(fd);
		return -errno;
	}

	_fd = fd;
	_parser.resetStats();
	_last_frame_time = 0;
	_no_data_rounds = 0;
	PX4_INFO("h104_imu: 打开 %s @115200 8N1（device_id=0x%08" PRIx32 "）", _port, _device_id);
	return PX4_OK;
}

int H104IMU::Start()
{
	/* ★ 不要在模块任务里 open：NuttX 的文件描述符表是**按任务**的，
	 *   Run() 跑在 INS0 工作队列线程里，用模块任务开的 fd 会直接 EBADF
	 *   （实测踩过：Start() 里 open 成功 → Run() 里 read 返回 9/EBADF，
	 *    靠错误分支关掉重开才偶然好了）。
	 *   ⇒ 端口一律在**真正读它的那个任务**（工作队列）里打开。 */
	PX4_INFO("h104_imu: 已调度周期读取 %s（端口将在工作队列里打开）", _port);
	ScheduleDelayed(kRunIntervalUs);
	return PX4_OK;
}

void H104IMU::Stop()
{
	ScheduleClear();

	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}
}

void H104IMU::UpdatePerfCounters()
{
	/* 解析器的统计本身是累计值，直接同步成 perf 的绝对计数
	 * （本版本 perf 只有 perf_count(handle) 一参形式 + perf_set_count(handle,count)） */
	const hipnuc_h104::Stats &s = _parser.stats();

	perf_set_count(_perf_frames, s.frames);
	perf_set_count(_perf_crc_bad, s.crc_bad);
	perf_set_count(_perf_len_bad, s.len_bad);
	perf_set_count(_perf_resync, s.resync);
}

void H104IMU::Publish(const hipnuc_h104::Sample &sample)
{
	/* 时间戳：PX4 侧用 hrt。模块的 system_time（u32 ms，对时后是当日 UTC、24h 回绕）
	 * 只用来做丢帧诊断，不当 timestamp。 */
	const hrt_abstime now = hrt_absolute_time();

	_temperature = sample.temperature_c;
	_px4_accel.set_temperature(sample.temperature_c);
	_px4_gyro.set_temperature(sample.temperature_c);

	_px4_accel.update(now, sample.acc_m_s2[0], sample.acc_m_s2[1], sample.acc_m_s2[2]);
	_px4_gyro.update(now, sample.gyro_rad_s[0], sample.gyro_rad_s[1], sample.gyro_rad_s[2]);

	_publish_count++;
	_last_frame_time = now;

	/* 一次性自检输出（每次启动只打两行，方便无 shell 时用 console 取证）：
	 * 第 1 行证明"端口通了 + 解析出帧 + 已发到 uORB（实例号 >=0 即已 advertise）"；
	 * 第 2 行（约 3 s 后）给出持续运行的帧率与坏帧统计。*/
	if (!_first_frame_logged) {
		_first_frame_logged = true;
		PX4_INFO("h104_imu: 第一帧 OK  device_id=0x%08" PRIx32 "  uORB 实例 accel=%d gyro=%d\n"
			 "           acc=(%.2f, %.2f, %.2f) m/s2  gyro=(%.4f, %.4f, %.4f) rad/s  T=%.0fC",
			 _device_id, _px4_accel.get_instance(), _px4_gyro.get_instance(),
			 static_cast<double>(sample.acc_m_s2[0]), static_cast<double>(sample.acc_m_s2[1]),
			 static_cast<double>(sample.acc_m_s2[2]),
			 static_cast<double>(sample.gyro_rad_s[0]), static_cast<double>(sample.gyro_rad_s[1]),
			 static_cast<double>(sample.gyro_rad_s[2]), static_cast<double>(sample.temperature_c));

	} else if (!_stats_logged && _publish_count >= 300) {
		_stats_logged = true;
		const hipnuc_h104::Stats &st = _parser.stats();
		PX4_INFO("h104_imu: 已发布 %" PRIu32 " 帧  解析 bytes=%" PRIu32 " 帧=%" PRIu32
			 " CRC好=%" PRIu32 " CRC坏=%" PRIu32 " 长度错=%" PRIu32,
			 _publish_count, st.bytes, st.frames, st.crc_ok, st.crc_bad, st.len_bad);
	}
}

void H104IMU::Run()
{
	if (_fd < 0) {
		if (OpenPort() == PX4_OK) {
			if (_ever_opened) { perf_count(_perf_reopens); }

			_ever_opened = true;

		} else {
			_open_fail_count++;
			ScheduleDelayed(1000000);   /* 1 s */
			return;
		}
	}

	int bytes_read = 0;

	for (int i = 0; i < kReadsPerRun; i++) {
		uint8_t buf[kReadChunk];
		const ssize_t n = ::read(_fd, buf, sizeof(buf));

		if (n < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) { break; }

			if (errno == EINTR) { continue; }

			PX4_ERR("h104_imu: read 失败 %d (%s)", errno, strerror(errno));
			perf_count(_perf_read_errors);
			_read_error_count++;
			::close(_fd);
			_fd = -1;
			break;

		} else if (n == 0) {
			break;
		}

		bytes_read += static_cast<int>(n);
		_parser.feed(buf, static_cast<size_t>(n));

		if (n < static_cast<ssize_t>(sizeof(buf))) { break; }
	}

	UpdatePerfCounters();

	if (bytes_read == 0) {
		_no_data_rounds++;

		/* 连续 ~2 s 一个字节都没有 ⇒ 串口可能掉了（拔线/模块掉电），重开端口 */
		if (_no_data_rounds > (2000 / (kRunIntervalUs / 1000))) {
			PX4_WARN("h104_imu: %s 连续 2 s 无数据，重开端口", _port);

			if (_fd >= 0) { ::close(_fd); _fd = -1; }

			_no_data_rounds = 0;
			perf_count(_perf_reopens);
		}

	} else {
		_no_data_rounds = 0;
	}

	ScheduleDelayed(kRunIntervalUs);
}

void H104IMU::PrintStatus()
{
	const hipnuc_h104::Stats &s = _parser.stats();
	const hrt_abstime now = hrt_absolute_time();
	const float elapsed_s = (_start_time == 0) ? 0.f : (now - _start_time) / 1e6f;

	PX4_INFO("h104_imu: %s  fd=%d  device_id=0x%08" PRIx32, _port, _fd, _device_id);
	PX4_INFO("  uORB 实例: accel=%d  gyro=%d  (is_external=true)",
		 _px4_accel.get_instance(), _px4_gyro.get_instance());
	PX4_INFO("  运行 %.1f s, 发布 %" PRIu32 " 帧 (%.1f Hz)",
		 static_cast<double>(elapsed_s), _publish_count,
		 elapsed_s > 1.f ? static_cast<double>(_publish_count / elapsed_s) : 0.0);
	PX4_INFO("  解析: bytes=%" PRIu32 " 候选帧=%" PRIu32 " CRC好=%" PRIu32 " CRC坏=%" PRIu32
		 " 长度错=%" PRIu32 " 重同步跳字节=%" PRIu32,
		 s.bytes, s.frames, s.crc_ok, s.crc_bad, s.len_bad, s.resync);
	PX4_INFO("  最近一帧距今 %.3f s  读错误=%" PRIu32 " 开端口失败=%" PRIu32,
		 _last_frame_time ? (now - _last_frame_time) / 1e6 : -1.0,
		 _read_error_count, _open_fail_count);
	PX4_INFO("  温度=%.0f °C   安装旋转参数=%d（0=已按 FRD 输出）",
		 static_cast<double>(_temperature), static_cast<int>(_rotation));

	perf_print_counter(_perf_frames);
	perf_print_counter(_perf_crc_bad);
	perf_print_counter(_perf_len_bad);
	perf_print_counter(_perf_resync);
	perf_print_counter(_perf_read_errors);
	perf_print_counter(_perf_reopens);
	perf_print_counter(_perf_overflow);
}
