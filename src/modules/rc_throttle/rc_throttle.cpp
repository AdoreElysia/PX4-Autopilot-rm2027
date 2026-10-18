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

/* ===========================================================================
 * rc_throttle —— RM2027 板（rm2027/fmu-h743）遥控链路自检
 *
 * 【一、在线看（有串口/控制台时）】
 *   rc_throttle start / status / stop
 *
 * 【二、离线看（只有 SWD/ST-Link 时，本板常用）】
 *   本文件把一个【固定布局】的状态块放在 RAM 里，符号名：
 *       rm2027_rc_status
 *   用 OpenOCD 不停机直接读那段内存即可判断"遥控信号到底有没有进来"：
 *
 *       arm-none-eabi-nm <elf> | grep rm2027_rc_status     # 取地址
 *       openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
 *         -c "init; mdw 0x2400xxxx 48; shutdown"           # 读 48 个字
 *
 *   结构体定义见 Rm2027RcStatus，字段含义见每个成员后的注释。
 *
 * 【三、内置的"原始串口探针"（不依赖 rc_input 驱动）】
 *   如果开机 6 秒后仍然没有 input_rc 数据，模块会自己打开 /dev/ttyS4，
 *   用 SBUS 参数（100kbps / 8E2）读两轮，每轮 3 秒：
 *       第 1 轮：不反相（正常极性）
 *       第 2 轮：打开 UART RX 反相
 *   把每轮的"收到多少字节 / 解析出多少帧"记进状态块。
 *   这样即使 rc_input 没被启动（端口没分配），也能独立测出
 *   "线上有没有信号"以及"信号是反相还是非反相"。
 *
 * 说明：本模块只观察，不控制任何执行器。
 * =========================================================================== */

#include <cstdint>
#include <cstdio>
#include <cstring>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/rc/sbus.h>
#include <lib/perf/perf_counter.h>
#include <parameters/param.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/manual_control_setpoint.h>

#if defined(__PX4_NUTTX)
# include <nuttx/serial/tioctl.h>     /* TIOCSINVERT / SER_INVERT_ENABLED_RX */
#endif

using namespace time_literals;

namespace
{
constexpr hrt_abstime kSampleInterval{100_ms};      /* 采样周期 */
constexpr int kPrintEveryNSamples = 10;             /* 每 10 次采样打印一行（1 秒） */
constexpr int kChannelActivityThresholdUs = 200;    /* 判定"通道在动"的门槛 */
constexpr hrt_abstime kActivityWindow{2_s};         /* 活动统计窗口 */

/* 探针参数 */
constexpr hrt_abstime kProbeStartDelay{6_s};        /* 开机后等多久判定"没见过数据" */
constexpr hrt_abstime kProbePhaseTime{3_s};         /* 每轮测量时长 */
constexpr const char *kProbeDevice = "/dev/ttyS4";  /* RC 口：UART5（PD2） */

/* 探针状态 */
enum ProbeState : uint32_t {
	PROBE_WAIT       = 0,   /* 待命（还在等 input_rc 数据） */
	PROBE_PHASE_A    = 1,   /* 正在测：不反相 */
	PROBE_PHASE_B    = 2,   /* 正在测：反相 */
	PROBE_DONE       = 3,   /* 测完 */
	PROBE_OPEN_FAIL  = 4,   /* 打不开设备（多半被 rc_input 占用，这本身就说明端口分配好了） */
};
} // namespace

/* ---------------------------------------------------------------------------
 * 这个结构体是给 SWD 直接读的：成员全部是定长整数，无指针。
 * 改字段顺序/类型等于改协议，记得同时改 version。
 * ------------------------------------------------------------------------- */
struct Rm2027RcStatus {
	uint32_t magic;             /* 'RC27' = 0x52433237，用来确认读到的是这块内存 */
	uint32_t version;           /* 结构体版本 */
	uint32_t run_count;         /* Run() 被调用次数（证明模块在跑） */
	uint32_t uptime_ms;         /* 开机到现在的毫秒数 */

	/* ---- input_rc（rc_input 驱动解出来的原始脉宽）---- */
	uint64_t rc_last_update;    /* 我们最近一次拿到 input_rc 的时刻（us，0 = 从未） */
	uint64_t rc_last_signal;    /* 驱动记录的"最近一次有信号"时刻（us） */
	uint32_t rc_updates;        /* 观测到的 input_rc 更新次数（>0 就说明收到过） */
	uint32_t rc_total_frames;   /* 驱动统计：总帧数 */
	uint32_t rc_lost_frames;    /* 驱动统计：丢帧数 */
	uint32_t rc_frame_rate;     /* 帧率 Hz */
	int32_t  rc_rssi;           /* 信号强度（100 = 满） */
	uint32_t rc_channels;       /* 通道数 */
	uint32_t rc_source;         /* 协议来源：9 = SBUS/W.BUS */
	uint32_t rc_lost;           /* 1 = 当前判定为链路丢失 */
	uint32_t rc_failsafe;       /* 1 = 接收机报失控保护 */
	uint32_t ch_last[8];        /* 最近一次的前 8 通道脉宽（us） */
	uint32_t ch_min[8];         /* 活动窗口内的最小值 */
	uint32_t ch_max[8];         /* 活动窗口内的最大值 */

	/* ---- manual_control_setpoint（rc_update 按 RC_MAP_* 映射后的摇杆）---- */
	float    thr;               /* 油门（-1~1）；恒为 0 通常意味着 RC_MAP_THROTTLE 没设 */
	float    roll, pitch, yaw;
	uint32_t mcs_valid;

	/* ---- 端口配置（快速判断 RC 有没有被分配）---- */
	int32_t  rc_port_config;    /* RC_PORT_CONFIG：300 = Radio Controller(/dev/ttyS4)，0 = Disabled */

	/* ---- 原始串口探针（不依赖 rc_input）---- */
	uint32_t probe_state;       /* 见 ProbeState */
	uint32_t probe_cfg_ret;     /* sbus_config() 返回值：0 = 串口参数设置成功 */
	uint32_t probe_bytes_a;     /* 第 1 轮（不反相）收到的字节数 */
	uint32_t probe_frames_a;    /* 第 1 轮解析出的有效 SBUS 帧数 */
	uint32_t probe_bytes_b;     /* 第 2 轮（反相）收到的字节数 */
	uint32_t probe_frames_b;    /* 第 2 轮解析出的帧数 */
	uint32_t probe_inv_ret;     /* 打开反相 ioctl 的返回值（0 = 成功） */
	uint32_t probe_first[8];    /* 最先收到的 8 个字节，每个占一个 u32（看 0x0F 判断是不是 SBUS） */
	uint32_t probe_ch[8];       /* 探针解出的前 8 通道（任一轮成功时填充） */
	uint32_t probe_ch_count;
	uint32_t probe_changes;     /* 探针观测到通道值变化的次数（>0 说明信号在动） */

	/* ---- 用飞控自己当"简易示波器"：直接采样 PD2 的电平 ---- */
	uint32_t wave_points;       /* 本次采样点数 */
	uint32_t wave_dt_us;        /* 采样窗口时长（us） */
	uint32_t wave_high;         /* 其中高电平的点数 */
	uint32_t wave_edges;        /* 电平跳变次数 */

	/* ---- UART5 寄存器（判断实际波特率/错误标志）---- */
	uint32_t uart_cr1;
	uint32_t uart_cr2;
	uint32_t uart_cr3;
	uint32_t uart_brr;
	uint32_t uart_isr;          /* bit1 FE 帧错误 / bit0 PE 校验错 / bit3 ORE 溢出 / bit5 RXNE */
	uint32_t reserved[2];
};

/* 这个全局符号就是 SWD 的观察窗 */
extern "C" {
	volatile Rm2027RcStatus rm2027_rc_status = {};
}

class RcThrottle : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	RcThrottle();
	~RcThrottle() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	int print_status() override;

private:
	void Run() override;

	static const char *protocol_name(uint8_t input_source);

	void reset_activity();
	void update_activity(const input_rc_s &rc);

	/* 原始串口探针 */
	void probe_update();

	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

	int16_t _channel_min[input_rc_s::RC_INPUT_MAX_CHANNELS] {};
	int16_t _channel_max[input_rc_s::RC_INPUT_MAX_CHANNELS] {};
	bool    _channel_seen[input_rc_s::RC_INPUT_MAX_CHANNELS] {};

	hrt_abstime _activity_window_start{0};
	int _sample_count{0};
	hrt_abstime _start_time{0};

	input_rc_s _last_rc {};
	manual_control_setpoint_s _last_manual {};

	/* 探针内部状态 */
	int _probe_fd{-1};
	hrt_abstime _probe_phase_start{0};
	uint16_t _probe_values[input_rc_s::RC_INPUT_MAX_CHANNELS] {};
	uint16_t _probe_num_values{0};
	bool _probe_have_channels{false};
	uint16_t _probe_prev_values[input_rc_s::RC_INPUT_MAX_CHANNELS] {};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::RC_MAP_ROLL>)    _param_rc_map_roll,
		(ParamInt<px4::params::RC_MAP_PITCH>)   _param_rc_map_pitch,
		(ParamInt<px4::params::RC_MAP_THROTTLE>)_param_rc_map_throttle,
		(ParamInt<px4::params::RC_MAP_YAW>)     _param_rc_map_yaw
	)

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
};

ModuleBase::Descriptor RcThrottle::desc{task_spawn, custom_command, print_usage};

RcThrottle::RcThrottle() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	reset_activity();
	_start_time = hrt_absolute_time();

	memset((void *)&rm2027_rc_status, 0, sizeof(rm2027_rc_status));
	rm2027_rc_status.magic = 0x52433237;   /* 'RC27' */
	rm2027_rc_status.version = 4;

	/* 把 RC_PORT_CONFIG 抄进状态块：300 = Radio Controller（/dev/ttyS4） */
	param_t h = param_find("RC_PORT_CONFIG");

	if (h != PARAM_INVALID) {
		int32_t v = 0;
		param_get(h, &v);
		rm2027_rc_status.rc_port_config = v;
	}
}

RcThrottle::~RcThrottle()
{
	if (_probe_fd >= 0) {
		::close(_probe_fd);
		_probe_fd = -1;
	}

	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

int RcThrottle::task_spawn(int argc, char *argv[])
{
	RcThrottle *instance = new RcThrottle();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		instance->ScheduleOnInterval(kSampleInterval);

		PX4_INFO("started (SWD 观察窗: rm2027_rc_status)");

		return PX4_OK;
	}

	PX4_ERR("alloc failed");
	return PX4_ERROR;
}

void RcThrottle::reset_activity()
{
	for (int i = 0; i < input_rc_s::RC_INPUT_MAX_CHANNELS; i++) {
		_channel_min[i] = INT16_MAX;
		_channel_max[i] = INT16_MIN;
		_channel_seen[i] = false;
	}

	_activity_window_start = hrt_absolute_time();
}

void RcThrottle::update_activity(const input_rc_s &rc)
{
	const int n = (rc.channel_count < input_rc_s::RC_INPUT_MAX_CHANNELS)
		      ? rc.channel_count : input_rc_s::RC_INPUT_MAX_CHANNELS;

	for (int i = 0; i < n; i++) {
		const int16_t v = (int16_t)rc.values[i];

		if (v > 900 && v < 2200) {
			if (v < _channel_min[i]) { _channel_min[i] = v; }

			if (v > _channel_max[i]) { _channel_max[i] = v; }

			_channel_seen[i] = true;
		}
	}
}

const char *RcThrottle::protocol_name(uint8_t input_source)
{
	switch (input_source) {
	case input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM:    return "PPM";
	case input_rc_s::RC_INPUT_SOURCE_PX4FMU_SBUS:   return "SBUS/W.BUS";
	case input_rc_s::RC_INPUT_SOURCE_PX4FMU_DSM:    return "DSM/Spektrum";
	case input_rc_s::RC_INPUT_SOURCE_PX4FMU_ST24:   return "ST24";
	case input_rc_s::RC_INPUT_SOURCE_PX4FMU_SUMD:   return "SUMD";
	case input_rc_s::RC_INPUT_SOURCE_PX4FMU_CRSF:   return "CRSF";
	case input_rc_s::RC_INPUT_SOURCE_PX4FMU_GHST:   return "GHST";
	case input_rc_s::RC_INPUT_SOURCE_MAVLINK:       return "MAVLink";
	default:                                        return "未知";
	}
}

/* ------------------------------------------------------------------ 探针 --
 * 独立于 rc_input：直接开 RC 串口，用 SBUS 参数读两轮（不反相 / 反相），
 * 把"收到多少字节、解出多少帧"记进状态块。这样即使 rc_input 没启动，
 * 也能回答"线上到底有没有信号、极性对不对"。
 * -------------------------------------------------------------------------- */
void RcThrottle::probe_update()
{
	Rm2027RcStatus *st = (Rm2027RcStatus *)&rm2027_rc_status;
	const hrt_abstime now = hrt_absolute_time();

	switch (st->probe_state) {

	case PROBE_WAIT:
		/* 只在"开机 6 秒后仍然一帧 input_rc 都没见过"时才启动探针 */
		if (st->rc_updates == 0 && hrt_elapsed_time(&_start_time) > kProbeStartDelay) {
			_probe_fd = ::open(kProbeDevice, O_RDWR | O_NONBLOCK);

			if (_probe_fd < 0) {
				/* 打不开：多半是 rc_input 正占着这个口，这本身是好消息 */
				st->probe_state = PROBE_OPEN_FAIL;

			} else {
				st->probe_cfg_ret = (uint32_t)sbus_config(_probe_fd, false);

#if defined(TIOCSINVERT)
				/* 先确保"不反相"，从第 1 轮开始 */
				ioctl(_probe_fd, TIOCSINVERT, 0);
#endif
				_probe_phase_start = now;
				st->probe_state = PROBE_PHASE_A;
				st->probe_bytes_a = 0;
				st->probe_frames_a = 0;
			}
		}

		break;

	case PROBE_PHASE_A:
	case PROBE_PHASE_B: {
			/* 把缓冲区读空（每轮最多 512 字节，10Hz 采样足够） */
			uint8_t buf[128];

			for (int i = 0; i < 4; i++) {
				const int n = ::read(_probe_fd, buf, sizeof(buf));

				if (n <= 0) { break; }

				if (st->probe_state == PROBE_PHASE_A) {
					st->probe_bytes_a += (uint32_t)n;

				} else {
					st->probe_bytes_b += (uint32_t)n;
				}

				/* 记下最先收到的 8 个字节（看是不是 0x0F 开头，判断 SBUS） */
				if (st->probe_first[0] == 0 && n > 0) {
					for (int k = 0; k < n && k < 8; k++) {
						st->probe_first[k] = 0x100 | buf[k];
					}
				}

				/* 尝试解析 SBUS 帧 */
				bool failsafe = false, frame_drop = false;
				unsigned drops = 0;

				if (sbus_parse(now, buf, (unsigned)n, _probe_values, &_probe_num_values,
					       &failsafe, &frame_drop, &drops, input_rc_s::RC_INPUT_MAX_CHANNELS)) {

					if (st->probe_state == PROBE_PHASE_A) {
						st->probe_frames_a++;

					} else {
						st->probe_frames_b++;
					}

					st->probe_ch_count = _probe_num_values;

					for (int c = 0; c < 8 && c < _probe_num_values; c++) {
						st->probe_ch[c] = _probe_values[c];
					}

					/* 和上一帧比，看通道值有没有在动 */
					if (_probe_have_channels) {
						for (int c = 0; c < 8 && c < _probe_num_values; c++) {
							if (_probe_values[c] != _probe_prev_values[c]) {
								st->probe_changes++;
								break;
							}
						}
					}

					memcpy(_probe_prev_values, _probe_values,
					       sizeof(uint16_t) * input_rc_s::RC_INPUT_MAX_CHANNELS);
					_probe_have_channels = true;
				}
			}

			/* 本轮时间到 → 切下一轮 / 收尾 */
			if (hrt_elapsed_time(&_probe_phase_start) > kProbePhaseTime) {

				if (st->probe_state == PROBE_PHASE_A) {
#if defined(TIOCSINVERT)
					st->probe_inv_ret = (uint32_t)ioctl(_probe_fd, TIOCSINVERT, SER_INVERT_ENABLED_RX);
#endif
					_probe_phase_start = now;
					st->probe_state = PROBE_PHASE_B;
					st->probe_bytes_b = 0;
					st->probe_frames_b = 0;

				} else {
#if defined(TIOCSINVERT)
					ioctl(_probe_fd, TIOCSINVERT, 0);
#endif
					::close(_probe_fd);
					_probe_fd = -1;
					st->probe_state = PROBE_DONE;
				}
			}

			break;
		}

	default:
		break;
	}
}

/* ------------------------------------------------------------------ 波形 --
 * "简易示波器"：直接读 GPIOD 的输入寄存器（PD2 = UART5_RX）。
 * 引脚是 UART 的复用模式也没关系 —— IDR 仍然反映引脚上的真实电平，
 * 所以不需要抢 UART 的所有权，也不会干扰 rc_input。
 *
 * 判读方法：
 *   50 Hz PWM  → 30ms 窗口里只有 2~4 个边沿
 *   100kHz 串流 → 同样的窗口里有上万个边沿
 *   没有信号    → 0 个边沿（电平恒定）
 * ------------------------------------------------------------------------- */
namespace
{
constexpr uint32_t kGpiodIdr = 0x58020C10u;   /* STM32H7 GPIOD 基址 0x58020C00 + IDR 偏移 0x10 */
constexpr uint32_t kUart5Base = 0x40005000u;  /* USART5（= /dev/ttyS4）*/

inline bool pd2_high()
{
	return (*(volatile uint32_t *)kGpiodIdr & (1u << 2)) != 0;
}
} // namespace

void RcThrottle::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	Rm2027RcStatus *st = (Rm2027RcStatus *)&rm2027_rc_status;

	/* ── 读数据 ── */
	const bool rc_updated = _input_rc_sub.copy(&_last_rc);
	_manual_control_setpoint_sub.copy(&_last_manual);

	if (rc_updated) {
		/* 只把"真的在收"的帧记成更新：rc_lost 时的空包不算 */
		if (!_last_rc.rc_lost) {
			update_activity(_last_rc);
			st->rc_updates++;
		}

		st->rc_last_update = _last_rc.timestamp_last_signal;
	}

	if (hrt_elapsed_time(&_activity_window_start) > kActivityWindow) {
		reset_activity();
	}

	/* ── 波形采样（PD2 电平，按时间窗做，保证覆盖 50Hz 的一个多周期）── */
	{
		uint32_t points = 0;
		uint32_t highs = 0;
		uint32_t edges = 0;
		bool prev = pd2_high();
		const hrt_abstime t0 = hrt_absolute_time();

		while (hrt_elapsed_time(&t0) < 25_ms && points < 5000000u) {
			const bool v = pd2_high();
			points++;
			highs += v ? 1 : 0;

			if (v != prev) {
				edges++;
				prev = v;
			}
		}

		st->wave_points = points;
		st->wave_dt_us = (uint32_t)hrt_elapsed_time(&t0);
		st->wave_high = highs;
		st->wave_edges = edges;
	}

	/* ── UART5 寄存器（看实际波特率、错误标志）── */
	{
		st->uart_cr1 = *(volatile uint32_t *)(kUart5Base + 0x00);
		st->uart_cr2 = *(volatile uint32_t *)(kUart5Base + 0x04);
		st->uart_cr3 = *(volatile uint32_t *)(kUart5Base + 0x08);
		st->uart_brr = *(volatile uint32_t *)(kUart5Base + 0x0C);
		st->uart_isr = *(volatile uint32_t *)(kUart5Base + 0x1C);
	}

	/* ── 更新 SWD 观察窗 ── */
	st->run_count++;
	st->uptime_ms = (uint32_t)(hrt_absolute_time() / 1000);
	st->rc_last_signal        = _last_rc.timestamp_last_signal;
	st->rc_total_frames       = _last_rc.rc_total_frame_count;
	st->rc_lost_frames        = _last_rc.rc_lost_frame_count;
	st->rc_frame_rate         = _last_rc.rc_frame_rate;
	st->rc_rssi               = _last_rc.rssi;
	st->rc_channels           = _last_rc.channel_count;
	st->rc_source             = _last_rc.input_source;
	st->rc_lost               = _last_rc.rc_lost ? 1 : 0;
	st->rc_failsafe           = _last_rc.rc_failsafe ? 1 : 0;

	for (int i = 0; i < 8; i++) {
		st->ch_last[i] = _last_rc.values[i];
		st->ch_min[i] = _channel_seen[i] ? (uint32_t)_channel_min[i] : 0;
		st->ch_max[i] = _channel_seen[i] ? (uint32_t)_channel_max[i] : 0;
	}

	st->thr        = _last_manual.throttle;
	st->roll       = _last_manual.roll;
	st->pitch      = _last_manual.pitch;
	st->yaw        = _last_manual.yaw;
	st->mcs_valid  = _last_manual.valid ? 1 : 0;

	/* ── 探针 ── */
	probe_update();

	/* ── 每秒打印（有控制台时看；全部 ASCII，避免控制台乱码） ── */
	if (++_sample_count >= kPrintEveryNSamples) {
		_sample_count = 0;

		if (_last_rc.timestamp == 0) {
			PX4_INFO("no RC frames yet | probe state=%u: A=%uB/%ufr B=%uB/%ufr",
				 (unsigned)st->probe_state,
				 (unsigned)st->probe_bytes_a, (unsigned)st->probe_frames_a,
				 (unsigned)st->probe_bytes_b, (unsigned)st->probe_frames_b);

		} else if (_last_rc.rc_lost) {
			PX4_INFO("RC link lost: last frame %.1f s ago",
				 (double)(hrt_absolute_time() - _last_rc.timestamp_last_signal) / 1e6);

		} else {
			const int thr_ch = _param_rc_map_throttle.get();
			int thr_raw = (thr_ch >= 1 && thr_ch <= (int)_last_rc.channel_count)
				      ? _last_rc.values[thr_ch - 1] : -1;

			PX4_INFO("RC OK protocol=%s ch=%u rate=%uHz rssi=%d | throttle ch%d = %d us | mcs.thr=%+.2f",
				 protocol_name(_last_rc.input_source), (unsigned)_last_rc.channel_count,
				 (unsigned)_last_rc.rc_frame_rate, (int)_last_rc.rssi,
				 thr_ch, thr_raw >= 0 ? thr_raw : 0,
				 (double)_last_manual.throttle);
		}

		/* 波形摘要：边沿多 = 串流；边沿少 = PWM；无边沿 = 没信号 */
		const double dt_s = (st->wave_dt_us > 0) ? st->wave_dt_us / 1e6 : 0.0;
		const double duty = (st->wave_points > 0) ? 100.0 * st->wave_high / st->wave_points : 0.0;
		const double edge_rate = (dt_s > 0.0) ? st->wave_edges / dt_s : 0.0;

		PX4_INFO("PD2 wave: %u pts in %u us, duty=%.1f%%, edges=%u (~%.0f edges/s)",
			 (unsigned)st->wave_points, (unsigned)st->wave_dt_us, duty,
			 (unsigned)st->wave_edges, edge_rate);
	}

	perf_end(_loop_perf);
}

int RcThrottle::print_status()
{
	const Rm2027RcStatus *st = (const Rm2027RcStatus *)&rm2027_rc_status;

	PX4_INFO("magic=0x%08x ver=%u run=%u uptime=%ums  RC_PORT_CONFIG=%d",
		 (unsigned)st->magic, (unsigned)st->version, (unsigned)st->run_count,
		 (unsigned)st->uptime_ms, (int)st->rc_port_config);

	PX4_INFO("input_rc: updates=%u proto=%s ch=%u rate=%uHz total=%u lost=%u last_signal=%llu",
		 (unsigned)st->rc_updates, protocol_name((uint8_t)st->rc_source),
		 (unsigned)st->rc_channels, (unsigned)st->rc_frame_rate,
		 (unsigned)st->rc_total_frames, (unsigned)st->rc_lost_frames,
		 (unsigned long long)st->rc_last_signal);

	for (int i = 0; i < 8; i++) {
		PX4_INFO("  ch%d = %4u  [%u..%u]", i + 1,
			 (unsigned)st->ch_last[i], (unsigned)st->ch_min[i], (unsigned)st->ch_max[i]);
	}

	PX4_INFO("mcs: valid=%u thr=%+.2f roll=%+.2f pitch=%+.2f yaw=%+.2f",
		 (unsigned)st->mcs_valid, (double)st->thr, (double)st->roll,
		 (double)st->pitch, (double)st->yaw);

	/* 探针 + 波形：这两条是"线上到底有什么"的直接证据 */
	const double dt_s = (st->wave_dt_us > 0) ? st->wave_dt_us / 1e6 : 0.0;
	const double duty = (st->wave_points > 0) ? 100.0 * st->wave_high / st->wave_points : 0.0;
	const double edge_hz = (dt_s > 0.0) ? st->wave_edges / dt_s : 0.0;

	PX4_INFO("probe: state=%u cfg=%u inv=%u | noInv %uB/%ufr | inv %uB/%ufr | ch_changes=%u",
		 (unsigned)st->probe_state, (unsigned)st->probe_cfg_ret, (unsigned)st->probe_inv_ret,
		 (unsigned)st->probe_bytes_a, (unsigned)st->probe_frames_a,
		 (unsigned)st->probe_bytes_b, (unsigned)st->probe_frames_b,
		 (unsigned)st->probe_changes);

	PX4_INFO("PD2 wave: %u pts/%u us  duty=%.1f%%  edges=%u (~%.0f/s)   [few edges=PWM, many=serial, 0=no signal]",
		 (unsigned)st->wave_points, (unsigned)st->wave_dt_us, duty,
		 (unsigned)st->wave_edges, edge_hz);

	/* USART5: BRR 可以反推实际波特率；ISR 的错误位说明线路质量 */
	PX4_INFO("USART5: CR1=0x%08x CR2=0x%08x CR3=0x%08x BRR=0x%04x ISR=0x%08x (FE=%u PE=%u ORE=%u RXNE=%u)",
		 (unsigned)st->uart_cr1, (unsigned)st->uart_cr2, (unsigned)st->uart_cr3,
		 (unsigned)(st->uart_brr & 0xffff), (unsigned)st->uart_isr,
		 (unsigned)((st->uart_isr >> 1) & 1), (unsigned)(st->uart_isr & 1),
		 (unsigned)((st->uart_isr >> 3) & 1), (unsigned)((st->uart_isr >> 5) & 1));

	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);

	return PX4_OK;
}

int RcThrottle::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RcThrottle::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
RM2027 遥控链路自检：打印 input_rc（原始脉宽）与 manual_control_setpoint（映射后摇杆值），
并在 rc_input 无数据时用内置"原始串口探针"直接读 RC 串口，判断线上有没有信号、极性对不对。

同时把状态写进 RAM 的固定结构体 rm2027_rc_status，可用 SWD（ST-Link）不停机直接读。
只观察，不驱动任何执行器。

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rc_throttle", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");

	return 0;
}

extern "C" __EXPORT int rc_throttle_main(int argc, char *argv[])
{
	return ModuleBase::main(RcThrottle::desc, argc, argv);
}
