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
 * @file h104cap.c
 * @brief H104（HiPNUC 超核电子）外置陀螺 —— 串口取证 / 完整帧抓取（一次性诊断工具）
 *
 * 设计要点（都是被坑出来的）：
 *  1) **结果写进固定 RAM 缓冲区，不走 console** —— 本板 console 环形缓冲只有 4 KB
 *     （BOARD_ENABLE_CONSOLE_BUFFER），hex/日志一大就被覆盖。所以：
 *       g_h104_report[]  人类可读报告
 *       g_h104_cap_buf[] 原始字节流
 *     用 ST-Link `dump_image` 按符号地址读走，**一个字节都不会丢**。
 *  2) **不用 TIOCGICOUNT** —— NuttX 上没有这个 ioctl（PX4 自带的 serial_test 也是
 *     `#if !defined(__PX4_NUTTX)` 排除掉的）。波特率对错靠"ASCII 应答是否可读"判。
 *  3) **绝不碰 SPI1/BMI270**（别人的地盘）、不碰 /dev/ttyUSB0（用户的 QGC）。
 *     只碰 USART1 = /dev/ttyS0（PA10 RX / PB6 TX），不修改任何引脚定义。
 *
 * 用法：
 *   h104cap all                        # ★ 一键：测有无数据 → LOG DISABLE → 三连查询(含波特率扫描) → LOG ENABLE → 抓 3 s → 全部写进 report
 *   h104cap capture -b 115200 -s 3     # 只抓
 *   h104cap ask -b 115200 "LOG VERSION" # 只发一条命令
 *   h104cap report                     # 把 g_h104_report 打到 console（内容短时才可靠）
 *   h104cap hexdump -n 256             # 把 g_h104_cap_buf 前 N 字节打到 console
 *
 * 取数据（SWD，不依赖 GCS）：
 *   addr=$(arm-none-eabi-nm <elf> | awk '/g_h104_report$/{print "0x"$1}')
 *   len=$(... g_h104_report_len ...)   # 或直接 dump 定长
 *   openocd ... -c "halt" -c "dump_image report.bin $addr 0x1000" -c "dump_image cap.bin $addr2 0x2000"
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>

#include <fcntl.h>
#include <poll.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>

/* ------------------------------------------------------------------ 常量 */

#define H104_SOF0        0x5Au
#define H104_SOF1        0xA5u
#define H104_TAG_HI91    0x91u
#define H104_HI91_LEN    76u
#define H104_MAX_PAYLOAD 512u

#define H104_CAP_SIZE    8192u     /* 原始字节：约 1.0 s @ HI91 100 Hz */
#define H104_REPORT_SIZE 16384u    /* 人类可读报告（要容得下 4 条查询的全部应答，别让统计被截掉）*/
#define H104_LINE_SIZE   768u      /* 单条应答的清洗文本上限 */

#define H104_BAUD_DEFAULT 115200

/* 模块支持的 10 档波特率（手册 §串口波特率） */
static const int g_baud_table[] = {4800, 9600, 19200, 38400, 57600, 115200,
				   230400, 256000, 460800, 921600
				  };
#define H104_NBAUD ((int)(sizeof(g_baud_table) / sizeof(g_baud_table[0])))

/* ---- 固定符号的 RAM 缓冲（nm 找得到、openocd 能 dump）---- */

__attribute__((used, section(".bss")))
uint8_t g_h104_cap_buf[H104_CAP_SIZE];
__attribute__((used, section(".bss")))
volatile uint32_t g_h104_cap_len;

__attribute__((used, section(".bss")))
char g_h104_report[H104_REPORT_SIZE];
__attribute__((used, section(".bss")))
volatile uint32_t g_h104_report_len;

/* ------------------------------------------------------------------ 报告 */

static uint32_t g_rep_used;

static void rep_reset(void)
{
	g_rep_used = 0;
	g_h104_report_len = 0;
	memset(g_h104_report, 0, sizeof(g_h104_report));
}

static void rep(const char *fmt, ...)
{
	if (g_rep_used + 2 >= H104_REPORT_SIZE) { return; }

	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(g_h104_report + g_rep_used, H104_REPORT_SIZE - g_rep_used - 1, fmt, ap);
	va_end(ap);

	if (n > 0) {
		g_rep_used += (uint32_t)n;

		if (g_rep_used >= H104_REPORT_SIZE) { g_rep_used = H104_REPORT_SIZE - 1; }

		g_h104_report[g_rep_used] = '\0';
		g_h104_report_len = g_rep_used;
	}
}

/* ------------------------------------------------------------------ 统计 */

struct h104_counters {
	uint32_t bytes;
	uint32_t frames;
	uint32_t crc_ok;
	uint32_t crc_bad;
	uint32_t len_bad;
	uint32_t hi91_ok;
	uint32_t resync;
	uint32_t byte_5a;
	uint32_t byte_a5;
	uint32_t sof_pairs;
	uint32_t main_status_last;
};

static struct h104_counters g_cnt;

/* ------------------------------------------------------------------ CRC */

/** CRC-16/XMODEM：poly 0x1021，init 0x0000，无输入/输出反转，无最终异或 */
static uint16_t h104_crc16_cont(uint16_t crc, const uint8_t *p, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		crc ^= (uint16_t)(p[i] << 8);

		for (int b = 0; b < 8; b++) {
			crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
		}
	}

	return crc;
}

static uint16_t h104_crc16(const uint8_t *p, size_t n)
{
	return h104_crc16_cont(0x0000, p, n);
}

static float h104_f32(const uint8_t *p)
{
	union { uint32_t u; float f; } v;
	v.u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
	return v.f;
}

/* ------------------------------------------------------------------ 组帧 */

static void h104_scan(const uint8_t *buf, size_t n, bool verbose)
{
	static uint8_t acc[1024];
	static size_t  acc_len;
	const uint32_t max_len = sizeof(acc) - 64;

	if (n > 0) {
		if (acc_len + n > max_len) {
			size_t drop = acc_len + n - max_len;
			memmove(acc, acc + drop, acc_len - drop);
			acc_len -= drop;
		}

		memcpy(acc + acc_len, buf, n);
		acc_len += n;
	}

	size_t i = 0;

	while (i + 6 <= acc_len) {
		if (acc[i] != H104_SOF0 || acc[i + 1] != H104_SOF1) {
			i++;
			g_cnt.resync++;
			continue;
		}

		uint16_t len = (uint16_t)acc[i + 2] | ((uint16_t)acc[i + 3] << 8);

		if (len < 1 || len > H104_MAX_PAYLOAD) {
			g_cnt.len_bad++;
			i += 2;
			continue;
		}

		size_t total = 6u + len;

		if (i + total > acc_len) { break; }

		uint16_t crc_rx = (uint16_t)acc[i + 4] | ((uint16_t)acc[i + 5] << 8);
		uint16_t crc_calc = h104_crc16(acc + i, 4);
		crc_calc = h104_crc16_cont(crc_calc, acc + i + 6, len);

		g_cnt.frames++;

		if (crc_rx != crc_calc) {
			g_cnt.crc_bad++;

		} else {
			g_cnt.crc_ok++;
			const uint8_t *pay = acc + i + 6;

			if (len == H104_HI91_LEN && pay[0] == H104_TAG_HI91) {
				g_cnt.hi91_ok++;
				g_cnt.main_status_last = (uint16_t)pay[1] | ((uint16_t)pay[2] << 8);

				if (verbose && g_cnt.hi91_ok <= 3) {
					rep("  HI91[%u] st=0x%04X T=%dC p=%.1fPa t=%ums "
					    "acc[G]=%.4f/%.4f/%.4f gyr[dps]=%.4f/%.4f/%.4f rpy=%.2f/%.2f/%.2f\n",
					    (unsigned)g_cnt.hi91_ok, g_cnt.main_status_last, (int)(int8_t)pay[3],
					    (double)h104_f32(pay + 4),
					    (unsigned)((uint32_t)pay[8] | ((uint32_t)pay[9] << 8) |
						       ((uint32_t)pay[10] << 16) | ((uint32_t)pay[11] << 24)),
					    (double)h104_f32(pay + 12), (double)h104_f32(pay + 16), (double)h104_f32(pay + 20),
					    (double)h104_f32(pay + 24), (double)h104_f32(pay + 28), (double)h104_f32(pay + 32),
					    (double)h104_f32(pay + 48), (double)h104_f32(pay + 52), (double)h104_f32(pay + 56));
				}
			}
		}

		i += total;
	}

	if (i > 0) {
		memmove(acc, acc + i, acc_len - i);
		acc_len -= i;
	}
}

/* ------------------------------------------------------------------ 端口 */

static int h104_open(const char *port, int baud)
{
	int fd = open(port, O_RDWR | O_NONBLOCK | O_NOCTTY);

	if (fd < 0) {
		rep("!! open(%s) 失败: errno=%d (%s)\n", port, errno, strerror(errno));
		return -1;
	}

	struct termios t;

	if (tcgetattr(fd, &t) != 0) {
		rep("!! tcgetattr 失败: errno=%d\n", errno);
		close(fd);
		return -1;
	}

	/* ★ 必须与 NuttX H7 驱动的要求严格对齐：
	 *   arch/arm/src/stm32h7/stm32_serial.c 的 `case TCSETS` 里有硬校验：
	 *       if (((termiosp->c_cflag & CSIZE) != CS8) ...) { ret = -EINVAL; break; }
	 *   ⇒ **c_cflag 里的 CSIZE 必须是 CS8**，否则 tcsetattr 一律 EINVAL，端口等于用不了。
	 *   ⇒ CBAUD 那几位驱动不看（它只认 `priv->baud = cfgetispeed()`，即 c_speed 的原始值），
	 *     所以像 serial_test 那样整段赋值 c_cflag 是安全的；cfsetspeed 对
	 *     "原始波特率整数" 也接受（会命中 g_baud_table[].value 那一支）。
	 *   踩过的坑：曾经写成 `c_cflag &= ~(CSIZE|...)` 却没补 CS8 ⇒ CSIZE=0(CS5)
	 *     ⇒ 10 个波特率**全部** EINVAL，报告看起来像"H104 根本没在答"。 */
	cfsetspeed(&t, (speed_t)baud);
	t.c_cflag = CS8 | CLOCAL | CREAD;
	t.c_iflag = 0;
	t.c_oflag = 0;
	t.c_lflag = 0;
	t.c_cc[VMIN] = 1;
	t.c_cc[VTIME] = 0;

	tcflush(fd, TCIOFLUSH);

	if (tcsetattr(fd, TCSANOW, &t) != 0) {
		rep("!! tcsetattr(%d) 失败: errno=%d (%s)\n", baud, errno, strerror(errno));
		close(fd);
		return -1;
	}

	/* 回读确认：CSIZE 必须回 3(CS8)，c_speed 应该是我们要的波特率 */
	{
		struct termios chk;

		if (tcgetattr(fd, &chk) == 0) {
			rep("   tcsetattr(%d) OK: CSIZE=%d(3=CS8) ispeed=%u ospeed=%u\n",
			    baud, (int)((chk.c_cflag & CSIZE) >> 4),
			    (unsigned)cfgetispeed(&chk), (unsigned)cfgetospeed(&chk));
		}
	}

	return fd;
}

/** 读一段时间，返回读到的字节数；同时把可打印字符清洗进 out（可为 NULL） */
static uint32_t h104_drain(int fd, unsigned ms, char *out, size_t out_max,
			   uint32_t *nonprint, uint32_t *newlines)
{
	uint8_t buf[256];
	uint32_t total = 0, np = 0, nl = 0;
	size_t op = 0;
	hrt_abstime t0 = hrt_absolute_time();

	while (hrt_elapsed_time(&t0) < (hrt_abstime)ms * 1000ULL) {
		struct pollfd pfd = { .fd = fd, .events = POLLIN };
		int pr = poll(&pfd, 1, 5);

		if (pr <= 0) { continue; }

		ssize_t n = read(fd, buf, sizeof(buf));

		if (n <= 0) { continue; }

		total += (uint32_t)n;

		for (ssize_t i = 0; i < n; i++) {
			uint8_t c = buf[i];

			if (c == '\n') { nl++; }

			if (c >= 32 && c < 127) {
				if (out && op + 1 < out_max) {
					out[op++] = (char)c;
					out[op] = '\0';
				}

			} else if (c != '\r' && c != '\n') {
				np++;

				if (out && op + 1 < out_max) { out[op++] = '.'; out[op] = '\0'; }
			}
		}
	}

	if (nonprint) { *nonprint = np; }

	if (newlines) { *newlines = nl; }

	return total;
}

/** 发一条 ASCII 命令并收应答；返回读到的字节数 */
static uint32_t h104_ask(int fd, const char *cmd, unsigned ms, char *out, size_t out_max)
{
	if (out && out_max) { out[0] = '\0'; }

	tcflush(fd, TCIFLUSH);

	char line[128];
	int len = snprintf(line, sizeof(line), "%s\r\n", cmd);

	if (len > 0) {
		ssize_t w = write(fd, line, (size_t)len);

		if (w < 0) { rep("!! write 失败: errno=%d\n", errno); }
	}

	return h104_drain(fd, ms, out, out_max, NULL, NULL);
}

/** ASCII 应答像不像人话（用于判断波特率和模块是否在答） */
static int ascii_score(const char *s)
{
	int score = 0;
	static const char *keys[] = {"PNAME", "APP_VER", "BL_VER", "OK", "ERR", "BAUD", "ONTIME",
				     "UUID", "SN", "BUILD", "PROFILE", "URFR", "COORD", "MAP", "="
				    };

	for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
		if (strstr(s, keys[i])) { score += 10; }
	}

	/* 连续可打印字符比例（乱码波特率下会很低） */
	size_t n = strlen(s), good = 0;

	for (size_t i = 0; i < n; i++) {
		if ((s[i] >= 'A' && s[i] <= 'Z') || (s[i] >= 'a' && s[i] <= 'z') ||
		    (s[i] >= '0' && s[i] <= '9') || s[i] == '=' || s[i] == ' ' || s[i] == '\n') { good++; }
	}

	if (n > 0 && good * 2 > n) { score += 5; }

	if (n > 20) { score += 3; }

	return score;
}

static const char *rate_verdict(uint32_t bytes, unsigned ms)
{
	/* HI91@100Hz ≈ 8200 B/s；给个粗判 */
	uint32_t bps = (uint32_t)((uint64_t)bytes * 1000ULL / (ms ? ms : 1));

	if (bytes == 0) { return "无数据(0 B)"; }

	if (bps > 6000) { return "≈8200B/s 有连续数据流(HI91@100Hz?)"; }

	if (bps > 500) { return "有中等数据流"; }

	return "数据很少";
}

/* ------------------------------------------------------------------ 动作 */

static void do_report_marker(void)
{
	rep("==== h104cap report ====\n");
	rep("SOF假设=5A A5  HI91数据域=76 整帧=82  CRC=CRC-16/XMODEM\n");
}

/** ① 端口有没有数据 + LOG DISABLE 生不生效（模块听不听得懂我们） */
static void stage_flow(const char *port, int baud, uint32_t *rate_before, uint32_t *rate_after)
{
	int fd = h104_open(port, baud);

	if (fd < 0) { *rate_before = *rate_after = 0; return; }

	char rsp[256] = {0};
	uint32_t nb = h104_drain(fd, 300, NULL, 0, NULL, NULL);
	*rate_before = nb;
	rep("[1] @%d 先静默观察 300ms：%u 字节 —— %s\n", baud, (unsigned)nb, rate_verdict(nb, 300));

	if (nb > 0) {
		uint32_t w = h104_ask(fd, "LOG DISABLE", 400, rsp, sizeof(rsp));
		uint32_t after = h104_drain(fd, 300, NULL, 0, NULL, NULL);
		*rate_after = after;
		rep("    发 LOG DISABLE（读到 %u B，应答=%s）后 300ms：%u 字节\n",
		    (unsigned)w, rsp[0] ? rsp : "(不可读)", (unsigned)after);

		if (after * 4 < nb) {
			rep("    ⇒ ✅ 数据流被停住了 ⇒ **模块听懂了 115200 的 ASCII 命令**\n");

		} else {
			rep("    ⇒ ❌ 数据流没停 ⇒ 模块可能不在这个波特率，或这条线不是它在发\n");
		}
	}

	close(fd);
}

/** ② 在指定波特率问一句，把清洗后的应答写进 report */

/** ③ 波特率扫描：每个波特率先 LOG DISABLE 把数据闸住，再问 LOG VERSION。
 *  这样"对的那个波特率"应答又短又干净，一眼能认出来（否则应答会被数据帧刷屏埋掉）。 */
static int probe_baud(const char *port)
{
	rep("[3] 波特率扫描（每个档先发 LOG DISABLE 闸住数据，再发 LOG VERSION）\n");
	int best = -1, best_score = 0;

	for (int i = 0; i < H104_NBAUD; i++) {
		int b = g_baud_table[i];
		int fd = h104_open(port, b);

		if (fd < 0) { continue; }

		char rsp[H104_LINE_SIZE] = {0};
		uint32_t np = 0, nl = 0;
		char dis[192] = {0};

		h104_drain(fd, 50, NULL, 0, NULL, NULL);
		tcflush(fd, TCIFLUSH);
		/* 先在**这个**波特率下把输出闸住（对得上的话数据会停） */
		write(fd, "LOG DISABLE\r\n", 13);
		uint32_t d = h104_drain(fd, 250, dis, sizeof(dis), NULL, NULL);

		tcflush(fd, TCIFLUSH);
		write(fd, "LOG VERSION\r\n", 13);
		uint32_t n = h104_drain(fd, 500, rsp, sizeof(rsp), &np, &nl);
		close(fd);

		int sc = ascii_score(rsp);

		if (sc > best_score) { best_score = sc; best = b; }

		rep("    %-7d : LOG_DISABLE读到%-5u B | LOG_VERSION读到%-5u B np=%-4u nl=%-3u score=%-3d %s\n",
		    b, (unsigned)d, (unsigned)n, (unsigned)np, (unsigned)nl, sc, sc >= 25 ? "★ 像应答" : "");

		if (sc >= 25) {
			char *e = strchr(rsp, '\n');

			if (e) { *e = '\0'; }

			if (strlen(rsp) > 90) { rsp[90] = '\0'; }

			rep("        ⇒ 应答摘录: %s\n", rsp);
		}
	}

	rep("    ⇒ 最像应答的波特率 = %d（score=%d）\n", best, best_score);
	return (best_score >= 25) ? best : -1;
}

static int do_all(const char *port)
{
	rep_reset();
	memset(&g_cnt, 0, sizeof(g_cnt));
	g_h104_cap_len = 0;

	do_report_marker();
	rep("[0] now=%llu us  port=%s  baud_default=%d\n", (unsigned long long)hrt_absolute_time(), port,
	    H104_BAUD_DEFAULT);

	/* 1. 有没有数据 + LOG DISABLE 是否生效 */
	uint32_t before = 0, after = 0;
	stage_flow(port, H104_BAUD_DEFAULT, &before, &after);

	/* 2. 三连查询 */
	rep("[2] ASCII 查询 @%d（每问间隔 600ms）\n", H104_BAUD_DEFAULT);
	const char *cmds[] = {"LOG VERSION", "LOG COMCONFIG", "LOG USRCONFIG", "LOG MCAL STAT"};

	for (int i = 0; i < 4; i++) {
		int fd = h104_open(port, H104_BAUD_DEFAULT);

		if (fd < 0) { break; }

		char rsp[H104_LINE_SIZE] = {0};
		uint32_t np = 0, nl = 0;
		h104_drain(fd, 50, NULL, 0, NULL, NULL);
		tcflush(fd, TCIFLUSH);

		char line[128];
		int len = snprintf(line, sizeof(line), "%s\r\n", cmds[i]);
		ssize_t w = write(fd, line, (size_t)len);
		(void)w;
		uint32_t n = h104_drain(fd, 600, rsp, sizeof(rsp), &np, &nl);
		close(fd);

		rep("  > %s\n", cmds[i]);
		rep("    收到 %u B  np=%u nl=%u score=%d\n", (unsigned)n, (unsigned)np, (unsigned)nl, ascii_score(rsp));

		if (rsp[0]) {
			int lines = 0;
			char *p = rsp;

			while (p && *p && lines < 8) {
				char *e = strchr(p, '\n');
				size_t l = e ? (size_t)(e - p) : strlen(p);

				if (l > 100) { l = 100; }

				if (l > 0) {
					char save = p[l];
					p[l] = '\0';
					rep("      | %s\n", p);
					p[l] = save;
				}

				if (!e) { break; }

				p = e + 1;
				lines++;
			}
		}
	}

	/* 3. 波特率扫描：115200 已经有连续数据流就信它；否则（或扫描有更强者）用扫描结果 */
	int good_baud = H104_BAUD_DEFAULT;
	int probed = probe_baud(port);

	if (before > 0) {
		rep("[3b] 115200 本来就有数据流（%u B/300ms）⇒ 维持 115200（扫描结果 %d 仅作参考）\n",
		    (unsigned)before, probed);
		good_baud = H104_BAUD_DEFAULT;

		if (probed > 0 && probed != H104_BAUD_DEFAULT) {
			rep("     ⚠️ 但扫描认为 %d 更像应答 —— 两者矛盾，需人工看上面明细\n", probed);
		}

	} else if (probed > 0) {
		good_baud = probed;
		rep("[3b] 115200 没数据，采用扫描出的波特率 %d\n", good_baud);

	} else {
		rep("[3b] ⚠️ 没有任何波特率能问出人话 ⇒ 这条线可能不是 H104 / 模块没在跑\n");
	}

	/* 4. 恢复输出 + 抓一段完整字节流 */
	{
		int fd = h104_open(port, good_baud > 0 ? good_baud : H104_BAUD_DEFAULT);

		if (fd >= 0) {
			char rsp[128] = {0};
			uint32_t w = h104_ask(fd, "LOG ENABLE", 400, rsp, sizeof(rsp));
			rep("[4] 发 LOG ENABLE（读到 %u B，应答=%s）\n", (unsigned)w, rsp[0] ? rsp : "(不可读)");
			close(fd);
		}

		int b = (good_baud > 0) ? good_baud : H104_BAUD_DEFAULT;
		rep("[5] 抓 3 秒原始字节 @%d 到 g_h104_cap_buf（%u 字节上限）\n", b, (unsigned)H104_CAP_SIZE);
		fd = h104_open(port, b);

		if (fd < 0) {
			rep("!! 抓包端口打开失败\n");

		} else {
			uint32_t written = 0;
			uint8_t chunk[512];
			uint8_t prev = 0;
			hrt_abstime t0 = hrt_absolute_time();

			while (hrt_elapsed_time(&t0) < 3000000ULL && written < H104_CAP_SIZE) {
				struct pollfd pfd = { .fd = fd, .events = POLLIN };
				int pr = poll(&pfd, 1, 5);

				if (pr <= 0) { continue; }

				ssize_t n = read(fd, chunk, sizeof(chunk));

				if (n <= 0) { continue; }

				g_cnt.bytes += (uint32_t)n;

				for (ssize_t i = 0; i < n; i++) {
					if (chunk[i] == H104_SOF0) { g_cnt.byte_5a++; }

					if (chunk[i] == H104_SOF1) { g_cnt.byte_a5++; }

					if (prev == H104_SOF0 && chunk[i] == H104_SOF1) { g_cnt.sof_pairs++; }

					prev = chunk[i];
				}

				size_t room = H104_CAP_SIZE - written;
				size_t cp = ((size_t)n < room) ? (size_t)n : room;
				memcpy(g_h104_cap_buf + written, chunk, cp);
				written += cp;
				g_h104_cap_len = written;

				h104_scan(chunk, (size_t)n, false);
			}

			h104_scan(NULL, 0, true);
			close(fd);

			uint32_t elapsed_ms = (uint32_t)(hrt_elapsed_time(&t0) / 1000ULL);
			rep("    抓到 %u 字节 / %u ms\n", (unsigned)written, (unsigned)elapsed_ms);
		}
	}

	/* 6. 统计 */
	rep("[6] 帧统计\n");
	rep("    bytes=%u  5A=%u  A5=%u  5A-A5相邻=%u\n", (unsigned)g_cnt.bytes, (unsigned)g_cnt.byte_5a,
	    (unsigned)g_cnt.byte_a5, (unsigned)g_cnt.sof_pairs);
	rep("    候选帧=%u  长度错=%u  重同步跳过=%u\n", (unsigned)g_cnt.frames, (unsigned)g_cnt.len_bad,
	    (unsigned)g_cnt.resync);
	rep("    CRC好=%u  CRC坏=%u  HI91帧=%u  main_status=0x%04X\n", (unsigned)g_cnt.crc_ok,
	    (unsigned)g_cnt.crc_bad, (unsigned)g_cnt.hi91_ok, g_cnt.main_status_last);

	if (g_cnt.bytes > 0) {
		rep("    0x5A 占比 %.1f%%（HI91 理论值约 1.2%%）\n", 100.0 * g_cnt.byte_5a / g_cnt.bytes);
	}

	rep("[7] 结论提示\n");
	rep("    sof_pairs≈候选帧 且 CRC好≈HI91帧 ⇒ 就是 HI91，协议坐实\n");
	rep("    sof_pairs=0 且 5A占比≫1.2%% ⇒ PA10 上不是 HI91 流\n");
	rep("==== end (cap_len=%u report_len=%u) ====\n", (unsigned)g_h104_cap_len, (unsigned)g_h104_report_len);

	/* console 只打一小段，免得撑爆 4KB 环形缓冲 */
	PX4_INFO("h104cap all done: cap=%u B report=%u B", (unsigned)g_h104_cap_len,
		 (unsigned)g_h104_report_len);
	return 0;
}

static void do_report_print(void)
{
	uint32_t n = g_h104_report_len;

	if (n > H104_REPORT_SIZE) { n = H104_REPORT_SIZE; }

	/* console 只有 4KB，超了就只打尾巴 */
	const uint32_t budget = 3000;
	uint32_t off = (n > budget) ? (n - budget) : 0;
	printf("%s%s\n", off ? "...(前面省略)\n" : "", g_h104_report + off);
}

static void do_hexdump(unsigned n)
{
	uint32_t lim = (n < g_h104_cap_len) ? n : g_h104_cap_len;

	PX4_INFO("hexdump %u/%u B:", (unsigned)lim, (unsigned)g_h104_cap_len);

	for (uint32_t off = 0; off < lim; off += 16) {
		char row[16 * 3 + 1];
		int p = 0;

		for (uint32_t k = 0; k < 16 && off + k < lim; k++) {
			p += snprintf(row + p, sizeof(row) - (size_t)p, "%02X ", g_h104_cap_buf[off + k]);
		}

		printf("%04X: %s\n", (unsigned)off, row);
	}
}

static int do_capture(const char *port, int baud, unsigned seconds)
{
	int fd = h104_open(port, baud);

	if (fd < 0) { return 1; }

	memset(&g_cnt, 0, sizeof(g_cnt));
	g_h104_cap_len = 0;
	uint32_t written = 0;
	uint8_t chunk[512];
	uint8_t prev = 0;
	hrt_abstime t0 = hrt_absolute_time();

	while (hrt_elapsed_time(&t0) < (hrt_abstime)seconds * 1000000ULL && written < H104_CAP_SIZE) {
		struct pollfd pfd = { .fd = fd, .events = POLLIN };

		if (poll(&pfd, 1, 5) <= 0) { continue; }

		ssize_t n = read(fd, chunk, sizeof(chunk));

		if (n <= 0) { continue; }

		g_cnt.bytes += (uint32_t)n;

		for (ssize_t i = 0; i < n; i++) {
			if (chunk[i] == H104_SOF0) { g_cnt.byte_5a++; }

			if (chunk[i] == H104_SOF1) { g_cnt.byte_a5++; }

			if (prev == H104_SOF0 && chunk[i] == H104_SOF1) { g_cnt.sof_pairs++; }

			prev = chunk[i];
		}

		size_t room = H104_CAP_SIZE - written;
		size_t cp = ((size_t)n < room) ? (size_t)n : room;
		memcpy(g_h104_cap_buf + written, chunk, cp);
		written += cp;
		g_h104_cap_len = written;
		h104_scan(chunk, (size_t)n, false);
	}

	h104_scan(NULL, 0, true);
	close(fd);
	PX4_INFO("capture: %u B, frames=%u crc_ok=%u crc_bad=%u hi91=%u 5A-A5对=%u",
		 (unsigned)written, (unsigned)g_cnt.frames, (unsigned)g_cnt.crc_ok, (unsigned)g_cnt.crc_bad,
		 (unsigned)g_cnt.hi91_ok, (unsigned)g_cnt.sof_pairs);
	return 0;
}

/* ------------------------------------------------------------------ 入口 */

static void usage(void)
{
	PX4_INFO("Usage: h104cap <all|capture|ask|report|hexdump> [options]");
	PX4_INFO("  all                            一键全流程（写 g_h104_report + g_h104_cap_buf）");
	PX4_INFO("  capture [-b 115200] [-s 3]     只抓原始字节");
	PX4_INFO("  ask [-b 115200] \"LOG VERSION\" 只发一条命令");
	PX4_INFO("  report                         把报告打到 console");
	PX4_INFO("  hexdump [-n 256]               把抓到的字节打到 console");
	PX4_INFO("  -p /dev/ttyS0  指定端口（默认 /dev/ttyS0 = USART1）");
}

int h104cap_main(int argc, char *argv[])
{
	const char *port = "/dev/ttyS0";
	int baud = H104_BAUD_DEFAULT;
	unsigned seconds = 3;
	unsigned dump_n = 256;
	const char *cmd = NULL;
	const char *action = (argc > 1) ? argv[1] : NULL;

	int myoptind = 2;
	int ch;
	const char *myoptarg = NULL;

	while ((ch = px4_getopt(argc, argv, "p:b:s:n:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p': port = myoptarg; break;

		case 'b': baud = atoi(myoptarg); break;

		case 's': seconds = (unsigned)atoi(myoptarg); break;

		case 'n': dump_n = (unsigned)atoi(myoptarg); break;

		default: usage(); return 1;
		}
	}

	if (myoptind < argc) { cmd = argv[myoptind]; }

	if (action == NULL) { usage(); return 1; }

	if (!strcmp(action, "all")) { return do_all(port); }

	if (!strcmp(action, "capture")) { return do_capture(port, baud, seconds); }

	if (!strcmp(action, "ask")) {
		if (cmd == NULL) { PX4_ERR("ask 需要命令字符串"); return 1; }

		int fd = h104_open(port, baud);

		if (fd < 0) { return 1; }

		char rsp[H104_LINE_SIZE] = {0};
		uint32_t n = h104_ask(fd, cmd, 700, rsp, sizeof(rsp));
		close(fd);
		PX4_INFO("ask \"%s\" -> %u B", cmd, (unsigned)n);
		printf("%s\n", rsp);
		return 0;
	}

	if (!strcmp(action, "report")) { do_report_print(); return 0; }

	if (!strcmp(action, "hexdump")) { do_hexdump(dump_n); return 0; }

	usage();
	return 1;
}
