// 主机侧单元测试：用真实上板抓包回归 H104 解析器（不需要板子、不需要 PX4）
//
// 构建 + 运行（一条命令）：
//   g++ -std=c++17 -Wall -Wextra -O2 -o /tmp/t_h104 h104_protocol_test.cpp && /tmp/t_h104 <h104-cap.bin>
//
// 或用 notes 仓库的一键脚本：
//   work/spi-runtime/h104/run-regression.sh
//
// 判据（对着 work/spi-runtime/h104/h104-verify.log 的数字，两边必须一致）：
//   帧数 98 / CRC 坏 0 / HI91 98 / 帧距恒 82 / 静止 acc_z ≈ −9.81 m/s²

#include "h104_protocol.hpp"

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <vector>

using namespace hipnuc_h104;

static std::vector<Sample> g_samples;

static void on_sample(void *ctx, const Sample &s)
{
	(void)ctx;
	g_samples.push_back(s);
}

static int g_fail = 0;

static void check(bool ok, const char *what, const char *detail = "")
{
	printf("  %-52s %s%s%s\n", what, ok ? "OK" : "*** FAIL ***",
	       detail[0] ? "   " : "", detail);
	if (!ok) { g_fail++; }
}

int main(int argc, char **argv)
{
	const char *path = (argc > 1) ? argv[1] : "h104-cap.bin";

	FILE *f = fopen(path, "rb");

	if (!f) {
		fprintf(stderr, "打不开 %s\n", path);
		return 2;
	}

	std::vector<uint8_t> buf;
	uint8_t tmp[4096];
	size_t n;

	while ((n = fread(tmp, 1, sizeof(tmp), f)) > 0) {
		buf.insert(buf.end(), tmp, tmp + n);
	}

	fclose(f);

	printf("== 文件 %s : %zu 字节 ==\n", path, buf.size());

	// ---- 1) 分块喂入（模拟真实串口：帧被任意切开）----
	Parser parser;
	parser.setCallback(on_sample, nullptr);

	const size_t chunk = 97;   // 刻意取一个与 82 互质的非整除块，逼出跨块半帧

	for (size_t off = 0; off < buf.size(); off += chunk) {
		const size_t len = (buf.size() - off < chunk) ? (buf.size() - off) : chunk;
		parser.feed(buf.data() + off, len);
	}

	const Stats &st = parser.stats();
	printf("\n[1] 分块喂入（chunk=%zu）\n", chunk);
	printf("    bytes=%u frames=%u crc_ok=%u crc_bad=%u len_bad=%u resync=%u hi91=%u\n",
	       st.bytes, st.frames, st.crc_ok, st.crc_bad, st.len_bad, st.resync, st.hi91_ok);
	check(st.frames == 98, "帧数 == 98");
	check(st.crc_bad == 0, "CRC 坏帧 == 0");
	check(st.hi91_ok == 98, "HI91 帧 == 98");
	check(st.len_bad == 0, "长度非法 == 0");
	check(g_samples.size() == 98, "回调收到 98 个样本");

	// ---- 2) 一次性喂入，结果必须完全一致（跨块状态正确）----
	Parser p2;
	p2.setCallback(on_sample, nullptr);
	std::vector<Sample> s2;
	g_samples.clear();
	p2.setCallback(on_sample, nullptr);
	p2.feed(buf.data(), buf.size());
	check(p2.stats().frames == st.frames && p2.stats().crc_bad == st.crc_bad,
	      "整块喂入与分块喂入结果一致");
	check(g_samples.size() == 98, "整块喂入也收到 98 个样本");

	// ---- 3) 物理/单位自检：静止时 acc_z ≈ −9.81 m/s²（FRD，Z 朝下）----
	double sum[3] = {0, 0, 0};
	double magsum = 0;

	for (const Sample &s : g_samples) {
		double m = 0;

		for (int a = 0; a < 3; a++) {
			sum[a] += s.acc_m_s2[a];
			m += s.acc_m_s2[a] * s.acc_m_s2[a];
		}

		magsum += sqrt(m);
	}

	const double cnt = static_cast<double>(g_samples.size());
	printf("\n[2] 换算到 FRD/m/s² 后（静止）\n");
	printf("    acc 均值 = (%.4f, %.4f, %.4f) m/s²   |acc| 均值 = %.4f\n",
	       sum[0] / cnt, sum[1] / cnt, sum[2] / cnt, magsum / cnt);
	check(fabs(sum[2] / cnt + 9.80665) < 0.05, "acc_z ≈ −9.81 m/s²（水平静止、FRD 正确）",
	      "手册示例帧若错位则必然失败");
	check(fabs(magsum / cnt - 9.80665) < 0.05, "|acc| ≈ 9.81 m/s²（单位换算正确）");
	check(fabs(sum[0] / cnt) < 0.2 && fabs(sum[1] / cnt) < 0.2, "acc_x / acc_y ≈ 0");

	// ---- 4) 陀螺：静止应接近 0，单位换算成 rad/s（数值应很小）----
	double gsum[3] = {0, 0, 0};

	for (const Sample &s : g_samples) {
		for (int a = 0; a < 3; a++) { gsum[a] += s.gyro_rad_s[a]; }
	}

	printf("    gyro 均值 = (%.6f, %.6f, %.6f) rad/s\n",
	       gsum[0] / cnt, gsum[1] / cnt, gsum[2] / cnt);
	check(fabs(gsum[0] / cnt) < 0.01 && fabs(gsum[1] / cnt) < 0.01 && fabs(gsum[2] / cnt) < 0.01,
	      "gyro 三轴均值 ≈ 0 rad/s");

	// ---- 5) 时间戳增量恒 10 ms → 100 Hz ----
	uint32_t same_delta = 0;

	for (size_t i = 1; i < g_samples.size(); i++) {
		if (g_samples[i].system_time_ms - g_samples[i - 1].system_time_ms == 10) { same_delta++; }
	}

	check(same_delta == g_samples.size() - 1, "system_time 增量恒 10 ms ⇒ 100 Hz");

	// ---- 6) 温度 ----
	printf("    温度 = %.0f °C\n", static_cast<double>(g_samples[0].temperature_c));
	check(fabs(g_samples[0].temperature_c - 32) < 0.5, "温度 ≈ 32 °C");

	printf("\n==== %s (%d 项失败) ====\n", g_fail ? "回归失败" : "回归全部通过", g_fail);
	return g_fail ? 1 : 0;
}
