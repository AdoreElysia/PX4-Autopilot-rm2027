// 主机侧单元测试：用一个**模拟的 I²C 从机**回归 SoftI2C 的时序与协议
//
// 不需要板子、不需要 PX4。构建 + 运行：
//   g++ -std=c++17 -Wall -Wextra -O2 -o /tmp/t_softi2c test_softi2c.cpp SoftI2C.cpp && /tmp/t_softi2c
//
// 它验证什么（"没有示波器时能证明的东西"）：
//   1. 空闲时两条线都是高（= 模块板载上拉存在 ⇒ `... test` 的自检判据成立）；
//   2. START / STOP 出现的位置正确，数量匹配；
//   3. ★ 波形规则：**SDA 只在 SCL 低电平期间变化**（START/STOP 两处例外）
//      —— 这是 I²C 最基本的规则，违反就是"从机收到的位全是错的"；
//   4. 写寄存器 → 从机寄存器堆真的被写了；
//   5. 复合传输（写寄存器地址 + 重复 START + 读）能读回刚写的值；
//   6. 多字节读自动递增；
//   7. 地址不匹配时识别出 NACK（= `vl53l0x_softi2c test` 报 "NO ACK" 的判据）。
//
// 从机模型只实现"一根总线 + 一个 7bit 地址 + 256 字节寄存器堆"的最小语义，
// 恰好覆盖 VL53L0X 驱动用到的全部访问方式（写单/多字节、读单/多字节）。
// 位序：MSB first；每个字节后跟 1 个 ACK 拍。

#include "SoftI2C.hpp"

#include <cstdio>
#include <cstring>

using rm2027::SoftI2C;
using rm2027::SoftI2CPins;

static int g_fail = 0;

static void check(bool ok, const char *what, const char *detail = "")
{
	printf("  %-56s %s%s%s\n", what, ok ? "OK" : "*** FAIL ***", detail[0] ? "   " : "", detail);

	if (!ok) { g_fail++; }
}

/* ------------------------------------------------------------------ */
/* 模拟从机                                                             */
/* ------------------------------------------------------------------ */

class MockI2CSlave
{
public:
	static constexpr uint8_t ADDRESS = 0x29;

	enum class State { Idle, Address, WriteData, ReadData };

	void reset()
	{
		_state = State::Idle;
		_bits = 0;
		_byte = 0;
		_reg = 0;
		_drive_low = false;
		_expect_reg_index = false;
		_starts = 0;
		_stops = 0;
		_violations = 0;
	}

	uint8_t  reg(uint8_t addr) const { return _regfile[addr]; }
	uint8_t *regfile() { return _regfile; }

	int starts() const { return _starts; }
	int stops() const { return _stops; }
	int violations() const { return _violations; }
	bool drives_low() const { return _drive_low; }

	/** 主机改变了 SDA 电平（scl_high = 变化瞬间 SCL 的电平） */
	void on_sda_change(bool scl_high, bool sda_now)
	{
		if (!scl_high) { return; }   /* SCL 低时改 SDA = 正常的数据位 */

		/* SCL 高时改 SDA 只有两种合法含义：START（下降）或 STOP（上升）。
		 *
		 * 合法时机：字节边界。允许 _bits == 1 的容差 —— 因为主机在 START/STOP 之前
		 * 必须先把 SCL 拉高（教科书时序：SCL 低 → 拉高 SCL → 再动 SDA），
		 * 这个"空"SCL 脉冲会被从机的位计数器当成 1 个位，属正常的 I²C 行为。
		 * 真正要抓的是"一个字节中间被改"（_bits >= 2）——那才会让数据错位。 */
		if (_bits > 1) {
			_violations++;
#ifdef TRACE_VIOLATION
			printf("      [violation] state=%d bits=%d scl_high=%d sda_now=%d\n",
			       (int)_state, _bits, scl_high ? 1 : 0, sda_now ? 1 : 0);
#endif
		}

		if (!sda_now) {
			_starts++;
			_state = State::Address;
			_bits = 0;
			_byte = 0;
			_drive_low = false;

		} else {
			_stops++;
			_state = State::Idle;
			_bits = 0;
			_drive_low = false;
		}
	}

	/** SCL 上升沿：所有参与者在这一刻采样 SDA */
	void on_scl_rising(bool sda_now)
	{
		if (_state == State::Idle) { return; }

		if (_bits < 8) {
			if (_state == State::Address || _state == State::WriteData) {
				_byte = static_cast<uint8_t>((_byte << 1) | (sda_now ? 1 : 0));
			}

			_bits++;

			if (_bits == 8) {
				if (_state == State::Address) {
					_rw = (_byte & 0x1) != 0;
					_addr_match = ((_byte >> 1) == ADDRESS);

				} else if (_state == State::WriteData) {
					if (_expect_reg_index) {
						/* VL53L0X 的写时序：地址字节之后的**第一个数据字节是寄存器号** */
						_reg = _byte;
						_expect_reg_index = false;

					} else {
						_regfile[_reg] = _byte;               /* 收满 8 位就落盘 */
						_reg = static_cast<uint8_t>(_reg + 1);
					}
				}
			}

			return;
		}

		/* _bits == 8：刚才那一拍是 ACK 拍（主机在这一拍采样），本字节结束。
		 * ⚠️ 这里**不放开 SDA** —— 从机必须把 ACK 保持到随后的 SCL 下降沿，
		 *    否则主机在上升沿之后就采不到 ACK 了（这正是第一版测试挂掉的原因）。 */
		_bits = 0;

		if (_state == State::Address) {
			_state = _addr_match ? (_rw ? State::ReadData : State::WriteData) : State::Idle;
			_expect_reg_index = (_state == State::WriteData);   /* 下一字节是寄存器号 */

		} else if (_state == State::ReadData) {
			_reg = static_cast<uint8_t>(_reg + 1);                /* 一个字节交付完，指针递增 */

			/* 读的最后一拍由主机回 ACK/NACK：NACK（主机放开 SDA=高）⇒ 主机不要了，
			 * 从机必须收工、吐出总线，否则会一直把 SDA 拉着 → 总线回不到空闲、
			 * STOP 也发不出来（这正是第二版测试挂掉的原因）。 */
			if (sda_now) {
				_state = State::Idle;
				_drive_low = false;
			}
		}
	}

	/** SCL 下降沿：从机准备好它在"下一个上升沿"要驱动的电平 */
	void on_scl_falling()
	{
		switch (_state) {
		case State::Address:
			/* 8 位收完 ⇒ 下一拍是 ACK；地址匹配就拉低（ACK），否则放开（NACK） */
			_drive_low = (_bits == 8) && _addr_match;
			break;

		case State::WriteData:
			_drive_low = (_bits == 8);                           /* 收到一字节 ⇒ ACK */
			break;

		case State::ReadData:
			if (_bits == 8) {
				_drive_low = false;                          /* ACK 拍交给主机 */

			} else {
				_byte = _regfile[_reg];                      /* 送下一拍要采的数据位 */
				_drive_low = ((_byte >> (7 - _bits)) & 0x1) == 0;
			}

			break;

		default:
			_drive_low = false;
			break;
		}
	}

private:
	State   _state{State::Idle};
	int     _bits{0};
	uint8_t _byte{0};
	uint8_t _reg{0};
	bool    _rw{false};
	bool    _addr_match{false};
	bool    _drive_low{false};
	bool    _expect_reg_index{false};
	uint8_t _regfile[256] {};

	int _starts{0};
	int _stops{0};
	int _violations{0};
};

/* ------------------------------------------------------------------ */
/* 模拟引脚（开漏总线）                                                  */
/* ------------------------------------------------------------------ */

class MockPins : public SoftI2CPins
{
public:
	explicit MockPins(MockI2CSlave &s) : _slave(s) {}

	void configure() override
	{
		_m_scl_low = false;
		_m_sda_low = false;
		_last_scl = scl();
		_last_sda = sda();
	}

	void set_scl(bool high) override
	{
		_m_scl_low = !high;

		const bool now_scl = scl();
		const bool now_sda = sda();

		if (now_scl != _last_scl) {
			_edge_count++;

			if (now_scl) {
				_slave.on_scl_rising(now_sda);

			} else {
				_slave.on_scl_falling();
			}

			/* 从机可能在回调里改了它驱动的 SDA 电平 —— 这属于从机行为，
			 * 不喂给"START/STOP 检测"（那里只判**主机**的波形）。 */
		}

		_last_scl = scl();
		_last_sda = sda();
	}

	void set_sda(bool high) override
	{
		_m_sda_low = !high;

		const bool now_sda = sda();

		if (now_sda != _last_sda) {
			_slave.on_sda_change(scl(), now_sda);   /* 这才是主机改 SDA，判 START/STOP */
		}

		_last_scl = scl();
		_last_sda = now_sda;
	}

	bool get_scl() override { return scl(); }
	bool get_sda() override { return sda(); }

	void delay_half_period() override {}

	bool scl() const { return !_m_scl_low; }
	bool sda() const { return !(_m_sda_low || _slave.drives_low()); }

	int edges() const { return _edge_count; }

private:
	MockI2CSlave &_slave;
	bool _m_scl_low{false};
	bool _m_sda_low{false};
	bool _last_scl{true};
	bool _last_sda{true};
	int  _edge_count{0};
};

/* ------------------------------------------------------------------ */

int main()
{
	MockI2CSlave slave;
	slave.reset();
	MockPins pins(slave);
	SoftI2C bus(pins);

	int n = 0;

	printf("=== SoftI2C 主机侧回归（模拟从机，无硬件）===\n");

	check(bus.init() == 0, "初始化返回成功"); n++;
	check(pins.scl() && pins.sda(), "空闲时 SCL/SDA 都是高（上拉存在）"); n++;
	check(slave.starts() == 0 && slave.stops() == 0, "初始化阶段没有多余的 START/STOP"); n++;

	check(bus.probe(MockI2CSlave::ADDRESS), "probe(0x29) 收到 ACK"); n++;
	check(slave.starts() == 1 && slave.stops() == 1, "probe 恰好 1 个 START + 1 个 STOP"); n++;

	check(!bus.probe(0x50), "probe(0x50) 得到 NACK（无此器件）"); n++;

	const uint8_t wr[] = {0x00, 0x42};
	check(bus.write(MockI2CSlave::ADDRESS, wr, 2) == 0, "写寄存器：0x00 = 0x42"); n++;
	check(slave.reg(0x00) == 0x42, "从机寄存器堆里确实变成了 0x42"); n++;

	uint8_t reg_a = 0x00;
	uint8_t val = 0;
	check(bus.write_read(MockI2CSlave::ADDRESS, &reg_a, 1, &val, 1) == 0, "复合传输：写地址 + 重复 START + 读"); n++;
	check(val == 0x42, "读回值 = 0x42"); n++;

	const uint8_t multi[] = {0x10, 0xAA, 0x55, 0x01, 0x80};
	check(bus.write(MockI2CSlave::ADDRESS, multi, 5) == 0, "多字节写：0x10..0x13"); n++;

	uint8_t reg_b = 0x10;
	uint8_t buf[4] {};
	check(bus.write_read(MockI2CSlave::ADDRESS, &reg_b, 1, buf, 4) == 0, "多字节读（寄存器地址自动递增）"); n++;
	check(buf[0] == 0xAA && buf[1] == 0x55 && buf[2] == 0x01 && buf[3] == 0x80,
	      "读回 4 字节内容与写入一致"); n++;

	check(bus.errors() == 0, "全程 I²C 错误计数为 0"); n++;
	check(bus.nacks() == 1, "NACK 计数正好 1（就是上面那次 0x50）"); n++;
	check(pins.scl() && pins.sda(), "结束时总线回到空闲高"); n++;
	check(slave.violations() == 0, "★ 波形规则：SDA 只在 SCL 低时变化（START/STOP 除外）"); n++;

	printf("\n共 %d 项检查，失败 %d 项\n", n, g_fail);
	return g_fail == 0 ? 0 : 1;
}
