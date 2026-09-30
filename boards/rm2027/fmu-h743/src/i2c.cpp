/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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

#include <px4_arch/i2c_hw_description.h>

/* 本板 I2C 总线（来源：原理图直读）
 *
 *   I2C2 = SPA06-003 气压计    SCL=PB10  SDA=PB11
 *   I2C4 = 磁罗盘              SCL=PD12  SDA=PD13
 *
 * ⚠️ 磁罗盘型号：原理图写的是 IST8310，但【实物不是它】。
 *    实测（2026-09-30）：
 *      af9838  -I -b 4 start   → af9838 #0 on I2C bus 4 address 0xC  ✅ 起得来
 *      ist8310 -I -b 4 start   → ERROR（IST8310 在 0x0E，本板无）      ❌
 *      ist8308 / ak09916 / ak8963 → ERROR                            ❌
 *    0x0C 正好是 AF9838 的默认 7bit 地址（af9838_registers.hpp: I2C_ADDR = 0x0C），
 *    故按「图纸写 IST8310、实物焊 AF9838」处理，启动命令用 af9838。
 *
 * ⚠️ 但要注意：af9838.cpp 的 probe() 【不做任何 ID 校验】—— 它只读一次
 *    REG_PCODE(0x00) 并检查 I2C 传输成功，读到什么都算过（只把值打印出来）。
 *    所以"能起来"只证明 0x0C 上有个会 ACK 的器件，还不等于手册级确认它是 AF9838。
 *    要坐实身份，需要拿 AF9838 手册核对 REG_PCODE 的期望值 —— 见 README 的待办。
 *
 * 总线号 = STM32 外设号（与 matek 模板一致：I2C1→1、I2C2→2）。
 * 两条都是板载器件，故都用 Internal。
 * 对应 nuttx defconfig 必须使能 CONFIG_STM32H7_I2C2 / CONFIG_STM32H7_I2C4。
 */
constexpr px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS] = {
	initI2CBusInternal(2),
	initI2CBusInternal(4),
};
