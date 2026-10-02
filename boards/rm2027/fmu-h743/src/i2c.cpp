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
 * ⚠️ 磁罗盘型号：**原理图写 IST8310，实物也是 IST8310**（不要被"0x0C"误导，见下）。
 *
 *   依据（唯一基准 = iSentek IST8310 数据手册，v1.2）：
 *     §6.4.2 WAI(0x00) Device ID 缺省 = 0x10
 *     §6.4.4 输出寄存器 = 0x03~0x08 (DATAXL/DATAXH/DATAYL/DATAYH/DATAZL/DATAZH)
 *     §6.1.1 从机地址表：CAD1=CAD0=VSS ⇒ 7bit 地址 = 0CH
 *            （VSS/VDD=0DH、VDD/VSS=0EH、VDD/VDD=0FH；悬空为 0EH）
 *     ⇒ **0x0C 是 IST8310 的一种合法配置**，不是"AF9838 专属地址"。
 *     手册：/mnt/h/AI/shared/projects/competition/rm2027-drone/references/硬件手册/
 *           iSentek_IST8310_datasheet_v1.2.pdf（含文字提取版）
 *
 *   ⚠️⚠️ 错误结论的来历（已作废，勿再引用）：
 *     曾按"0x0C=AF9838 默认地址、IST8310 在 0x0E"推断"实物是 AF9838"。
 *     两条推理都不成立：
 *       (a) 0x0C 本身就是 IST8310 的合法地址（手册地址表）；
 *       (b) 当年那次 `ist8310 -I -b 4 start` 报的是 WARN，而 WARN 只在 **-X（外部总线）**
 *           路径上出现 —— -X 下 PX4 不遍历 Internal 总线，**根本没碰过 I2C4**。
 *     而 `af9838` 之所以"起来了"，是因为 af9838.cpp 的 probe() **不校验 ID**
 *     （只读一次 REG_PCODE(0x00) 并检查 I2C 传输成功，读到什么都算过）。
 *     ⇒ 一个不校验 ID 的驱动，把 IST8310 认成了 AF9838。
 *
 *   ✅ 2026-09-30 晚已切换并实测通过：启动命令 = `ist8310 -I -b 4 -a 0x0C -R 0 start`
 *      （**必须带 `-a 0x0C`**：IST8310 驱动缺省 0x0E，其 probe() 只在给定地址上读 WAI，
 *        0x0C~0x0F 那圈只用来写软复位、不用来读；不传 -a 实测报 no instance started）
 *      实测：device_id Type=0x06 / I2C:4 0x0C，|B| = 0.259 G ⇒ 已落进 lm_mag_fit 窗口 (0.2,0.7)。
 *
 *   ⚠️ 之前用 af9838 驱动时的读数是**错的**（|B| 只有 6.9~17.1 µT），三处与手册不符：
 *        标度 0.1（手册 0.3 µT/LSB）、软复位写 0x11（IST8310 无此寄存器，
 *        应为 CNTL2(0x0B) bit0 SRST）、自检写 CNTL1=0x09（手册里是 Reserved，
 *        自检在 STR(0x0C) bit6）。详见 work/spi-runtime/README.md 磁罗盘条目。
 *
 * 总线号 = STM32 外设号（与 matek 模板一致：I2C1→1、I2C2→2）。
 * 两条都是板载器件，故都用 Internal。
 * 对应 nuttx defconfig 必须使能 CONFIG_STM32H7_I2C2 / CONFIG_STM32H7_I2C4。
 */
constexpr px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS] = {
	initI2CBusInternal(2),
	initI2CBusInternal(4),
};
