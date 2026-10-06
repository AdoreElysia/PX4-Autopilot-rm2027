/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file board_config.h
 *
 * Board internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

// #define FLASH_BASED_PARAMS


/* ============================================================================
 * 本板：rm2027/fmu-h743（自研 STM32H743VIT6 飞控板）
 *
 * 引脚来源：原理图 V1.0 图面直读（见仓库 firmware/fmu-h743/引脚映射-H743.md）
 * 与模板板 matek/h743-slim 的差异集中在本文件 + nuttx-config/include/board.h
 *
 * 本板特点：
 *   · 没有指示灯（原理图零个 LED 网络）
 *   · 没有蜂鸣器/tone alarm
 *   · 没有 USB
 *   · 没有 microSD
 *   · 有 4 路独立 ESC 电流采样（PC0/PC1/PC4/PC5）
 *   · 有独立 CAN2 收发器
 * ========================================================================== */


/* 指示灯（占位） ************************************************************
 * 本板**没有硬件 LED**。为规避「PX4 无 0 灯板先例」的构建风险，
 * 把两个逻辑 LED 映射到原理图上悬空的 PD10 / PD11：
 * 驱动能跑、能编译，只是物理上看不见。
 * 真要做指示灯 → 飞线到这两个脚即可，无需改代码。
 */
#define GPIO_nLED_BLUE          /* PD10（本板悬空，占位） */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN10)
#define GPIO_nLED_GREEN         /* PD11（本板悬空，占位） */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN11)

#define BOARD_HAS_CONTROL_STATUS_LEDS   1
#define BOARD_ARMED_STATE_LED           1
#define BOARD_OVERLOAD_LED              0


/* ADC 通道 ******************************************************************
 * 4 路 ESC 电流采样（一路一个）：
 *   ESC1 → PC0 (ADC1_INP10)      ESC2 → PC1 (ADC1_INP11)
 *   ESC3 → PC4 (ADC12_INP4)      ESC4 → PC5 (ADC12_INP8)
 *
 * ⚠️ 本板**没有**电池总电压采样（全图未见 VBAT 分压）。因此不定义
 *    ADC_BATTERY_VOLTAGE_CHANNEL；PX4 的电池模块需据此配置，
 *    否则会读不到电压。见 firmware/fmu-h743/开发日志/04 的 Q-02b。
 */
#define ADC1_CH(n)                  (n)
#define ADC2_CH(n)                  (n)

#define PX4_ADC_GPIO  \
	/* PC0  */  GPIO_ADC123_INP10, \
	/* PC1  */  GPIO_ADC123_INP11, \
	/* PC4  */  GPIO_ADC12_INP4,  \
	/* PC5  */  GPIO_ADC12_INP8

/* 索引 0..3 = ESC1..ESC4 电流（顺序与原理图座子 U13/U11/U14/U15 对应） */
#define ADC_ESC1_CURRENT_CHANNEL        ADC1_CH(10)
#define ADC_ESC2_CURRENT_CHANNEL        ADC1_CH(11)
#define ADC_ESC3_CURRENT_CHANNEL        ADC1_CH(4)
#define ADC_ESC4_CURRENT_CHANNEL        ADC2_CH(8)

#define ADC_CHANNELS \
	((1 << ADC_ESC1_CURRENT_CHANNEL) | \
	 (1 << ADC_ESC2_CURRENT_CHANNEL) | \
	 (1 << ADC_ESC3_CURRENT_CHANNEL) | \
	 (1 << ADC_ESC4_CURRENT_CHANNEL))

/* ⚠️⚠️ 电池通道「占位」——必须在确认电压来源后修正 ⚠️⚠️
 *
 * PX4 的 board_common.h 在编译期就要求 ADC_BATTERY_VOLTAGE_CHANNEL /
 * ADC_BATTERY_CURRENT_CHANNEL 存在，否则 battery_status 模块编不过。
 * 但本板**没有电池电压采样**（原理图全图未见 VBAT 分压）。
 *
 * 因此这里指向 PC0/PC1 —— 请注意它们物理上分别是 ESC1/ESC2 的**电流**采样，
 * pay 不是电压。为了让 PX4 忽略这块"砖"，同时定义：
 *     BOARD_ADC_BRICK_VALID 0
 * 于是模块能编译，但不会把错误的电压当成电池电压参与 failsafe。
 *
 * 待硬件确认电压来自哪里（ESC 板经 UART 上报？还是需要飞线？）后：
 *   1. 修正这两个宏
 *   2. 把 BOARD_ADC_BRICK_VALID 改回 1（或不定义）
 * 见 firmware/fmu-h743/开发日志/04-待确认与风险.md 的 Q-02b
 */
#define ADC_BATTERY_VOLTAGE_CHANNEL     ADC1_CH(10)   /* 占位，物理是 ESC1 电流 */
#define ADC_BATTERY_CURRENT_CHANNEL     ADC1_CH(11)   /* 占位，物理是 ESC2 电流 */
#define BOARD_ADC_BRICK_VALID           0             /* 本板无有效电池砖 */


/* CAN **********************************************************************
 * CAN1 = H104 外置陀螺（PD0/PD1）
 * CAN2 = 空闲（PB12/PB13）
 * 本板两路都没有 silent 控制脚（模板板的 PD3 在本板是 SPI2_SCK）
 */


/* PWM / DShot **************************************************************
 * 4 路 DShot，★ 跨两个定时器（2026-10-06 按原理图网表修正，此前写错了）：
 *   路1 TIM1_CH2 = PE11 → 座 U13    路2 TIM1_CH3 = PE13 → 座 U11
 *   路3 TIM8_CH3 = PC8  → 座 U14    路4 TIM8_CH4 = PC9  → 座 U15
 * ⚠️ PA8(TIM1_CH1) / PE14(TIM1_CH4) 在本板是【悬空单点网络】，没接 ESC 座，
 *    不要再用 —— 旧版本把路1/路4 指到这两个脚，导致那两个座永远收不到信号。
 * 这里按已接的 4 路报给上层。
 */
#define DIRECT_PWM_OUTPUT_CHANNELS   4


/* 备用 GPIO（均为本板未连接引脚，留着给上层当普通 IO 用） *******************
 * 仅列出原理图上确实悬空的脚，避免误驱动已占用引脚。
 */
/* 注：PD10/PD11 已被上面的占位 LED 占用，不再作为备用 GPIO */


/* 蜂鸣器：本板没有 ********************************************************* */
// #define GPIO_TONE_ALARM_IDLE ...
// #define GPIO_TONE_ALARM_GPIO ...


/* USB：本板没有 ************************************************************ */
// #define GPIO_OTGFS_VBUS ...


/* 高分辨率定时器 ***********************************************************
 * TIM2 在本板未被占用（原理图「资源汇总」表里没有 TIM2）→ 沿用模板设置。
 */
#define HRT_TIMER               2
#define HRT_TIMER_CHANNEL       1


/* RC 串口 ******************************************************************
 * 本板遥控走 UART5（DBUS）。/dev/ttySx 编号取决于 nuttx defconfig 里的
 * UART 使能顺序，需在 defconfig 定稿后核对（见开发日志 Q-08）。
 */
#define RC_SERIAL_PORT          "/dev/ttyS4"
#define BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT


/* SD 卡：本板没有 ********************************************************** */
// #define SDIO_SLOTNO 0
// #define SDIO_MINOR  0


/* DMA / 复位 *************************************************************** */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120
#define BOARD_HAS_ON_RESET 1


#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO, \
		GPIO_CAN1_TX, \
		GPIO_CAN1_RX, \
		GPIO_CAN2_TX, \
		GPIO_CAN2_RX, \
		GPIO_nLED_BLUE, \
		GPIO_nLED_GREEN, \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER
#define BOARD_NUM_IO_TIMERS 4


__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spiinitialize
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
