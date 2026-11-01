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

// ★ rm2027/fmu-h743：本板【既没有 SD 卡也没有 USB】，参数无处可存。
//   不开这个 ⇒ 每次上电参数全丢、标定白做。故开启片内 flash 存参数，
//   占用最后一个 128K 扇区 0x081E0000（配套 script.ld 的 FLASH LENGTH 1792K）。
//   详见 src/init.c 里 params_sector_map 的注释。
#define FLASH_BASED_PARAMS


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
 * v2 板（FMU_1）实际只有一路电池电压检测：
 *   VIN → R32 47K → PC4/ADC1_IN4 → R33 4.7K → GND，C6 100nF 滤波
 *   分压比 = (47K + 4.7K) / 4.7K = 11:1；3.3V ADC 对应最高约 36.3V。
 *
 * v2 FMU 没有 ESC 电流采样：PC0/PC1/PC5 均无外接（PC5/ADC2_IN8 只有 U1
 * 一个焊盘）。旧板的 4 路 ESC 电流表已作废；PC1 只保留为
 * ADC_BATTERY_CURRENT_CHANNEL 的占位通道，配 BAT1_A_PER_V=0 时电流为 0。
 */
#define ADC1_CH(n)                  (n)
#define ADC2_CH(n)                  (n)

#define PX4_ADC_GPIO  \
	/* PC1  */  GPIO_ADC123_INP11, \
	/* PC4  */  GPIO_ADC12_INP4

#define ADC_BATTERY_VOLTAGE_CHANNEL     ADC1_CH(4)    /* PC4，VIN 分压 47K/4.7K */
#define ADC_BATTERY_CURRENT_CHANNEL     ADC1_CH(11)   /* PC1 占位，无电流检测 */
#define BOARD_ADC_BRICK_VALID           1             /* v2 板电池电压检测有效 */

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL) | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL))

/*
 * VREF+ 没有外接 3.3V，只挂了 C31，设计意图是使用 STM32H7 内部 VREFBUF。
 * 在 board_app_initialize() 里使能 VREFBUF，输出 2.5V（VSCALE=000）。
 * 因此 ADC 参考必须是 2.5V，不能再用默认的 3.3V。
 *
 * 注意：BAT1_V_DIV=11 时，最大可测电压 = 2.5V × 11 = 27.5V。
 *       3S/6S 够用，8S 会超过量程。
 */
#define BOARD_ADC_USE_VREFBUF           1
#define BOARD_ADC_POS_REF_V             2.5f


/* CAN **********************************************************************
 * CAN1 = H104 外置陀螺（PD0/PD1）
 * CAN2 = 空闲（PB12/PB13）
 * 本板两路都没有 silent 控制脚（模板板的 PD3 在本板是 SPI2_SCK）
 */


/* PWM / DShot **************************************************************
 * 4 路 DShot，★ 跨两个定时器（2026-10-06 按原理图网表修正，此前写错了）：
 *   路1 TIM1_CH3 = PE13 → P1.8    路2 TIM1_CH2 = PE11 → P1.7
 *   路3 TIM8_CH3 = PC8  → P1.6    路4 TIM8_CH4 = PC9  → P1.5
 * ★ 2026-10-31 v2 板核对：P1.8=DSHOT1=PE13、P1.7=DSHOT2=PE11；
 *   旧板 U13/U11 的顺序（PE11=DSHOT1、PE13=DSHOT2）与 v2 相反，
 *   timer_config.cpp 已按 P1 交换前两路。
 * ⚠️ PA8(TIM1_CH1) / PE14(TIM1_CH4) 在本板是【悬空单点网络】，没接 ESC 座，
 *    不要再用 —— 旧版本把路1/路4 指到这两个脚，导致那两个座永远收不到信号。
 * 这里按已接的 4 路报给上层。
 */
#define DIRECT_PWM_OUTPUT_CHANNELS   4


/* 备用 GPIO（均为本板未连接引脚，留着给上层当普通 IO 用） *******************
 * 仅列出原理图上确实悬空的脚，避免误驱动已占用引脚。
 */
/* 注：PD10/PD11 已被上面的占位 LED 占用，不再作为备用 GPIO */


/* VL53L0X 的临时引脚（U18 / PE7 / PE8）已于 2026-10-18 测试完成后撤销，
 * UART7 恢复给 ESC3 串口遥测使用。完整记录与重新启用步骤见：
 *   docs/交接-VL53L0X测距-2026-10-18.md
 */


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
