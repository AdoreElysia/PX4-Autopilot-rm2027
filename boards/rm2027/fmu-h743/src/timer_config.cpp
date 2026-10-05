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

#include <px4_arch/io_timer.h>
#include <px4_arch/io_timer_hw_description.h>

/* 本板 IO 定时器（来源：FMU 板网表实际连线 —— 客户端导出的 Protel 2.0 网表）
 *
 *   ESC1 = 座 U13 → PE11 = TIM1_CH2
 *   ESC2 = 座 U11 → PE13 = TIM1_CH3
 *   ESC3 = 座 U14 → PC8  = TIM8_CH3
 *   ESC4 = 座 U15 → PC9  = TIM8_CH4
 *   （座子编号由三路证据交叉确定：信号网 ESC_DSHOT1..4、电流采样
 *     ADC1_IN10/IN11/IN4/ADC2_IN8 → PC0/PC1/PC4/PC5、遥测串口
 *     UART4/USART6/UART7/UART8 —— 三者的座子顺序一致。）
 *
 * ⚠️ 四路 ESC 横跨【两个】高级定时器（TIM1 两路 + TIM8 两路），
 *    所以 io_timers[] 里必须同时登记 TIM8。PX4 的 DShot 按定时器组驱动
 *    （src/drivers/dshot/DShot.cpp 遍历 MAX_IO_TIMERS，每组有独立速率参数
 *    <prefix>_TIM<n>）；DMA 映射照抄同为 H7、且引脚组合完全相同的
 *    aedrox/aedroxh7（TIM1 + TIM8，两者都是 DMA::Index1）。
 *
 * ⚠️ PA8(TIM1_CH1) / PE14(TIM1_CH4) 在图上【悬空】（网表里各自只有 U1 一个
 *    成员，不连任何座子），不是 ESC 输出。原理图上作者批注
 *    「pa7 pA8 改成timer1ch1一对互补？spi去，用spi2」——PA8 当时被预留作
 *    其它用途。上一版误把这两个脚当作 ESC1/ESC4，后果是 1、4 号电机无输出、
 *    2、3 号错位（详见本次提交说明）。
 *
 * 本板其余可用定时器（原理图「资源汇总 / All Timer For H Bridge」列出，可后续启用）：
 *   TIM15_CH1/CH1N = PE5/PE4   TIM16_CH1 = PB8      TIM17_CH1N = PB7
 *   TIM5_CH1/2/3 = PA0/PA1/PA2 TIM3_CH4  = PB1
 *
 * 通道数上限 MAX_TIMER_IO_CHANNELS = DIRECT_PWM_OUTPUT_CHANNELS = 4
 * （board_config.h 里设的），所以这里只能列 4 条。
 */

constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
	initIOTimer(Timer::Timer1, DMA{DMA::Index1}),
	initIOTimer(Timer::Timer8, DMA{DMA::Index1}),	/* ESC3/ESC4 在 TIM8 上 */
};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel2}, {GPIO::PortE, GPIO::Pin11}),  /* ESC1 = 座 U13 */
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel3}, {GPIO::PortE, GPIO::Pin13}),  /* ESC2 = 座 U11 */
	initIOTimerChannel(io_timers, {Timer::Timer8, Timer::Channel3}, {GPIO::PortC, GPIO::Pin8}),   /* ESC3 = 座 U14 */
	initIOTimerChannel(io_timers, {Timer::Timer8, Timer::Channel4}, {GPIO::PortC, GPIO::Pin9}),   /* ESC4 = 座 U15 */
};

constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
	initIOTimerChannelMapping(io_timers, timer_io_channels);

/* 本板没有 LED PWM；象征性给一个未占用定时器，避免引用空数组。
 * （LED PWM 驱动未在本板 default.px4board 中启用）
 */
constexpr io_timers_t led_pwm_timers[MAX_LED_TIMERS] = {
	initIOTimer(Timer::Timer16),
};
