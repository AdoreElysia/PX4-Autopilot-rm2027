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

/* 本板 IO 定时器（来源：原理图资源汇总表 + 直读引脚）
 *
 *   DShot1..4 = TIM1_CH1..CH4 = PA8 / PE11 / PE13 / PE14
 *   （TIM1 是高级定时器，DShot 需要它；DMA 映射照抄 3dr/ctrl-n1 ——
 *     该板同为 H7 且 TIM1 通道引脚与本板完全一致）
 *
 * 本板未接 ESC 的备用定时器（原理图上有，可后续启用）：
 *   TIM8_CH3/CH4 = PC8/PC9     TIM15_CH1/CH1N = PE5/PE4
 *   TIM16_CH1    = PB8         TIM17_CH1N     = PB7
 *   TIM5_CH1/2/3 = PA0/PA1/PA2 TIM3_CH4      = PB1
 *
 * 通道数上限 MAX_TIMER_IO_CHANNELS = DIRECT_PWM_OUTPUT_CHANNELS = 4
 * （board_config.h 里设的），所以这里只能列 4 条。
 */

constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
	initIOTimer(Timer::Timer1, DMA{DMA::Index1}),
};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel1}, {GPIO::PortA, GPIO::Pin8}),   /* ESC1 */
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel2}, {GPIO::PortE, GPIO::Pin11}),  /* ESC2 */
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel3}, {GPIO::PortE, GPIO::Pin13}),  /* ESC3 */
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel4}, {GPIO::PortE, GPIO::Pin14}),  /* ESC4 */
};

constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
	initIOTimerChannelMapping(io_timers, timer_io_channels);

/* 本板没有 LED PWM；象征性给一个未占用定时器，避免引用空数组。
 * （LED PWM 驱动未在本板 default.px4board 中启用）
 */
constexpr io_timers_t led_pwm_timers[MAX_LED_TIMERS] = {
	initIOTimer(Timer::Timer16),
};
