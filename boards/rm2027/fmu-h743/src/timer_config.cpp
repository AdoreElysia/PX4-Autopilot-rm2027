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

/* 本板 IO 定时器 —— ★ 引脚表以【本板原理图网表】为准，不要从别的板复制。
 *
 *   ESC 插座 → 网络 → H7 引脚 → 定时器通道：
 *     U13.pin3  ESC_DSHOT1  PE11  TIM1_CH2
 *     U11.pin3  ESC_DSHOT2  PE13  TIM1_CH3
 *     U14.pin3  ESC_DSHOT3  PC8   TIM8_CH3
 *     U15.pin3  ESC_DSHOT4  PC9   TIM8_CH4
 *
 * ★★ 2026-10-06 修正（此前是错的，会害死人）：
 *   旧版写 "DShot1..4 = TIM1_CH1..CH4 = PA8 / PE11 / PE13 / PE14"，
 *   那是从 boards/3dr/ctrl-n1 整段照抄来的。PE11/PE13 两根恰好与本板一致，
 *   所以前两路能转、看着"没问题"，把另两根的错误藏了很久。
 *   实测（原理图网表 + fmu_2026-09-24.pdf 双证）：PA8(网络名 TIM1_CH1) 与
 *   PE14(网络名 TIM1_CH4) 都是【单点网络】—— 整条网只有 MCU 自己一个焊盘，
 *   没有接到任何 ESC 座 ⇒ 旧版第 1/第 4 路打在悬空脚上，插在那两个座的
 *   电调永远收不到 DShot。
 *
 *   本板 DShot 实际跨【两个】定时器，所以 io_timers[] 必须有两条：
 *     TIM1 只用 CH2/CH3（CH1 空着）、TIM8 只用 CH3/CH4（CH1/CH2 空着）
 *   通道有缺口是 PX4 明确支持的：io_timer.c:624/646 会取组内最低通道号设 rDCR，
 *   dshot.c:349/392 用 channel_count_including_gaps。
 *   上游同类先例（前 4 路主输出跨多定时器）共 19 块板，最像的是
 *   boards/cbunmanned/h753-som（:43-44 就是 PE11=TIM1_CH2 / PE13=TIM1_CH3）。
 *
 *   副作用（改完必须做）：会多生成一个参数 PWM_MAIN_TIM1，
 *   它与 PWM_MAIN_TIM0 都带 @reboot_required，且【两组协议必须一致】。
 *
 * 本板其余可用定时器（原理图上有，本板未接 ESC）：
 *   TIM1_CH1/CH4 = PA8/PE14（★ 悬空单点网络，不可用）
 *   TIM15_CH1/CH1N = PE5/PE4   TIM16_CH1 = PB8    TIM17_CH1N = PB7
 *   TIM5_CH1/2/3 = PA0/PA1/PA2 TIM3_CH4  = PB1
 *
 * 通道数上限 MAX_TIMER_IO_CHANNELS = DIRECT_PWM_OUTPUT_CHANNELS = 4
 * （board_config.h 里设的），所以这里只能列 4 条。
 * ★ 写数组的顺序 = 输出路号（PWM_MAIN_FUNC 的 n）：同一 timer 的必须挨着，
 *   因为 Tools/module_config/output_groups_from_timer_config.py:133 是按
 *   连续同 timer 分组（groupby）来生成参数的，交错会解析错。
 */

constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
	initIOTimer(Timer::Timer1, DMA{DMA::Index1}),   // index 0 → 生成 PWM_MAIN_TIM0
	initIOTimer(Timer::Timer8, DMA{DMA::Index1}),   // index 1 → 生成 PWM_MAIN_TIM1
	// ★ DMA 只写 Index1 就够：hw_description.h:242 会据此查 DMAMAP_DMA12_TIM8UP_0/1，
	//   且 :258 有 constexpr_assert(dma_map_up != 0) ⇒ 配错【编译期就失败】。
};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel2}, {GPIO::PortE, GPIO::Pin11}),  /* 路1 ESC_DSHOT1 → U13 */
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel3}, {GPIO::PortE, GPIO::Pin13}),  /* 路2 ESC_DSHOT2 → U11 */
	initIOTimerChannel(io_timers, {Timer::Timer8, Timer::Channel3}, {GPIO::PortC, GPIO::Pin8}),   /* 路3 ESC_DSHOT3 → U14 */
	initIOTimerChannel(io_timers, {Timer::Timer8, Timer::Channel4}, {GPIO::PortC, GPIO::Pin9}),   /* 路4 ESC_DSHOT4 → U15 */
};

constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
	initIOTimerChannelMapping(io_timers, timer_io_channels);

/* 本板没有 LED PWM；象征性给一个未占用定时器，避免引用空数组。
 * （LED PWM 驱动未在本板 default.px4board 中启用）
 */
constexpr io_timers_t led_pwm_timers[MAX_LED_TIMERS] = {
	initIOTimer(Timer::Timer16),
};
