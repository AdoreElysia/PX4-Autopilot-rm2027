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
 * @file init.c
 *
 * FMU-specific early startup code. This file implements the
 * board_app_initialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialisation.
 */

#include "board_config.h"

#include <syslog.h>

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <arch/board/board.h>
#include <stm32_rcc.h>
#include "arm_internal.h"

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>
#include <systemlib/px4_macros.h>
#include <px4_arch/io_timer.h>
#include <px4_platform_common/init.h>
#include <px4_platform/gpio.h>
#include <px4_platform/board_dma_alloc.h>

#if defined(FLASH_BASED_PARAMS)
#  include <parameters/flashparams/flashfs.h>
#endif

__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
__END_DECLS

/************************************************************************************
 * Name: board_peripheral_reset
 *
 * Description:
 *
 ************************************************************************************/
__EXPORT void board_peripheral_reset(int ms)
{
	UNUSED(ms);
}

/************************************************************************************
 * Name: board_on_reset
 *
 * Description:
 * Optionally provided function called on entry to board_system_reset
 * It should perform any house keeping prior to the rest.
 *
 * status - 1 if resetting to boot loader
 *          0 if just resetting
 *
 ************************************************************************************/
__EXPORT void board_on_reset(int status)
{
	for (int i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; ++i) {
		px4_arch_configgpio(PX4_MAKE_GPIO_INPUT(io_timer_channel_get_as_pwm_input(i)));
	}

	/*
	 * On resets invoked from system (not boot) ensure we establish a low
	 * output state on PWM pins to disarm the ESC and prevent the reset from potentially
	 * spinning up the motors.
	 */
	if (status >= 0) {
		up_mdelay(100);
	}
}

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/
__EXPORT void stm32_boardinitialize(void)
{
	/* Reset PWM first thing */
	board_on_reset(-1);

	/* configure LEDs */
	board_autoled_initialize();

	/* configure pins */
	const uint32_t gpio[] = PX4_GPIO_INIT_LIST;
	px4_gpio_init(gpio, arraySize(gpio));

	/* configure SPI interfaces */
	stm32_spiinitialize();

	/* 2026-11-04：释放板载 VL53L0CX 的 XSHUT（PC13）。
	 *
	 *   实测：PC13 上电后是 ANALOG + IDR=0，XSHUT 低电平，
	 *   VL53 一直处于 RESET。原理图 R17 10k 上拉未起作用。
	 *   不拉高它，I2C 地址能 ACK 但寄存器全部不响应 ⇒ i2cdetect 全空白。
	 *
	 *   DS11555 §4.1：XSHUT 拉高后 >= 1 ms 才能访问 I2C，这里留 10 ms。
	 */
	px4_arch_configgpio(GPIO_VL53_XSHUT);
	up_mdelay(10);

	/* 本板无 USB，不调用 stm32_usbinitialize() */

}

/************************************************************************************
 * Name: vrefbuf_init
 *
 * Description:
 *   VREF+ 没有外接 3.3V，使用 STM32H7 内部 VREFBUF 作为 ADC 参考。
 *   VSCALE=000 -> 2.5V，对应 board_config.h 的 BOARD_ADC_POS_REF_V。
 *
 ************************************************************************************/
#if defined(BOARD_ADC_USE_VREFBUF)
#define STM32_VREFBUF_BASE   0x58003c00
#define STM32_VREFBUF_CSR    (STM32_VREFBUF_BASE + 0x00)

#define VREFBUF_CSR_ENVR     (1u << 0)
#define VREFBUF_CSR_VRR      (1u << 3)

static void vrefbuf_init(void)
{
	/* VREFBUF 在 APB4，先开时钟 */
	modifyreg32(STM32_RCC_APB4ENR, 0, RCC_APB4ENR_VREFEN);

	/* ENVR=1, HIZ=0, VRS=00 -> 2.5V */
	putreg32(VREFBUF_CSR_ENVR, STM32_VREFBUF_CSR);

	const hrt_abstime start = hrt_absolute_time();

	while ((getreg32(STM32_VREFBUF_CSR) & VREFBUF_CSR_VRR) == 0) {
		if (hrt_elapsed_time(&start) > 10000) {
			syslog(LOG_ERR, "[boot] VREFBUF not ready\n");
			break;
		}

		up_udelay(10);
	}
}
#endif

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX;
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/
__EXPORT int board_app_initialize(uintptr_t arg)
{
	/* Need hrt running before using the ADC */
	px4_platform_init();

#if defined(BOARD_ADC_USE_VREFBUF)
	vrefbuf_init();
#endif

	/* configure the DMA allocator */
	if (board_dma_alloc_init() < 0) {
		syslog(LOG_ERR, "[boot] DMA alloc FAILED\n");
	}

	/* initial LED state */
	drv_led_start();
	led_off(LED_RED);
	led_off(LED_BLUE);

	if (board_hardfault_init(2, true) != 0) {
		led_on(LED_BLUE);
	}

#ifdef CONFIG_MMCSD
	int ret = stm32_sdio_initialize();

	if (ret != OK) {
		led_on(LED_BLUE);
	}

#endif


#if defined(FLASH_BASED_PARAMS)
	/* ★ rm2027/fmu-h743：本板【既没有 SD 卡也没有 USB】，参数无处可存 ——
	 *   不开这个就等于"每次上电参数全丢，标定白做"。所以必须用片内 flash 存。
	 *
	 *   扇区选择：STM32H743 是 2MB 双 bank、每 bank 8 个 128K 扇区
	 *   ⇒ bank2 最后一个扇区 = 扇区 15 = 0x081E0000，正好在 app 区之外。
	 *   参考同 MCU 的 4 块板子（aedrox/aedroxh7、corvon/743v1、corvon/743v2、
	 *   cbunmanned/h753-som）全都是【只用一个扇区 {15, 128K, 0x081E0000}】。
	 *   flashfs 的 get_next_sector_descriptor() 在只有一项时会回到自己，
	 *   所以单扇区是合法配置（代价：没有扇区级磨损均衡）。
	 *   配套要求：script.ld 的 FLASH LENGTH 必须从 1920K 缩到 1792K，
	 *   把 0x081E0000 这个扇区让出来，否则链接器可能把代码放进去、
	 *   参数写入就会打烂代码。
	 */
	static sector_descriptor_t params_sector_map[] = {
		{15, 128 * 1024, 0x081E0000},
		{0, 0, 0},
	};

	/* Initialize the flashfs layer to use heap allocated memory */
	int result = parameter_flashfs_init(params_sector_map, NULL, 0);

	if (result != OK) {
		syslog(LOG_ERR, "[boot] FAILED to init params in FLASH %d\n", result);
		led_on(LED_BLUE);
		return -ENODEV;
	}

#endif

	/* Configure the HW based on the manifest */
	px4_platform_configure();

	return OK;
}
