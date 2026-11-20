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

#pragma once

// DMAMUX1
#define DMAMAP_SPI1_RX    DMAMAP_DMA12_SPI1RX_0 /* DMA1:37 */
#define DMAMAP_SPI1_TX    DMAMAP_DMA12_SPI1TX_0 /* DMA1:38 */

// SPI2 = Bosch BMI088（2026-10-30 新增）。
// 必须有这两行：NuttX 的 stm32_spi.c:479-480 在编译期就引用
// DMAMAP_SPI2_RX/TX，缺了会直接 `error: 'DMAMAP_SPI2_RX' undeclared`
// （而不是运行时报错）。
// 选 _0（DMA1）与本板 SPI1 同源，与 3dr/ctrl-n1、accton/ga1、aedrox/aedroxh7 一致。
#define DMAMAP_SPI2_RX    DMAMAP_DMA12_SPI2RX_0 /* DMA1:39 */
#define DMAMAP_SPI2_TX    DMAMAP_DMA12_SPI2TX_0 /* DMA1:40 */

// SPI3 = 外挂光流 PMW3901（2026-11-18 新增）。
// ★★ 这两行是**编译期必需**的：NuttX 的 stm32_spi.c:551-552 在
//    #ifdef CONFIG_STM32H7_SPI3_DMA 里直接引用 DMAMAP_SPI3_RX/TX。
//    H7 的 dmamux 头只提供 DMAMAP_DMA12_SPI3RX_0/1 与 _TX_0/1 这种全名，
//    没有缩写别名 ⇒ 不在这里补上，stm32_spi.c 会编译失败：
//        error: 'DMAMAP_SPI3_RX' undeclared
//    ⚠️ 这个错误的可怕之处：它发生在 NuttX 子构建里，而 PX4 的增量构建
//       会沿用上一次的 libarch.a（表现为 "编译成功"，但固件里根本没有
//       g_spi3dev 符号，运行时 px4_spibus_initialize(3) 返回 NULL，
//       报 -ENODEV）。2026-11-18 排查很久，根因就是这里缺两行。
// 选 _0（DMA1）与 SPI1/SPI2 保持一致。
#define DMAMAP_SPI3_RX    DMAMAP_DMA12_SPI3RX_0 /* DMA1:61 */
#define DMAMAP_SPI3_TX    DMAMAP_DMA12_SPI3TX_0 /* DMA1:62 */


// DMAMUX2
#define DMAMAP_SPI4_RX    DMAMAP_DMA12_SPI4RX_1 /* DMA2:83 */
#define DMAMAP_SPI4_TX    DMAMAP_DMA12_SPI4TX_1 /* DMA2:84 */
