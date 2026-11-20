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

#pragma once

/****************************************************************************
 * 10-8--2016:
 *  To simplify the ripple effect on the tools, we will be using
 *  /dev/serial/by-id/<asterisk>PX4<asterisk> to locate PX4 devices. Therefore
 *  moving forward all Bootloaders must contain the prefix "PX4 BL "
 *  in the USBDEVICESTRING
 *  This Change will be made in an upcoming BL release
 ****************************************************************************/
/*
 * Define usage to configure a bootloader
 *
 *
 * Constant                example          Usage
 * APP_LOAD_ADDRESS     0x08004000            - The address in Linker Script, where the app fw is org-ed
 * BOOTLOADER_DELAY     5000                  - Ms to wait while under USB pwr or bootloader request
 * BOARD_FMUV2
 * INTERFACE_USB        1                     - (Optional) Scan and use the USB interface for bootloading
 * INTERFACE_USART      1                     - (Optional) Scan and use the Serial interface for bootloading
 * USBDEVICESTRING      "PX4 BL FMU v2.x"     - USB id string
 * USBPRODUCTID         0x0011                - PID Should match defconfig
 * BOARD_TYPE           9                     - Must match .prototype boad_id
 * _FLASH_KBYTES        (*(uint16_t *)0x1fff7a22) - Run time flash size detection
 * BOARD_FLASH_SECTORS  ((_FLASH_KBYTES == 0x400) ? 11 : 23) - Run time determine the physical last sector
 * BOARD_FLASH_SECTORS   11                   - Hard coded zero based last sector
 * BOARD_FLASH_SIZE     (_FLASH_KBYTES*1024)-   Total Flash size of device, determined at run time.
 *                         (1024 * 1024)      - Hard coded Total Flash of device - The bootloader and app reserved will be deducted
 *                                              programmatically
 *
 * BOARD_FIRST_FLASH_SECTOR_TO_ERASE  2        - Optional sectors index in the flash_sectors table  (F4 only), to begin erasing.
 *                                               This is to allow sectors to be reserved for app fw usage. That will NOT be erased
 *                                               during a FW upgrade.
 *                                               The default is 0, and selects the first sector to be erased, as the 0th entry in the
 *                                               flash_sectors table. Which is the second physical sector of FLASH in the device.
 *                                               The first physical sector of FLASH is used by the bootloader, and is not defined
 *                                               in the table.
 *
 * APP_RESERVATION_SIZE (BOARD_FIRST_FLASH_SECTOR_TO_ERASE * 16 * 1024) - Number of bytes reserved by the APP FW. This number plus
 *                                                                        BOOTLOADER_RESERVATION_SIZE  will be deducted from
 *                                                                        BOARD_FLASH_SIZE to determine the size of the App FW
 *                                                                        and hence the address space of FLASH to erase and program.
 * USBMFGSTRING            "PX4 AP"            - Optional USB MFG string (default is '3D Robotics' if not defined.)
 * SERIAL_BREAK_DETECT_DISABLED                -  Optional prevent break selection on Serial port from entering or staying in BL
 *
 * * Other defines are somewhat self explanatory.
 */

/* Boot device selection list*/
#define USB0_DEV       0x01
#define SERIAL0_DEV    0x02
#define SERIAL1_DEV    0x04

#define APP_LOAD_ADDRESS               0x08020000
#define BOOTLOADER_DELAY               5000
/* ★ 本板（rm2027/fmu-h743）**没有 USB 硬件**，也**没有引出的 USART1**
 *
 * ★★ 2026-11-18 实测结论：必须把 INTERFACE_USART 关成 0。
 *
 *   原来这里写 INTERFACE_USART=1 / "/dev/ttyS0,57600"，理由是"模板板就有串口通道"。
 *   但本板原理图与网表（work/FMU2-引脚识别-20261030/netlist-FMU_1.txt）里
 *   **USART1 的四个脚 PA9/PA10/PB14/PB15 一个焊点都没有** —— 全部悬空。
 *
 *   后果（实测踩到，排查了很久）：
 *     1. bootloader 在 USART1 上等上传命令（bl.c:642-662 的 while(true) 循环）；
 *     2. PA10(RX) 浮空，收到噪声；
 *     3. 噪声被当成命令字节 ⇒ 拼出 GET_SYNC/CHIP_ERASE ⇒ **把 app 区整片擦掉**；
 *     4. 擦完之后 jump_to_app() 发现向量表是 0xFFFFFFFF，跳不过去；
 *     5. bootloader 顺手把 BOOT_RTC_SIGNATURE(0xb007b007) 写进 RTC_BK0R；
 *     6. 之后每次复位都因为读到这个签名而 timeout=0，**永远停在 bootloader**。
 *   现象就是："固件明明烧进去了，一复位就回到 bootloader，app 区变成 ffffffff"。
 *
 *   ⇒ 关掉 USART 通道后，bootloader 只做"延时 5 s → 跳 app"，
 *     既不擦 flash，也不写真签名。
 *
 * ⚠️ 代价：**没有串口上传固件的通道了**。本板本来就只能用 ST-Link 烧写
 *   （README §3.5、§6 也是这个结论），所以没有实际损失。
 *   以后若真把 USART1 引到座子上，把这两行改回 1 即可。
 */
#define INTERFACE_USB                  0
// #define INTERFACE_USB_CONFIG        "/dev/ttyACM0"
// #define BOARD_VBUS                  MK_GPIO_INPUT(GPIO_OTGFS_VBUS)   /* 本板无 VBUS 检测脚 */

//#define USE_VBUS_PULL_DOWN
#define INTERFACE_USART                0
// #define INTERFACE_USART_CONFIG       "/dev/ttyS0,57600"
/* ★ 必须与 firmware.prototype 的 board_id 一致（本板 = 2027） */
#define BOARD_TYPE                     2027
#define _FLASH_KBYTES                  (*(uint32_t *)0x1FF1E880)
#define BOARD_FLASH_SECTORS            (15)
#define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

#define OSC_FREQ                       8

#define BOARD_PIN_LED_ACTIVITY         GPIO_nLED_BLUE // BLUE
#define BOARD_PIN_LED_BOOTLOADER       GPIO_nLED_GREEN // GREEN
#define BOARD_LED_ON                   0
#define BOARD_LED_OFF                  1

#define SERIAL_BREAK_DETECT_DISABLED   1

#if !defined(ARCH_SN_MAX_LENGTH)
# define ARCH_SN_MAX_LENGTH 12
#endif

/* ★ 参数区为 app 让出的空间 = 1 个 128K 扇区（扇区 15 = 0x081E0000）。
 *   必须与两处同源：
 *     script.ld            FLASH LENGTH = 1792K
 *     src/init.c           params_sector_map {15, 128K, 0x081E0000}
 *   推导：APP_SIZE_MAX = BOARD_FLASH_SIZE - (BOOTLOADER_RESERVATION_SIZE + APP_RESERVATION_SIZE)
 *                     = 2048K - (128K + 128K) = 1792K
 *
 *   ⚠ 为什么之前是 0（提交 eb4b205550）：
 *   那时从 matek/h743-slim 整段拄来，而参照板没开 FLASH_BASED_PARAMS
 *   （它有 SD 卡），0 是合法的。本板在 be9f428b17 开了片内参数存储，
 *   但只改了 script.ld / board_config.h / init.c，漏了这里 ⇒ bootloader 编译报：
 *     #error "APP_RESERVATION_SIZE must be greater than 0 if FLASH_BASED_PARAMS is defined"
 *   app 不检查该宏，所以一直没暴露。
 */
#if !defined(APP_RESERVATION_SIZE)
#  define APP_RESERVATION_SIZE (128 * 1024)
#endif

#if !defined(BOARD_FIRST_FLASH_SECTOR_TO_ERASE)
#  define BOARD_FIRST_FLASH_SECTOR_TO_ERASE 1
#endif

#if !defined(USB_DATA_ALIGN)
# define USB_DATA_ALIGN
#endif

#ifndef BOOT_DEVICES_SELECTION
#  define BOOT_DEVICES_SELECTION USB0_DEV|SERIAL0_DEV|SERIAL1_DEV
#endif

#ifndef BOOT_DEVICES_FILTER_ONUSB
#  define BOOT_DEVICES_FILTER_ONUSB USB0_DEV|SERIAL0_DEV|SERIAL1_DEV
#endif
