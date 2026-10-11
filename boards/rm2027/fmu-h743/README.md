# rm2027 / fmu-h743 —— 板级说明

> ⚠️ **本文件是新写的，不是从参照板或旧版沿用。**
> 旧版（`dev` 分支上的 118 行）**已失效**：它仍在描述"骨架 / 尚不可编译"，
> 而那之后板子早已编出固件、四个传感器也已上板验证。
> **本文只写"当前成立的事实 + 出处"。**

---

## 1. 这块板是什么

| 项 | 值 | 依据 |
|---|---|---|
| 主控 | **STM32H743VIT6**（LQFP100，cortex-m7） | 数据手册 Figure 5 与网表逐脚核对（12 组全命中） |
| 晶振 | **HSE 25 MHz**（X1 / OSC7_IN） | 原理图 V1.0 直读 |
| 协处理 | STM32F103C8T6（经 USART2 连接）—— **不是 PX4IO** | 原理图；本板 `CONFIG_BOARD_IO` 未设 |
| 板名 | vendor=`rm2027` · model=`fmu-h743` · label=`default` | 目录结构即构建目标名（`Makefile:237` → `px4_config.cmake:74-83`） |
| 构建目标 | `make rm2027_fmu-h743_default`（app）· `make rm2027_fmu-h743_bootloader` | |

---

## 2. 与参照板（`boards/matek/h743-slim`）的硬件差异

**这是本板与"照抄来的骨架"真正的分界。**

| 部件 | 参照板 | **本板** | 后果 |
|---|---|---|---|
| HSE 晶振 | 8 MHz | **25 MHz** | ⇒ PLL 必须重算（见 §3.1） |
| USB | 有 | **无** | ⇒ bootloader 只走串口；无 USB console |
| microSD | 有 | **无** | ⇒ 参数只能存片内 flash（见 §3.4） |
| 板载 IMU | ICM-42688P | ★ **华轩阳 HXY「ICM-42688P」** | **是兼容件**：`WHO_AM_I(0x01)=0x6A`，与 Bosch ICM-42688P **不是同一颗** |
| 外置 IMU | — | **H104（HiPNUC HI04M0）** on USART1 | 自研驱动 |
| 磁罗盘 | — | **iSentek IST8310** on I2C4，7bit `0x0C` | 手册 §6.1.1 地址表 |
| 气压计 | DPS310 | **Goertek SPA06** on I2C2，7bit `0x77` | 驱动缺省 `0x76` ⇒ 必须显式 `-a` |
| 遥控 | — | **DBUS** on UART5 | |
| 电机输出 | 12 路 | **4 路 DShot**：TIM1_CH2/CH3 + TIM8_CH3/CH4 | ★ 跨两个定时器（见 §3.2） |
| LED | — | 2 个逻辑 LED（映射到原理图上悬空的脚） | |
| I2C4 | 无 | **本板新增**（PD12/PD13） | ★ 曾与定时器表冲突（见 §3.2） |

---

## 3. 已落地的关键判决（都有出处）

### 3.1 时钟：HSE 8 → 25 MHz，但**输出频率不变**

**约束不是"照抄分频比"，而是"输出频率与参照板一致"** —— 否则 SPI/ADC 时钟会跟着偏，
而现象是"能跑但时序不对"，极难查。

```
PLL1: M 1→5, N 120→192  ⇒ VCO 960 MHz 不变 ⇒ SYSCLK 480 MHz 不变
PLL2: M 2→5, N 48→96,  P/Q/R 2→5 ⇒ 96/96/96 MHz 不变
PLL3: M 2→5, N 48→96,  P 2→5, Q 4→10, R 2→5 ⇒ 96/48/96 MHz 不变
```
依据：`nuttx-config/include/board.h` 逐条注释；晶振频率来自原理图直读。

### 3.2 ★ 4 路 DShot 引脚 —— 这里**错过一次**，务必别照抄别的板

```
路1 PE11 = TIM1_CH2 → 座 U13      路2 PE13 = TIM1_CH3 → 座 U11
路3 PC8  = TIM8_CH3 → 座 U14      路4 PC9  = TIM8_CH4 → 座 U15
```
- 依据：原理图网表（`ESC_DSHOT1..4` 每条网络是「MCU + 插座」两成员）+ LQFP100 数据手册
- ★ **旧版是从 `boards/3dr/ctrl-n1` 整段照抄的**，把路 1/4 指到 `PA8`/`PE14`
  —— 而这两个脚在本板是**单点网络（只有 MCU 一个焊盘，没接任何东西）**，
  导致那两个座的电调**永远收不到 DShot**。整段照抄时前两路"碰巧对"，
  把后两路的错藏了很久。
- ★ 连带：参照板的 12 路表会声明 `PD12–PD15`，而 **`PD12/PD13` 正是磁罗盘的 I2C4**
  ⇒ 会把磁罗盘抢死。**这就是本板必须有自己那张表的原因。**

### 3.3 串口分配（8 个全开）

| 外设 | 节点 | 用途 |
|---|---|---|
| USART1 | `/dev/ttyS0` | H104 外置陀螺 |
| USART2 | `/dev/ttyS1` | 协处理 F103 |
| USART3 | `/dev/ttyS2` | **TEL1 → MAVLink** |
| UART4 / USART6 / UART7 / UART8 | ttyS3 / ttyS5 / ttyS6 / ttyS7 | 4 路 ESC 遥测（预留） |
| UART5 | `/dev/ttyS4` | DBUS 遥控 |

⇒ `CONFIG_BOARD_SERIAL_TEL1="/dev/ttyS2"`（与 USART3 对齐）。

### 3.4 参数存储：片内 flash 扇区 15

本板无 SD、无 USB ⇒ 参数只能存片内 flash：**扇区 15 = `0x081E0000`，128 K**。
**三处必须同源**：`script.ld` 的 `FLASH LENGTH = 1792K` + `board_config.h` 的
`FLASH_BASED_PARAMS` + `init.c` 的 `params_sector_map[]`。
**缺任何一处**：要么链接器把代码放进参数区（写参数打烂代码），要么参数一断电就丢。

### 3.5 上传通道

无 USB ⇒ bootloader 关掉 USB（`hw_config.h` 的 `INTERFACE_USB 0`）只走串口。

---

## 4. 既有约束（改之前先看这里）

| 约束 | 值 / 规则 | 出处 |
|---|---|---|
| **SPI 设备槽位上限** | **6**（不可覆盖） | `platforms/common/include/px4_platform_common/spi.h:54` |
| **`rc.board_*` 文件名白名单** | 只认 5 个：`early / defaults / sensors / extras / mavlink` | `ROMFS/CMakeLists.txt:196-202` |
| **ROMFS 脚本非注释行** | **不能有非 ASCII**（否则构建中断） | `Tools/px_romfs_pruner.py:121` |
| **串口节点编号** | `ttyS<k>` = 从 USART1 数起**第 k+1 个被使能的串口** | `stm32_serial.c:3934-3955`；`SERIAL_DISABLE_REORDERING=y` ⇒ 不受 console 影响，但**关掉任一个口会让后续节点整体前移** |
| **参数区** | 扇区 15 `0x081E0000`，与 `script.ld` 的 1792K 同源 | §3.4 |
| **板级 `src/` 目录名** | 必须叫 `src` | `cmake/kconfig.cmake:454-455` |
| **新驱动目录命名的 7 种候选** | 目录路径必须能被 `kconfig.cmake:197-213` 的 7 种展开之一命中 | 否则 configure 直接 `FATAL_ERROR` |
| **`CONFIG_SYSTEMCMDS_*`** | **不猜路径**，直拼 `systemcmds/<小写名>` | `kconfig.cmake:258` |
| **`CONFIG_DRIVERS_*`** | 走 7 种候选路径猜测 | `kconfig.cmake:197-213` |

---

## 5. 哪些是抄的、哪些是算的

| 类别 | 文件 | 说明 |
|---|---|---|
| **纯模板（逐字节相同）** | `src/led.c` | 照参照板复制，未改一字 |
| **骨架照抄、内容重算** | `src/*` 其余 · `nuttx-config/*` | 结构照参照板；**引脚 / 时钟 / 总线 / 外设全部按本板原理图重算** |
| **本板自研** | `src/drivers/imu/hxy/hxy42688p/` · `src/drivers/imu/hipnuc_h104/` · `src/systemcmds/h104cap/` | 上游没有对应实现 |
| **有意不含** | `src/drivers/imu/bosch/bmi270/`（兼容件补丁，682 行） | 已推翻的路线，本分支有意不加 |
| **有意不含** | `extras/*_bootloader.bin` | 见 §6 |

---

## 6. 有意不含的东西（不是遗漏）

| 项 | 状态 | 原因 |
|---|---|---|
| `src/drivers/imu/bosch/bmi270/*`（682 行） | **有意不含** | 死路产物（BMI270 兼容件路线已推翻）；`default.px4board` 未选中它 |
| `extras/rm2027_fmu-h743_bootloader.bin`（38,148 B） | **当前不含** | 待决 —— 见下方 |

**关于 `extras/*`:**
它是**预编译的 bootloader 镜像**，供 ROMFS 的固件升级路径使用
（`ROMFS/CMakeLists.txt:264` 用 `${VENDOR}_${MODEL}_bootloader.bin` 匹配）。
**它不是源码、不参与 app 编译** ⇒ 少了它不影响本板启动与飞行，
只影响"从 ROMFS 里升级 bootloader"这条路。

**⇒ 两条路，二选一（当前未决，不许留成"意外缺失"）：**
1. **补回来** —— 保留 bootloader 升级路径（代价：仓库里多一个 38 KB 的二进制产物）
2. **明写"有意不含"** —— 若决定不保留该路径，把这一行改成"有意不含 + 原因"

---

## 7. 上板验证状态（2026-10-11）

| 项 | 实测 | 判据 |
|---|---|---|
| 磁罗盘 IST8310 | `Type=0x06` / `I2C:4 (0x0C)` / \|B\|=0.552 G | 身份 + 地址 + 量级 |
| 板载 HXY42688P | `z = −9.74`，静止陀螺 ≈ 0 | 符号 + 标度 |
| 外置 H104 | `z = −9.85`；**99.6 Hz**；`crc bad=0` `len bad=0` | 帧率 + 通信质量 |
| 气压计 SPA06 | `spa06 #0 on I2C bus 2 address 0x77` | |
| EKF2 姿态 | Roll −0.3° / Pitch 0.8° | |
| **未做** | **电机 / 电调联调**；**原理图↔实物引脚通断复核** | 见 §8 |

---

## 8. 还没做 / 待决

| # | 事项 | 说明 |
|---|---|---|
| 1 | **电机 / 电调联调** | 需先设 `PWM_MAIN_TIM0 = -4` 且 `PWM_MAIN_TIM1 = -4`（**DShot300**，一帧 16 bit / 53.3 µs；`dshot.c:308`），`PWM_MAIN_FUNC1..4 = 101..104`；**两组 TIM 必须一致** |
| 2 | **原理图 ↔ 实物引脚通断复核**（6 条读数） | 属"接电机之前"的物理双证。**当前判定：可延后** —— 引脚层面本板没有"图纸与实物不符"的实例（型号层面有过，但那是 identity 问题）。**在接电机前做即可** |
| 3 | `PWM_MAIN_FUNC` 的默认值 | 默认全 **0 = Disabled** ⇒ 不驱动任何引脚。**在联调前不要随手设它** |

---

## 9. 环境注意

- **构建的 Python 解释器**：PX4 构建可能挑到用户环境里的版本化 python（缺依赖）。
  用 `-DPYTHON_EXECUTABLE=/usr/bin/python3` 显式钉住；自检脚本：`work/check-px4-python.sh`
- 本仓库是**浅克隆**：`2c47a16f` 是 shallow root（`git rev-parse 2c47a16f^` 失败）
  ⇒ 比它更早的上游历史本地不存在
- **跨端**：`init/rc.board_*` 是 Linux/NuttX 侧 shell 脚本，**行尾必须 LF**
