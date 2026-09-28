# rm2027/fmu-h743 — 自研飞控板 PX4 板级支持（草稿）

> 状态：**骨架 / 不可编译** — 2026-09-28
> 分支：`board/fmu-h743`（不污染 `main`）

## ⚠️ 先读这段

**这个目录目前还不能编译，也不应该被编译。**

- `default.px4board.wip` 用 `.wip` 后缀，**故意不被 `boards/*/*.px4board` 的 glob 命中**，
  防止它出现在 `make list_config_targets` / CI 目标枚举里。
- `init/rc.board_sensors` 里的总线号是**占位符**，不是从原理图得到的。
- 还缺 `src/board_config.h`、`nuttx-config/`、`src/spi.cpp`、`src/i2c.cpp` 等，
  没有这些 CMake 根本组不出这个板子。

## 为什么卡住

**GPIO 引脚号无法从原理图 PDF 自动提取**（已实验证伪，见下）。
需要硬件组提供**网表**或**引脚分配表**才能继续。

详细过程、命令、负结果、决策与回退记录，全部在这个目录里：

```text
/mnt/h/AI/shared/projects/competition/rm2027-drone/firmware/fmu-h743/
├── README.md               移植状态总览
├── 资源与引脚映射.md         ✅ 可靠部分 + ❌ 未知部分
└── 开发日志/
    ├── README.md            日志规则（5 问）与索引
    ├── 01-资料盘点.md
    ├── 02-原理图信息提取.md   ★ 含「引脚提取失败」的负结果
    ├── 03-决策记录.md        ★ 含 D-01..D-06 + 2 条返工实录
    └── 04-待确认与风险.md
```

## 本目录文件清单与可信度

| 文件 | 状态 | 可信度 |
|---|---|---|
| `README.md` | ✅ | — |
| `default.px4board.wip` | 🟡 草稿 | **串口 `/dev/ttySx` 全是占位**，待 NuttX defconfig 确定后改 |
| `init/rc.board_sensors` | 🟡 草稿 | 外设归属可信（SPI1/IIC4/IIC2）；**`-b` 总线号是占位** |

### `default.px4board.wip` 里哪些是抄来的、哪些是占位

```text
可信（来自各驱动 Kconfig 核实）：
  CONFIG_DRIVERS_IMU_INVENSENSE_ICM42688P=y
  CONFIG_DRIVERS_MAGNETOMETER_ISENTEK_IST8310=y
  CONFIG_DRIVERS_BAROMETER_GOERTEK_SPA06=y

占位（必须重定）：
  CONFIG_BOARD_SERIAL_TEL1..4    ← 取决于 NuttX defconfig 里的 UART 编号

未包含（因为本板没有）：
  CONFIG_DRIVERS_CDCACM_AUTOSTART   ← 板子无 USB
  CONFIG_SYSTEMCMDS_USB_CONNECTED   ← 同上
  CONFIG_DRIVERS_UAVCAN             ← 暂不需要
```

## 板子事实（依据原理图 V1.0）

```text
主控    STM32H743VIT6   （本板 = FMU）
协处理  STM32F103C8T6   （伴生 MCU，经 USART2 连接；不是 PX4IO）
IMU     ICM-42688-P      SPI1
磁罗盘  IST8310          IIC4
气压计  SPA06-003        IIC2
外置陀螺 H104            CAN1 / USART1
遥控    DBUS             UART5
电机    ESC_DSHOT1..4    TIM1_CH1..CH4
```

## 从参考板抄写时的差异（重要）

参考 `matek/h743-slim`（同为 STM32H743VI）：

```text
1. 本板无 USB      → 去掉 CDCACM 相关配置
2. 本板无 microSD  → 去掉 CONFIG_BOARD_PARAM_FILE="/fs/microsd/params"
3. IMU 只有 ICM-42688P 一颗 → 去掉 mpu6000 / icm20602 / icm42605 兜底
4. 磁罗盘 IST8310（不是 qmc5883l）
5. 气压计 SPA06（不是 dps310）
6. 无 I/O 协处理器 → 不存在 px4io 驱动；MAIN/AUX 之分在本板不适用
```

## ⚠️ 跨端编辑提醒

本目录的文件属于 **Linux 侧执行的工程**，行尾必须 **LF**：

```text
init/rc.board_sensors    NuttX nsh 执行的 shell 脚本 —— CRLF 会直接失败
default.px4board.wip     Kconfig 片段（CRLF 可能被解析出多余字符）
src/*.cpp / *.h          编译源码
```

若在 Windows 侧编辑过任何文件，提交前先跑：

```bash
sed -i 's/\r$//' <file>            # 或 dos2unix <file>
bash /mnt/h/AI/shared/tools/maintenance/check-lineendings.sh
```

约定全文：`H:\AI\shared\tools\maintenance\跨端行尾与脚本约定.md`

---

## 下一步（解锁顺序）

```text
[ ] 1. 拿到网表/引脚表（硬件组）                    ← 卡在这
[ ] 2. src/board_config.h：引脚宏 + PX4_I2C_BUS_* / PX4_SPI_BUS_*
[ ] 3. src/spi.cpp / src/i2c.cpp：总线与外设声明
[ ] 4. nuttx-config/：defconfig + include/board.h + scripts/linker.ld
[ ] 5. src/CMakeLists.txt（px4_add_board）
[ ] 6. git mv default.px4board.wip default.px4board
[ ] 7. 编译 → 烧录 → listener sensor_gyro 看到数据
```
