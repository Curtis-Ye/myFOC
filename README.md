# myFOC

基于 **STM32F103CB** 的 FOC（磁场定向控制）无刷电机驱动固件。

控制算法用 **Q15 定点**实现（1024 点正弦表 + 线性插值），Cortex-M3 无 FPU，定点化可避免软件浮点在中断里的开销。

---

## 硬件

| 项目 | 型号 / 参数 |
|---|---|
| MCU | STM32F103CBT6，LQFP48，128 KB Flash / 20 KB RAM，Cortex-M3 |
| 系统时钟 | HSE 8 MHz 晶振 → PLL ×9 → **72 MHz** |
| 功率驱动 | **DRV8313** 三相驱动（数据手册见 [`drv8313.pdf`](drv8313.pdf)） |
| 位置反馈 | **AS5600** 磁性编码器，I2C1，12 位角度 |
| 电机 | **7 极对**（见 `FOC.h` 的 `POLE_PAIRS`） |

## 引脚分配

| 引脚 | 功能 | 说明 |
|---|---|---|
| PA8 | TIM1_CH1 | PWM — A 相 |
| PA9 | TIM1_CH2 | PWM — B 相 |
| PA10 | TIM1_CH3 | PWM — C 相 |
| PA1 | GPIO 输出 | 推挽输出，上电默认低电平；`.ioc` 中未命名 |
| PA15 | GPIO 输出 | 推挽输出，上电默认低电平；`.ioc` 中未命名 |
| PA2 / PA3 | USART2 TX / RX | 115200 8N1 |
| PA13 / PA14 | SWDIO / SWCLK | SWD 调试口 |
| PB6 / PB7 | I2C1 SCL / SDA | 100 kHz，接 AS5600 |
| PB8 / PB9 | CAN_RX / CAN_TX | 750 kbit/s |
| PD0 / PD1 | OSC_IN / OSC_OUT | 8 MHz 外部晶振 |

> PA1 / PA15 在 `gpio.c` 里被初始化为低电平，`.ioc` 中没有标注用途。按 DRV8313 的典型方案通常是使能/休眠控制，请以实际原理图为准。

## 定时器与外设配置

| 外设 | 配置 | 结果 |
|---|---|---|
| **TIM1** | PSC = 0，ARR = 1799，PWM 模式 1，CH1/CH2/CH3 | **40 kHz** 三相 PWM，无死区、未启用刹车 |
| **TIM3** | PSC = 71，ARR = 99 | **10 kHz** 定时中断，触发 FOC 更新 |
| I2C1 | 标准模式 | 100 kHz |
| USART2 | 异步 | 115200 |
| CAN | PSC = 16，SJW = 1TQ，BS1 = 1TQ，BS2 = 1TQ | 750 kbit/s |

TIM1 属于高级控制定时器，**必须启动 PWM 输出才会置位 MOE**，否则引脚保持无效电平 —— 这一点在 `main.c` 里有对应注释。

## 目录结构

```
myFOC.ioc            STM32CubeMX 工程（所有外设配置的源头，改配置请从这里改）
sin-table.txt        Q15 正弦表生成结果，粘贴到 FOC.c 的 sin_tab[]
myFOC.xmind          设计思路思维导图
drv8313.pdf          DRV8313 数据手册
code/
├── Core/            业务代码
│   ├── Inc/         FOC.h  PID.h  AS5600.h  + CubeMX 生成的头文件
│   └── Src/         FOC.c  PID.c  AS5600.c  main.c  + CubeMX 生成的外设初始化
├── Drivers/         STM32F1xx HAL 驱动与 CMSIS（厂商代码，一般不需要改）
└── MDK-ARM/         Keil MDK 工程
    ├── myFOC.uvprojx         Keil 工程文件
    ├── myFOC.uvoptx          调试器 / 断点 / 窗口布局
    ├── startup_stm32f103xb.s 启动文件
    └── DebugConfig/          调试器配置
```

### 控制算法（`FOC.c`）

1. `angle_reverse()` — 机械/电角度（度）→ 0~65535 的定点角度
2. `park_inverse()` — 反 Park 变换，(Ud, Uq) → (Uα, Uβ)，用 `sin_tab` 查表 + 线性插值
3. `clark_inverse()` — 反 Clark 变换，(Uα, Uβ) → (Ua, Ub, Uc)
4. 占空比 = `(U + 32768) * ARR >> 16`，并限幅到 `[0, ARR]` 后写入 TIM1 的 CCR

占空比满量程直接取 `htim1.Init.Period`，避免和 `tim.c` 两处硬编码不一致。

## 编译与烧录

**前置条件**

1. **Keil MDK-ARM 5（µVision）**。工程由 CubeMX 按 `MDK-ARM V5.32` 生成。
2. **`Keil.STM32F1xx_DFP.2.4.1` 器件包**（工程里写死了这个版本）。没装的话打开工程会提示缺 Device，在 Pack Installer 里搜索 `STM32F1` 安装即可。
3. **ARM Compiler 5**。工程里 `uAC6 = 0`，即使用 AC5（armcc）。**Keil MDK 5.37 以后默认不再附带 AC5**，如果编译报找不到编译器，需要单独安装 Arm Compiler 5，或在 `Project → Options for Target → Target` 里改成 `Use default compiler version 6`（切 AC6 可能需要处理少量编译选项差异）。

**步骤**

```
1. 打开 code/MDK-ARM/myFOC.uvprojx
2. Build (F7)
3. Download (F8)，调试器默认配置在 DebugConfig/myFOC_STM32F103CB_1.0.0.dbgconf
```

`code/MDK-ARM/myFOC/`（`.o` / `.axf` / `.hex` / `.map` 等编译产物）、`code/MDK-ARM/RTE/` 和分散加载文件 `myFOC.sct` 都由 Keil 首次编译时自动生成，已加入 `.gitignore`，不需要也不应该提交。

## 当前状态

目前主循环跑的是**开环 V/F**：

```
TIM3 10 kHz 中断 → FOC_OpenLoop_Update()
    ├─ 电角度以 2 Hz 递增（open_loop_elec_freq）
    └─ FOC_control(电角度, Uq = 4000)   // Uq 固定为 Q15 的 4000
```

已经写好但**尚未接入控制环**的部分：

- `AS5600.c` — 编码器读取（`AS5600_ReadRawAngle()` / `AS5600_ReadAngle()`）
- `PID.c` — 带积分限幅和输出限幅的 PID
- `can.c` / `usart.c` — CAN 与串口外设初始化

待办：

- [ ] 把 AS5600 角度 × 极对数得到电角度，替换开环角度，实现闭环
- [ ] 用 `PID` 模块搭速度环 / 电流环
- [ ] 电流采样：当前 `.ioc` **没有配置 ADC**，做电流环前需要先加采样电路和外设配置
- [ ] 明确 PA1 / PA15 的用途并补进本文档
- [ ] CAN / USART2 的通信协议

> 改外设配置请用 STM32CubeMX 打开根目录的 `myFOC.ioc` 重新生成，不要手改 `code/Core` 里 CubeMX 托管的部分（`/* USER CODE BEGIN */` 与 `/* USER CODE END */` 之间的代码会被保留）。

## 许可证

本项目采用 **MIT License**，见 [`LICENSE`](LICENSE)。

`code/Drivers/` 下是 STMicroelectronics 与 ARM 的第三方代码，各自遵循其目录内的 `LICENSE.txt`：

- `STM32F1xx_HAL_Driver/` — ST 的 BSD-3-Clause
- `CMSIS/` — ARM 的 Apache-2.0
