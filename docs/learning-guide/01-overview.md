# 01 · 仓库全景与架构概览

> 本章目标：对整个 ODrive 仓库建立"地图式"的认知，知道**每个目录干什么、彼此如何协作**。不求细节，只求全局。

---

## 1. 顶层目录结构

```
ODrive-learn/
├── Firmware/            ⭐ 最核心：STM32 固件（C/C++）
├── tools/               ⭐ 上位机：Python 工具（odrivetool 等）
├── GUI/                 可视化：Vue.js + Electron 桌面应用
├── analysis/            Python/MATLAB 分析脚本与电机仿真
├── Arduino/             Arduino 库 + 示例（外部主控对接）
├── docs/                Sphinx RST 文档（官方用户手册）
├── .github/             CI/CD 配置
├── README.md
├── CHANGELOG.md
├── LICENSE.md           MIT
└── Dockerfile           固件构建 Docker 环境
```

**学习重点优先级**：Firmware > tools > analysis > docs > GUI > Arduino

---

## 2. Firmware 固件目录（重中之重）

ODrive 的**灵魂**在这里。ODrive v3.x 使用 **STM32F405**（部分型号 F722） + **DRV8301** 栅极驱动芯片 + **FreeRTOS**。

```
Firmware/
├── MotorControl/        ⭐⭐⭐ 所有控制算法（42 个源文件）
├── Board/
│   └── v3/              STM32CubeMX 生成的板级代码（STM32F4/F7）
│       ├── Inc/         头文件（adc.h / tim.h / gpio.h / main.h / ...）
│       └── Src/         源文件（adc.c / tim.c / gpio.c / ...）
├── Drivers/
│   ├── DRV8301/         DRV8301 栅极驱动芯片驱动
│   └── STM32/           STM32 外设的 C++ 封装（gpio/timer/spi/nvm）
├── communication/       通信协议
│   ├── ascii_protocol.*     ASCII 文本协议（最简单，调试用）
│   ├── interface_usb.*      USB CDC 接口
│   ├── interface_uart.*     UART 接口
│   ├── interface_i2c.*      I2C 接口
│   └── can/
│       ├── can_simple.*     CAN 简易协议（命令/状态）
│       └── odrive_can.*     CAN 总线控制器
├── fibre-cpp/           Fibre 二进制 RPC 协议（C++ 实现）
├── ThirdParty/          第三方库
│   ├── CMSIS/           ARM Cortex-M 标准接口 + DSP 库
│   ├── FreeRTOS/        实时操作系统内核
│   └── STM32F4xx_HAL_Driver/  STM32 HAL 库
├── Tests/               单元测试
├── odrive-interface.yaml  ⭐ 接口定义（机器可读）→ 自动生成多语言代码
├── Tupfile.lua          Tup 构建配置
├── Makefile             Make 桩（代理 Tup）
├── tup.config.default   构建配置默认值
└── .clang-format        代码风格
```

### 2.1 MotorControl/ — 控制算法的全部

| 文件 | 职责 | 第一次看要花多久 |
|------|-----|----------------|
| `main.cpp` | 固件入口、任务创建 | 1 天 |
| `axis.cpp/.hpp` | Axis 类（电机轴对象）、状态机 | 3 天 |
| `motor.cpp/.hpp` | Motor 类、参数校准、力矩→电流 | 3 天 |
| `foc.cpp/.hpp` | FOC 核心算法（Clarke/Park/SVM/PI） | 5 天 |
| `controller.cpp/.hpp` | 位置/速度/力矩级联、抗齿槽 | 5 天 |
| `encoder.cpp/.hpp` | 编码器处理（增量/绝对/霍尔/sincos） | 5 天 |
| `sensorless_estimator.cpp/.hpp` | 反电动势磁链观测器 | 3 天 |
| `acim_estimator.cpp/.hpp` | 异步电机转子磁链观测器 | 2 天 |
| `trapTraj.cpp/.hpp` | 梯形速度规划 | 1 天 |
| `open_loop_controller.cpp/.hpp` | 开环控制（校准、锁相旋转用） | 1 天 |
| `low_level.cpp/.h` | 底层：PWM、ADC、制动电阻、安全 | 5 天 |
| `thermistor.cpp/.hpp` | 温度采样、过温限流 | 半天 |
| `endstop.cpp/.hpp` | 限位开关 | 半天 |
| `mechanical_brake.cpp/.hpp` | 机械刹车 | 半天 |
| `pwm_input.cpp/.hpp` | PWM 输入解码 | 半天 |
| `oscilloscope.cpp/.hpp` | 在线采样（调试波形） | 半天 |
| `utils.cpp/.hpp` | 数学工具、滤波器、斜坡 | 随用随查 |
| `component.hpp` | 组件基类接口（InputPort/OutputPort） | 2 天 |
| `nvm_config.hpp` | 非易失配置（Flash 存储 + CRC） | 1 天 |

### 2.2 组件互联架构（非常关键）

ODrive 采用一种 **InputPort/OutputPort 连接模式**（类似 Simulink 信号线）。每个组件（Encoder、Controller、Motor、FOC）有自己的输入端口和输出端口，在 `axis.cpp` 的 `start_closed_loop_control()` 里被**动态连接**起来。

举例（节选自 `axis.cpp:296-314`）：
```cpp
motor_.torque_setpoint_src_.connect_to(&controller_.torque_output_);
// ↑ Motor 的力矩输入 ← Controller 的力矩输出

motor_.current_control_.phase_src_.connect_to(stator_phase_src);
// ↑ FOC 的相位输入 ← Encoder 或 Sensorless 的相位输出
```

**学习提示**：理解这个"连接"机制是掌握 ODrive 软件架构的钥匙。不懂它，整个代码看起来就是一团乱麻。

---

## 3. tools/ — Python 上位机工具

```
tools/
├── odrive/              Python 包（pip install odrive）
│   ├── __init__.py
│   ├── configuration.py     配置管理
│   ├── dfu.py               DFU 固件升级
│   ├── dfuse/               DFU 协议实现
│   ├── enums.py             从 odrive-interface.yaml 自动生成的枚举
│   ├── pyfibre/             Python 版 Fibre 协议
│   ├── shell.py             REPL 交互模式（odrivetool shell 的入口）
│   ├── tests/
│   ├── utils.py             ⭐ 常用工具函数（电机校准、波形、诊断）
│   └── version.py
├── fibre-tools/         Fibre 协议工具
├── motion_planning/     轨迹规划工具
├── odrivetool           ⭐ 命令行入口（Python 脚本）
├── create_can_dbc.py    生成 CAN DBC 数据库文件
├── setup.py             pip 安装配置
└── requirements.txt
```

**典型使用**：
```bash
$ odrivetool              # 进入交互式 REPL，自动发现 USB 设备
In [1]: odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
In [2]: odrv0.axis0.controller.input_pos = 10  # 让电机转到 10 圈位置
```

Python 工具与固件通过 **Fibre 协议**（USB/UART/CAN）自动镜像固件中所有暴露的属性和方法——这是 `odrive-interface.yaml` 的魔法。

---

## 4. GUI/ — Vue.js + Electron 桌面应用

```
GUI/
├── src/                 Vue 组件、Vuex 状态管理
├── public/              静态资源
├── server/              本地后端服务
├── fibre-js/            JavaScript Fibre 协议实现
├── scripts/             构建脚本
├── package.json         Node 依赖
├── vue.config.js
└── babel.config.js
```

对学习者而言 GUI 不是重点，但它展示了：
- Fibre 协议如何跨语言（C++ / Python / JS）工作
- 如何在上位机实时绘制控制波形
- Electron 如何把 Web UI 包装成桌面应用

---

## 5. analysis/ — 仿真与分析脚本

```
analysis/
├── Simulation/
│   ├── MotorSim.py              ⭐ Python BLDC 仿真（很适合入门 + 算法研究）
│   └── TranslationalMass.py     直线电机模型
├── cogging_torque/
│   └── cogging_harmonics.py     齿槽力矩谐波分析
├── motor_analysis/
│   ├── ac_induction_motor.py    异步电机分析
│   ├── 350kvTP.PNG              参考图表
│   └── VelliPlot.m              MATLAB 脚本
├── numeric_path_opt/
│   ├── Main.m                   MATLAB 轨迹优化
│   └── predictionmatrices.m
├── filterpoles.py               滤波器极点分析
└── thermistors.py               热敏电阻曲线拟合
```

对**算法研究**而言这个目录是金矿——你可以：
- 先在 Python 里验证你的新算法（如 SMO 观测器）
- 用 MATLAB 做最优控制研究
- 用仿真代替真机减少实验成本

---

## 6. docs/ — 官方 Sphinx 文档

30+ 个 `.rst` 文件，涵盖：

| 类别 | 文件 |
|------|------|
| 入门 | `getting-started.rst` / `specifications.rst` |
| 配置 | `control-modes.rst` / `encoders.rst` / `pinout.rst` |
| 协议 | `ascii-protocol.rst` / `native-protocol.rst` / `can-protocol.rst` / `can-guide.rst` |
| 硬件 | `thermistors.rst` / `hoverboard.rst` / `analog-input.rst` |
| I/O | `rc-pwm.rst` / `step-direction.rst` / `endstops.rst` |
| 开发 | `developer-guide.rst` / `configuring-vscode.rst` / `configuring-eclipse.rst` |
| 通信 | `uart.rst` / `usb.rst` |
| 其他 | `control.rst` / `ground-loops.rst` / `troubleshooting.rst` / `migration.rst` |

**推荐第一次阅读**：`getting-started.rst` → `control-modes.rst` → `control.rst` → `developer-guide.rst`

---

## 7. 构建系统

### 7.1 固件构建：Tup

- 主配置：`Firmware/Tupfile.lua` + `Firmware/tup.config.default`
- 编译器：ARM GCC 交叉编译
- 支持板型：v3.2 / v3.3 / v3.4-24V / v3.4-48V / v3.5-24V / v3.5-48V / v3.6-24V / v3.6-56V
- `Makefile` 只是桩，实际调用 Tup

典型命令（在 Firmware 目录下）：
```bash
make PREVENT_USB=1 CONFIG_BOARD_VERSION=v3.6-56V    # 编译
make flash                                          # 烧录（OpenOCD）
```

### 7.2 接口代码自动生成

`Firmware/odrive-interface.yaml` 是**"真理之源"**。它用 YAML 描述所有对外暴露的对象、属性、方法、枚举。构建时通过 **Jinja2 模板**生成：
- `autogen/interfaces.hpp` — 固件侧 C++ 接口
- `tools/odrive/enums.py` — Python 枚举
- `ODriveArduino` 的头文件
- CAN DBC 文件

**含义**：想加一个新的参数暴露到 Python？只需要改 YAML，编译系统会自动帮你生成 C++ 和 Python 胶水代码。

### 7.3 Python 工具构建

`tools/setup.py` — 标准的 setuptools，支持 `pip install -e .` 做本地开发。

### 7.4 GUI 构建

`GUI/package.json` 定义的 npm 脚本：
- `serve`：本地 Vue 开发服务器
- `build`：Web 打包
- `electron:build`：桌面应用打包

---

## 8. 关键技术栈一览

| 层 | 技术 |
|-----|------|
| MCU | STM32F405 (Cortex-M4F) / STM32F722 (Cortex-M7) |
| RTOS | FreeRTOS |
| 语言 | C++17（固件）/ Python 3（工具）/ JavaScript + Vue 3（GUI）|
| 数学库 | CMSIS DSP（ARM 优化的定点/浮点数学） |
| 通信 | USB CDC / UART / CAN 2.0B / I2C / SPI（编码器） |
| 协议 | Fibre RPC（二进制）/ ASCII（文本）/ CANSimple |
| 控制 | FOC（Field-Oriented Control）/ PI 级联 / 梯形轨迹 |
| 状态估计 | PLL / 非线性磁链观测器（Lee-Ortega）/ACIM 转子磁链 |
| 构建 | Tup（固件）/ setuptools（Python）/ Vite + npm（GUI） |

---

## 9. 仓库导航小技巧

### 快速定位功能
- 搜索 `odrive-interface.yaml` → 找所有对外暴露的属性
- 搜索 `AXIS_STATE_` → 找所有状态机状态
- 搜索 `CRITICAL_SECTION` → 找中断临界区
- 搜索 `OutputPort<` / `InputPort<` → 找组件连接点

### 追踪错误
- 所有错误定义都在 `autogen/interfaces.hpp`（由 YAML 生成）
- 查错误位掩码：搜索 `ERROR_SPINOUT_DETECTED` 之类的符号

### 理解调用链
- PWM 中断 → `motor.current_meas_cb()` → `current_control_.on_measurement()` → `current_control_.get_alpha_beta_output()` → SVM → PWM timings
- Axis 线程循环（1 kHz）→ 各组件 `update()`：encoder → controller → motor → acim_estimator → ...

---

## 10. 本章动手实验

**实验 0-1：克隆仓库并浏览目录**
```bash
git clone https://github.com/odriverobotics/ODrive.git
cd ODrive
tree -L 2 -d        # 目录树
wc -l Firmware/MotorControl/*.cpp | sort -n   # 按行数排序看规模
```

**实验 0-2：安装 Python 工具**
```bash
cd tools
pip install -e .
odrivetool --help
```

**实验 0-3：阅读 odrive-interface.yaml**
- 打开 `Firmware/odrive-interface.yaml`
- 找到 `Axis` 对象的定义
- 数一数它暴露了多少个属性、枚举、方法
- 对比 Python 侧 `tools/odrive/enums.py` 中生成的枚举

### 检验标志

读完本章，你应该能回答：
- [ ] ODrive 一共有哪些主要目录，每个是做什么的？
- [ ] 固件是用什么构建的？支持几种板型？
- [ ] 上位机（Python）是怎么"认识"固件里有哪些属性的？
- [ ] 一个 Axis 对象里有哪些子组件？它们是怎么连起来的？
- [ ] 从 USB 命令到电机转动，数据流要经过哪些层？

---

下一步 → [02-phase1-foundation.md：基础理论与项目概览](02-phase1-foundation.md)

上一步 ← [README.md：指南首页](README.md)
