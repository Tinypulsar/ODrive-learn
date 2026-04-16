# 阶段一 · 基础理论与项目概览（第 1-2 周）

> **目标**：建立电机控制的理论基础，了解 ODrive 系统运行全貌。**不碰复杂算法**，重在打地基。

---

## 1.1 你需要掌握的基础理论

### 电机相关

#### BLDC / PMSM 电机原理
- **BLDC**（Brushless DC，无刷直流）：其实是一种 PMSM，只是梯形反电动势、用方波驱动
- **PMSM**（Permanent Magnet Synchronous Motor，永磁同步电机）：正弦反电动势、正弦电流驱动，ODrive 主要控制的对象
- **关键参数**：
  - **极对数（pole pairs）**：转子一圈内经历的电周期数。机械角度 × 极对数 = 电角度
  - **相电阻 R**（Ω）、**相电感 L**（H）
  - **反电动势常数 Kv（rpm/V）** 或 **力矩常数 Kt（Nm/A）**，两者有关系 `Kt ≈ 8.27 / Kv`
  - **磁链 λ**（Wb）：永磁体在定子绕组中产生的磁通链

#### 三相逆变器
- 6 个 MOSFET（上下桥臂 × 3 相）组成桥式电路
- 每相输出三种电平：+Vdc / 0 / -Vdc
- 通过 PWM 控制每个开关的导通比例，等效合成任意幅值/相位的电压矢量
- **死区时间（dead time）**：防止上下桥臂直通造成短路

#### 电机控制的本质
> 通过调节三相电压/电流的**幅值、频率、相位**，让定子磁场以期望的方向和大小旋转，拖动永磁转子产生力矩。

### 控制理论

#### PID 控制
- P：比例，对当前误差作用
- I：积分，消除稳态误差
- D：微分，抑制超调（ODrive 主要用 PI，不用 D）
- **级联 PID**：位置环的输出作为速度环的输入，速度环的输出作为力矩/电流环的输入

#### 参考坐标系
ODrive 的所有控制算法都建立在**坐标变换**之上，这是 FOC 的核心：

| 坐标系 | 含义 | 特点 |
|------|------|------|
| **abc（三相）** | 三相电压/电流（120° 相位差）| 实际物理量 |
| **αβ（两相静止）** | Clarke 变换后 | 静止坐标系，两轴正交 |
| **dq（两相旋转）** | Park 变换后（跟随转子旋转）| 旋转坐标系，DC 量易控制 |

**关键洞察**：在 dq 坐标系下，**Iq 直接对应力矩，Id 通常为 0**（或用于弱磁控制）。这就把复杂的交流电机控制化简成了类似直流电机的控制——这是 FOC 的威力所在。

### 嵌入式系统

- **STM32F4 架构**：Cortex-M4F、168 MHz、硬件 FPU、DSP 指令
- **关键外设**：
  - **TIM1/TIM8**：高级定时器，3 对互补 PWM 通道，用于驱动逆变器
  - **ADC1/ADC2/ADC3**：12 位 ADC，PWM 同步采样电流
  - **SPI**：编码器、栅极驱动 DRV8301 通信
  - **CAN**：CAN 2.0B
- **FreeRTOS**：
  - 任务（Task）：Axis 状态机各有一个任务
  - 信号量 / 队列 / 消息
  - 中断和任务的交互
- **实时性要求**：PWM 中断是硬实时（~8 kHz，即 125 μs 周期），所有 FOC 计算必须在中断内完成

---

## 1.2 推荐学习资源

**书籍**（选一本）：
- 《Permanent Magnet Synchronous and Brushless DC Motor Drives》 — R. Krishnan
- 《Electric Motor Drives: Modeling, Analysis, and Control》 — R. Krishnan（中文版《电动机驱动——建模、分析与控制》）
- 《现代永磁同步电机控制原理及 MATLAB 仿真》 — 袁雷

**在线课程 / 视频**：
- Texas Instruments 的 InstaSPIN / MotorWare 培训视频（免费）
- 诸如 MATLAB Motor Control Blockset 的官方教程

**博客 / 文章**：
- ODrive 官方文档 [`docs/control.rst`](../control.rst)
- Tim Morizur's "FOC 从入门到精通" 系列
- Ben Katz（MIT mini-cheetah 作者）关于 actuator design 的分享

---

## 1.3 阅读顺序（按本阶段优先级）

### 第 1 天：项目介绍

1. `README.md` 根目录
2. `CHANGELOG.md` — 扫一眼近几个版本的变更，了解项目的活跃方向
3. `docs/getting-started.rst` — 官方入门

### 第 2 天：控制概念

4. `docs/control-modes.rst` — ⭐ 理解 ODrive 支持的控制模式（POSITION / VELOCITY / TORQUE）和输入模式（PASSTHROUGH / TRAP_TRAJ / POS_FILTER / ...）
5. `docs/control.rst` — 级联 PID 架构说明
6. `docs/encoders.rst` — 支持哪些编码器

### 第 3 天：接口定义

7. `Firmware/odrive-interface.yaml` — ⭐⭐ **极其重要**，看一遍心里就有 Axis / Motor / Encoder / Controller 各自包含哪些字段的完整地图。这份文件是全栈代码生成的"单一真理源"。

### 第 4-5 天：入口与启动流程

8. `Firmware/MotorControl/odrive_main.h` — 全局头
9. `Firmware/MotorControl/main.cpp` — `main()` 做了什么
10. `Firmware/Board/v3/board.cpp` — 两个 Axis（M0、M1）如何被实例化、硬件资源如何分配

### 第 6-7 天：开发环境搭建

11. `docs/developer-guide.rst`
12. `docs/configuring-vscode.rst`
13. 尝试搭建编译环境（Windows：WSL 推荐；Linux / macOS：原生）
14. 尝试编译一次固件（不要求烧录）
15. 安装 Python 工具：`pip install -e tools/`

---

## 1.4 深度阅读：`main.cpp` 入口剖析

打开 `Firmware/MotorControl/main.cpp`，关注以下结构：

### 全局对象
```cpp
ODrive odrv{};                          // 全局 ODrive 实例
ConfigManager config_manager;           // 配置管理
StatusLedController status_led_controller;
```

### `rtos_main()` / `default_task`（FreeRTOS 主任务）
大致做这些事：

1. **硬件初始化**：调用 `board.cpp` 中的 `system_init()`，配置 GPIO/TIM/ADC/SPI/UART/USB/CAN
2. **加载配置**：从 Flash 读取之前保存的 `config`（`load_configuration()`）
3. **组件初始化**：调用每个 Encoder / Motor / Controller 的 `apply_config()` / `setup()`
4. **启动 Axis 任务**：`axes[i].start_thread()` 创建两个独立的 FreeRTOS 任务
5. **启动通信任务**：USB / UART / CAN 各自的任务
6. **进入无限循环**：处理低优先级事务

### 关键技术点
- **堆（heap）放在 CCMRAM**：`__attribute__((section(".ccmram"))) uint8_t ucHeap[]` —— 利用 STM32F405 的 64KB 核心耦合内存提升性能
- **信号量/消息队列**：`sem_usb_irq`、`uart_event_queue`、`usb_event_queue`、`sem_can` —— 用于中断到任务的同步

### 检查点
- [ ] 你能否指出 Axis 任务从哪里被创建？
- [ ] 你能否指出中断 vs 任务分别做什么？
- [ ] 你能否在 `board.cpp` 中找到 `axes[]` 数组是怎么初始化的？

---

## 1.5 深度阅读：`board.cpp` 硬件实例化

打开 `Firmware/Board/v3/board.cpp`，关注：

### 全局实例声明
```cpp
Drv8301 m0_gate_driver{&hspi3, ... };        // M0 的栅极驱动器
Drv8301 m1_gate_driver{&hspi3, ... };        // M1 的栅极驱动器
Encoder encoders[AXIS_COUNT] = { ... };      // 两个编码器
Motor motors[AXIS_COUNT] = { ... };          // 两个电机
Axis axes[AXIS_COUNT] = { ... };             // 两个轴（组合所有）
```

每个 `Axis` 构造时会接收该轴的所有子组件引用（Encoder、Motor、Controller 等），把它们"组合"成一个完整的控制单元（见 `axis.cpp:10-43` 的构造函数）。

### 硬件资源分配（v3.6 为例）
| 资源 | M0 | M1 |
|------|-----|-----|
| PWM 定时器 | TIM1 | TIM8 |
| 编码器定时器 | TIM3 | TIM4 |
| 电流采样 ADC | ADC2 | ADC3 |
| 栅极驱动 SPI | SPI3 | SPI3（共用，通过仲裁）|

### 关键函数
- `board_init()`：顶层硬件初始化
- `start_adc_pwm()`：启动 PWM 和 ADC 同步采样
- `vbus_adc_cb()`：DC 母线电压采样回调

---

## 1.6 阶段一动手实验

**实验 1-1：阅读 YAML**
打开 `Firmware/odrive-interface.yaml`，找到：
- `ODrive` 对象暴露了哪些顶层属性？
- `Axis` 有哪些子对象？
- `AxisState` 枚举有哪些取值？对应什么状态？

**实验 1-2：搭建编译环境**（不要求实际烧录）
```bash
# 安装 ARM GCC 和 Tup
# Ubuntu:
sudo apt install gcc-arm-none-eabi
# 安装 tup: https://gittup.org/tup/

cd Firmware
tup init
make CONFIG_BOARD_VERSION=v3.6-56V PREVENT_USB=1
```

如果出错，记录错误信息 — 这本身就是学习。

**实验 1-3：Python 工具探索**
```bash
cd tools
pip install -e .
odrivetool            # 假装有硬件，看它如何报"no devices found"
odrivetool --help
```

**实验 1-4：画一张启动流程图**
用任意画图工具（draw.io / PlantUML / 笔纸），画出从上电到进入 `AXIS_STATE_IDLE` 经过哪些步骤。参考 `main.cpp` 和 `axis.cpp::run_state_machine_loop()` 的第一个 `case`。

---

## 1.7 理论自测

先尝试回答这些问题，不会再回去看：

1. 为什么 PMSM 控制需要知道转子位置？不知道行不行？
2. 三相逆变器 6 个 MOSFET 同时打开会发生什么？
3. 为什么 ODrive 说 "Kt (Nm/A) ≈ 8.27 / Kv (rpm/V)"？推导一下
4. Clarke 变换是正交投影还是非正交投影？Park 变换呢？
5. 什么叫"电角度"和"机械角度"？对一个 7 极对电机，机械转一圈电角度走了多少弧度？
6. 为什么 STM32F405 的 PWM 用 TIM1/TIM8 而不是其他定时器？它们有什么特别？
7. FreeRTOS 中，ISR 和任务之间用什么同步？为什么不能用普通的 mutex？
8. ODrive 的 PWM 中断频率约 8 kHz，为什么不是 20 kHz 或更高？

> 答案不写在这里，但看完阶段二、三、四后会自动浮现。

---

## 1.8 检验标志

完成本阶段，你应能：

- [ ] 能用自己的话解释 "BLDC / PMSM / FOC / 级联 PID" 这些术语
- [ ] 能画出 ODrive 的层次架构（硬件 → 固件层 → 控制层 → 上位机）
- [ ] 知道 `main()` 做了哪几件大事
- [ ] 理解 `odrive-interface.yaml` 的作用和跨语言代码生成机制
- [ ] 成功编译固件（或至少能解释为什么没编译成功）
- [ ] 成功安装 Python 工具

---

下一步 → [03-phase2-hal.md：硬件抽象层与底层驱动](03-phase2-hal.md)

上一步 ← [01-overview.md：仓库全景](01-overview.md)
