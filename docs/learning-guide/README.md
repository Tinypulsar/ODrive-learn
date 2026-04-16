# ODrive 源码学习指南

> 一份面向有编程基础的工程师 / 研究者的 ODrive v3.x 开源电机控制器深度学习路线。目标：从零开始理解整个系统，最终具备**独立修改控制算法、做算法研究**的能力。

---

## 本指南适合谁

- 对 BLDC / PMSM 电机控制感兴趣的嵌入式工程师
- 想把 ODrive 作为算法研究平台的研究生 / 科研人员
- 有 C/C++ 或 Python 基础，但电机控制知识有限的开发者
- 希望深入理解一个**工业级开源电机控制器**架构设计的学习者

## 前置知识

- **必备**：C/C++ 基础、基本的线性代数、微积分
- **加分**：STM32 / ARM Cortex-M 经验、控制理论基础（PID、状态空间）、信号处理基础
- **不要求**：电机控制、FOC、FreeRTOS（本指南会讲）

---

## 学习路线总览

本指南采用 **自底向上 + 沿信号流方向** 的设计：先建立理论与项目概览，然后从硬件驱动出发，逐层向上学习到上层控制器，最后进入算法研究阶段。

| 阶段 | 文档 | 主题 | 预计时间 |
|------|------|------|----------|
| 0 | [01-overview.md](01-overview.md) | 仓库全景与架构概览 | 1 周 |
| 1 | [02-phase1-foundation.md](02-phase1-foundation.md) | 基础理论与项目概览 | 1-2 周 |
| 2 | [03-phase2-hal.md](03-phase2-hal.md) | 硬件抽象层与底层驱动 | 2 周 |
| 3 | [04-phase3-encoder.md](04-phase3-encoder.md) | 编码器与位置估计 | 2 周 |
| 4 | [05-phase4-foc.md](05-phase4-foc.md) | FOC 核心与电机控制 | 3 周 |
| 5 | [06-phase5-controller.md](06-phase5-controller.md) | 上层控制器与状态机 | 3 周 |
| 6 | [07-phase6-communication.md](07-phase6-communication.md) | 通信协议与工具链 | 2 周 |
| 7 | [08-phase7-research.md](08-phase7-research.md) | 仿真分析与算法研究 | 持续 |
| — | [09-appendix.md](09-appendix.md) | 附录：关键文件清单、资源 | — |

**总周期**：约 14-16 周 + 持续研究阶段

> 💡 **使用建议**：每个阶段都有明确的"学习目标"、"阅读顺序"、"动手实验"、"检验标志"。不要跳过"动手实验"——只看代码不动手是学不会电机控制的。

---

## 控制链路一图看懂

```
                     ┌────────────────────────────────────────────────────┐
上位机/用户 ──[CAN/USB/UART]──► │                   通信层                   │
                                │ (ascii_protocol / fibre / can_simple)     │
                                └──────────────┬─────────────────────────────┘
                                               │ set input_pos / vel / torque
                                               ▼
                                ┌─────────────────────────────┐
                                │       Controller 上层         │  (controller.cpp)
                                │  ▸ 输入模式：PASSTHROUGH /    │
                                │    TRAP_TRAJ / POS_FILTER 等  │
                                │  ▸ 位置环 → 速度环 → 力矩      │
                                │  ▸ 抗齿槽前馈 / 增益调度       │
                                └──────────────┬──────────────┘
                                               │ torque_setpoint
                                               ▼
                                ┌─────────────────────────────┐
                                │       Motor (motor.cpp)       │
                                │  ▸ 力矩 → Idq 目标            │
                                │  ▸ 电阻/电感校准             │
                                │  ▸ 温度限流                  │
                                └──────────────┬──────────────┘
                                               │ Idq_setpoint
                                               ▼
                                ┌─────────────────────────────┐
                                │    FOC (foc.cpp)              │
                                │  ▸ Clarke/Park 变换           │
                                │  ▸ PI 电流环                 │
                                │  ▸ 空间矢量调制 (SVM)         │
                                └──────────────┬──────────────┘
                                               │ PWM timings
                                               ▼
                                ┌─────────────────────────────┐
                                │   low_level / DRV8301 / PWM   │
                                │   硬件层：STM32 TIM/ADC/SPI   │
                                └──────────────┬──────────────┘
                                               ▼
                                          ┌────────┐
                                          │ 电机 + 编码器 │
                                          └────┬───┘
                                               │
                     ┌─────────────────────────┴─────────────────────────┐
                     ▼                                                    ▼
          ┌──────────────────┐                                ┌──────────────────┐
          │  Encoder         │                                │ Sensorless       │
          │  (encoder.cpp)   │                                │ Estimator        │
          │  ▸ PLL 估计位置/速度 │                              │ (反电动势观测器) │
          └─────────┬────────┘                                └────────┬─────────┘
                    └────────────── phase / vel_estimate ─────────────┘
                                               │
                                               └─► Controller / FOC
```

**关键时序**：整个控制环路以 **~8 kHz**（`current_meas_hz`）的频率运行，由 PWM 定时器中断驱动。

---

## 项目核心文件速览

| 模块 | 核心文件 | 行数（约） | 难度 |
|------|---------|--------|------|
| 入口 | `Firmware/MotorControl/main.cpp` | 500+ | ★★ |
| 状态机 | `Firmware/MotorControl/axis.cpp` | 600+ | ★★★★ |
| FOC | `Firmware/MotorControl/foc.cpp` | 190 | ★★★★ |
| 控制器 | `Firmware/MotorControl/controller.cpp` | 450 | ★★★★ |
| 电机管理 | `Firmware/MotorControl/motor.cpp` | 600+ | ★★★★ |
| 编码器 | `Firmware/MotorControl/encoder.cpp` | 900+ | ★★★★ |
| 无传感器 | `Firmware/MotorControl/sensorless_estimator.cpp` | 110 | ★★★★★ |
| 轨迹规划 | `Firmware/MotorControl/trapTraj.cpp` | 92 | ★★★ |
| 底层硬件 | `Firmware/MotorControl/low_level.cpp` | 1000+ | ★★★★★ |

> 难度说明：★★ = 需要基础阅读；★★★ = 需要对应领域知识；★★★★ = 复杂算法 / 系统耦合；★★★★★ = 安全关键 / 数学推导密集

---

## 如何高效使用本指南

### 三遍阅读法（推荐）

- **第一遍：粗读**
  快速浏览所有章节，建立全局印象。不求看懂每个公式和代码细节，只求知道"哪块代码做什么事"。

- **第二遍：精读 + 动手**
  一个阶段一个阶段地精读，严格按阅读顺序阅读源码，完成每章的"动手实验"。关键算法需要推导数学公式。

- **第三遍：贯通 + 改造**
  随便挑一个"研究方向"做实验，把某个模块换掉或改进，验证全链路仍能工作。这时候才真正掌握了 ODrive。

### 配合硬件与仿真

- **无硬件**：使用 `analysis/Simulation/MotorSim.py` 做 Python 仿真，理解算法行为
- **有硬件**（ODrive v3.x + BLDC 电机 + 编码器）：所有"动手实验"都能在真机上验证
- **半硬件**：用 ODrivetool 连接到真机，只做配置、读取状态、查看波形，不做破坏性修改

---

## 学习成果检验

完成整个指南后，你应该能：

✅ **理解级**：解释 ODrive 从用户发出 `odrv0.axis0.controller.input_pos = 10` 到电机转动之间发生的**所有事情**

✅ **调试级**：看到 `ERROR_SPINOUT_DETECTED` 能立刻知道去查哪些寄存器、哪些变量、可能的原因

✅ **修改级**：能在 `controller.cpp` 中添加一个新的 `InputMode`，配置 Python 侧暴露，测试通过

✅ **研究级**：能独立设计并实现一个新的无传感器观测器（如 SMO / EKF），在仿真和真机上对比性能

---

## 许可证与来源

本学习指南基于 [ODrive Robotics](https://github.com/odriverobotics/ODrive) 的开源项目编写，遵循与原项目相同的 MIT 许可证。

---

下一步 → [01-overview.md：仓库全景与架构概览](01-overview.md)
