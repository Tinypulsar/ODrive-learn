# 阶段四 · FOC 核心与电机控制（第 7-9 周）

> **目标**：深度理解 ODrive FOC 算法的完整实现——从三相电流采样到 PWM 输出的全过程。本阶段是整个指南的**技术顶点**。

---

## 4.1 学习目标

- 能**手推** Clarke 变换、Park 变换及其逆变换的矩阵
- 理解 **空间矢量调制（SVPWM）** 的几何原理
- 能对照代码行追踪 FOC 算法的每一步
- 理解 **PI 控制器的积分抗饱和** 设计
- 看懂电机的**电阻/电感自动测量**算法
- 理解力矩 → Iq 的转换关系

---

## 4.2 前置理论

### 4.2.1 Clarke 变换（abc → αβ）

三相电流 `iA, iB, iC`（满足 `iA + iB + iC = 0`）转换到两相静止坐标系 `iα, iβ`：

**幅值不变形式**：
```
iα = iA
iβ = (iA + 2·iB) / √3 = (iB - iC) / √3    ← ODrive 用这种
```

ODrive 代码（`foc.cpp:14-17`）：
```cpp
Ialpha_beta = {
    (*currents)[0],
    one_by_sqrt3 * ((*currents)[1] - (*currents)[2])
};
// 即：Iα = iA, Iβ = (iB - iC) / √3
```

**几何解释**：三相正交基向量投影到两个正交轴 α（与 A 相重合）和 β（与 α 相差 90°）。

### 4.2.2 Park 变换（αβ → dq）

从静止的 αβ 系转到跟随转子旋转的 dq 系：

```
id =  cos(θ)·iα + sin(θ)·iβ
iq = -sin(θ)·iα + cos(θ)·iβ
```

或写成矩阵：
```
[id]   [ cos(θ)  sin(θ)] [iα]
[iq] = [-sin(θ)  cos(θ)] [iβ]
```

**几何解释**：以电角度 `θ` 旋转坐标系，让 d 轴对齐转子 N 极方向。

ODrive 代码：
```cpp
float c_I = cos(I_phase);
float s_I = sin(I_phase);
Idq = {
    c_I * Ialpha + s_I * Ibeta,     // Id
    c_I * Ibeta - s_I * Ialpha      // Iq
};
```

### 4.2.3 d/q 轴的物理意义

- **d 轴（直轴）**：与转子永磁体 N 极方向对齐
- **q 轴（交轴）**：与 d 轴正交，是"产生力矩"的方向

对于 SPMSM（表贴电机），力矩方程：
```
Te = (3/2) · p · Ψm · iq
```

其中 p 是极对数，Ψm 是永磁磁链。**力矩正比于 Iq**，Id 不产生力矩但会产生磁场（弱磁时用）。

### 4.2.4 逆 Park 变换（dq → αβ）

```
vα = cos(θ)·vd - sin(θ)·vq
vβ = cos(θ)·vq + sin(θ)·vd
```

ODrive 代码：
```cpp
float mod_alpha = c_p * mod_d - s_p * mod_q;
float mod_beta  = c_p * mod_q + s_p * mod_d;
```

### 4.2.5 空间矢量调制（SVPWM）

把 αβ 空间的电压矢量 `(vα, vβ)` 分解成**两个相邻的基本矢量**（V1-V6）加上**零矢量**（V0, V7）的时间加权组合。

六个基本矢量对应逆变器 6 种开关状态：
```
V1 (100) = Vdc · (1, 0)
V2 (110) = Vdc · (1/2, √3/2)
V3 (010) = Vdc · (-1/2, √3/2)
V4 (011) = Vdc · (-1, 0)
V5 (001) = Vdc · (-1/2, -√3/2)
V6 (101) = Vdc · (1/2, -√3/2)
V0 (000), V7 (111) 是零矢量
```

SVM 的输出是三个 PWM 占空比 `tA, tB, tC`。

ODrive 的 `SVM()` 函数实现在 `utils.cpp`/`utils.hpp`，采用的是"中心对齐 + 零矢量对称分配"的标准实现。

**最大调制度**：对于线性区 `||V|| ≤ Vdc / √3`（约 0.577 · Vdc），超过会进入**过调制**区。ODrive 代码中限幅到 `0.80 × √3/2 ≈ 0.693`（留一定余量给 SVPWM）。

### 4.2.6 PI 电流控制器

电压方程（dq 系）：
```
vd = R·id + L·(did/dt) - ω·L·iq
vq = R·iq + L·(diq/dt) + ω·L·id + ω·Ψm
```

后面两项是**交叉耦合**和**反电动势前馈**。

PI 控制：
```
vd = Kp · (id_ref - id) + Ki · ∫(id_ref - id) dt
vq = Kp · (iq_ref - iq) + Ki · ∫(iq_ref - iq) dt
```

**增益整定**（基于一阶模型近似）：
- 选择闭环带宽 `ωc`（rad/s，在 Motor::Config_t 中是 `current_control_bandwidth`）
- `Kp = L · ωc`
- `Ki = R · ωc`
- 时间常数 `τ = L/R`，Kp/Ki 比值 = L/R，消除了开环零极点，留下单一极点 ωc

ODrive `motor.cpp::update_current_controller_gains()`：
```cpp
pi_gains_ = {phase_inductance * current_control_bandwidth,
             phase_resistance * current_control_bandwidth};
```

---

## 4.3 阅读顺序

### 第 1-2 天：FOC 类结构

**源文件**：`Firmware/MotorControl/foc.hpp`

类继承：
```
ComponentBase ──┐
                ├──► AlphaBetaFrameController ──► FieldOrientedController
PhaseControlLaw<3> ─┘
```

- `PhaseControlLaw<3>`（`phase_control_law.hpp`）：三相输入接口
- `AlphaBetaFrameController`：把三相 → αβ 的 Clarke 变换和 αβ → PWM 的 SVM 封装掉
- `FieldOrientedController`：真正的 FOC，实现 Park / PI / 逆 Park

**关键输入端口**：
```cpp
InputPort<float2D> Idq_setpoint_src_;    // 电流设定值 (Id, Iq)
InputPort<float2D> Vdq_setpoint_src_;    // 电压前馈（或开环时直接用这个）
InputPort<float>   phase_src_;           // 电角度
InputPort<float>   phase_vel_src_;       // 电角速度（用于延时补偿）
bool enable_current_control_src_;        // true=电流模式, false=电压模式
```

**关键状态**：
```cpp
std::optional<float2D> pi_gains_;                // [Kp, Ki]
float v_current_control_integral_d_;             // Id 积分器
float v_current_control_integral_q_;             // Iq 积分器
float final_v_alpha_, final_v_beta_;             // 实际施加电压（给 sensorless 用）
```

### 第 3-4 天：FOC 主逻辑

**源文件**：`Firmware/MotorControl/foc.cpp`

#### `on_measurement()`（接收电流测量）

```cpp
// 入口：每次 ADC 采样完成后
Motor::Error FieldOrientedController::on_measurement(
        std::optional<float> vbus_voltage,
        std::optional<float2D> Ialpha_beta,
        uint32_t input_timestamp) {
    i_timestamp_ = input_timestamp;
    vbus_voltage_measured_ = vbus_voltage;
    Ialpha_beta_measured_ = Ialpha_beta;
    return Motor::ERROR_NONE;
}
```

**注意**：这里**只存数据**，真正计算在 `get_alpha_beta_output()` 里。这种分离是为了**时序灵活性**——ADC 采样和 PWM 更新可以异步。

#### `get_alpha_beta_output()`（核心计算）

按以下 5 个步骤阅读：

**步骤 1：时序检查**
```cpp
if (abs((int32_t)(i_timestamp_ - ctrl_timestamp_)) > MAX_CONTROL_LOOP_UPDATE_TO_CURRENT_UPDATE_DELTA) {
    return Motor::ERROR_BAD_TIMING;
}
```

**步骤 2：Park 变换（加相位延时补偿）**
```cpp
// 补偿 ADC 采样到现在的时间差：相位前移 ω·Δt
float I_phase = phase + phase_vel * ((float)(i_timestamp_ - ctrl_timestamp_) / TIM_1_8_CLOCK_HZ);
float c_I = cos(I_phase), s_I = sin(I_phase);
Idq = {
    c_I * Ialpha + s_I * Ibeta,       // Id
    c_I * Ibeta - s_I * Ialpha        // Iq
};
```

这里的**延时补偿**非常关键——高速时电角度变化快（比如 10000 rpm × 7 极对 = 70000 电 rpm = 7330 rad/s），PWM 周期 125 μs 就转了 0.92 rad（53°）！如果不补偿，Park 会用"过期"的相位。

**步骤 3：PI 控制 + 抗饱和**
```cpp
float Ierr_d = Id_setpoint - Id;
float Ierr_q = Iq_setpoint - Iq;

// 输出 = 前馈 + 积分 + 比例
mod_d = V_to_mod * (Vd + v_integral_d_ + Ierr_d * p_gain);
mod_q = V_to_mod * (Vq + v_integral_q_ + Ierr_q * p_gain);

// 过调制保护
float mod_scalefactor = 0.80f * sqrt3_by_2 / sqrt(mod_d² + mod_q²);
if (mod_scalefactor < 1.0f) {
    mod_d *= mod_scalefactor;
    mod_q *= mod_scalefactor;
    v_integral_d_ *= 0.99f;    // 积分器衰减（clamping / back-calculation）
    v_integral_q_ *= 0.99f;
} else {
    v_integral_d_ += Ierr_d * i_gain * Ts;
    v_integral_q_ += Ierr_q * i_gain * Ts;
}
```

**抗饱和设计**：当输出饱和时，停止积分 + 衰减已有积分，避免 "integrator windup"。衰减因子 0.99 是经验值。

**步骤 4：逆 Park**
```cpp
// 再次补偿延时（到下一个 PWM 更新时刻）
float pwm_phase = phase + phase_vel * ((float)(output_timestamp - ctrl_timestamp_) / TIM_1_8_CLOCK_HZ);
float c_p = cos(pwm_phase), s_p = sin(pwm_phase);
float mod_alpha = c_p * mod_d - s_p * mod_q;
float mod_beta  = c_p * mod_q + s_p * mod_d;
```

**步骤 5：输出与记录**
```cpp
*mod_alpha_beta = {mod_alpha, mod_beta};
final_v_alpha_ = mod_to_V * mod_alpha;   // 记录实际电压给 sensorless
final_v_beta_  = mod_to_V * mod_beta;
*ibus = mod_d * Id + mod_q * Iq;         // 估算母线电流
power_ = vbus_voltage * ibus;            // 瞬时功率
```

#### `get_output()` in `AlphaBetaFrameController`

```cpp
// αβ → 三相 PWM
auto [tA, tB, tC, success] = SVM(mod_alpha_beta->first, mod_alpha_beta->second);
pwm_timings[0] = tA;
pwm_timings[1] = tB;
pwm_timings[2] = tC;
```

### 第 5 天：Motor 类

**源文件**：`Firmware/MotorControl/motor.hpp` / `motor.cpp`

#### 配置参数
```cpp
int32_t pole_pairs = 7;
float phase_inductance = 0.0f;        // [H]
float phase_resistance = 0.0f;        // [Ω]
float torque_constant = 0.04f;        // [Nm/A]
MotorType motor_type = MOTOR_TYPE_HIGH_CURRENT;
float current_lim = 10.0f;            // [A]
float current_control_bandwidth = 1000.0f;  // [rad/s]
```

#### 力矩 → Iq 转换
在 `Motor::update()` 中：
```cpp
float Iq = torque_setpoint / torque_constant;
Idq_setpoint_ = {0.0f, Iq};   // Id = 0 (不做弱磁)
```

对于 ACIM（异步电机），需要特殊处理（见 `acim_estimator.cpp`）。

### 第 6-7 天：电机参数自动校准

#### `measure_phase_resistance()` — 电阻测量

原理：
1. 用一个特殊的 **ResistanceMeasurementControlLaw**（在 `motor.cpp` 顶部定义）
2. 给定 `target_current`，内置一个慢积分器调节电压
3. 稳态后 `R = V / I`

代码片段（`motor.cpp:31-34`）：
```cpp
// 每个周期：
test_voltage_ += (kI * Ts) * (target_current_ - actual_current_);
// 稳态时 target_current_ = actual_current_, test_voltage 稳定在 R·I
```

#### `measure_phase_inductance()` — 电感测量

原理：
1. 施加**方波电压**（正反交替切换）
2. 测量电流波形的斜率
3. 由 `V = L · di/dt` 反推 `L = V / (di/dt)`

代码（`motor.cpp` 中的 **InductanceMeasurementControlLaw**）：
```cpp
// 每周期翻转电压极性
test_voltage *= -1;
// 测量半周期内电流的变化量 Δi
// L = (V · T/2) / Δi
```

#### `run_calibration()` — 完整校准流程

```cpp
bool Motor::run_calibration() {
    // 1. 检查电压
    // 2. 测电阻
    measure_phase_resistance(config_.calibration_current, config_.resistance_calib_max_voltage);
    // 3. 测电感（用较高电压、短脉冲）
    measure_phase_inductance(config_.resistance_calib_max_voltage);
    // 4. 自动更新 PI 增益
    update_current_controller_gains();
    is_calibrated_ = true;
}
```

---

## 4.4 完整信号流总图

```
┌─────────────┐                            ┌──────────────┐
│ ADC 中断    │ 三相电流 iA,iB,iC         │ PWM 定时器  │ TIM1 / TIM8
│  ↓          │──┐                      ┌──│  ↑          │
└─────────────┘  │                      │  └──────────────┘
                 │                      │
                 ▼                      │
           [Clarke 变换]                │
             iα, iβ                     │
                 │                      │
                 ▼                      │
           [Park 变换]  ← phase (来自 Encoder/Sensorless)
             id, iq                     │
                 │                      │
                 ▼                      │
           [PI 电流控制] ← Id_ref, Iq_ref (来自 Motor/Controller)
            vd, vq                      │
                 │                      │
                 ▼                      │
           [逆 Park 变换] ← phase       │
            vα, vβ                      │
                 │                      │
                 ▼                      │
           [SVM 调制]                   │
            tA, tB, tC ──────────────────┘ (PWM 占空比)
```

---

## 4.5 动手实验

**实验 4-1：手推 Clarke / Park 矩阵**

在纸上（或 LaTeX）推导：
1. 验证 iA + iB + iC = 0 下 Clarke 变换的对称性
2. 验证 Park 变换是正交变换（矩阵行列式 = 1）
3. 推导 Park 逆变换

**实验 4-2：仿真 FOC**

用 `analysis/Simulation/MotorSim.py` 作为起点，实现一个完整的 FOC 电流环。输入 Iq_ref 阶跃，观察：
- Iq 响应速度（应该接近 `current_control_bandwidth`）
- Id 有没有 "crosstalk"（交叉耦合）

```python
# 伪代码
R, L = 0.1, 100e-6
Kp = L * wc
Ki = R * wc
int_d, int_q = 0, 0

for t in range(N):
    id_err = id_ref - id
    iq_err = iq_ref - iq
    vd = Kp * id_err + int_d
    vq = Kp * iq_err + int_q
    int_d += Ki * id_err * dt
    int_q += Ki * iq_err * dt
    # ... 送入电机模型
```

**实验 4-3：过调制实验**

在仿真中突然把 Iq_ref 设得很大（超过当前电压能产生的最大值），观察：
- 电流是否能跟上
- 积分器是否 windup
- 打开 / 关闭抗饱和时行为差异

**实验 4-4：反推带宽**

假设你观察到 Iq 阶跃响应的上升时间（10%→90%）是 0.35 ms。估算 `current_control_bandwidth`。
（提示：一阶系统 tr ≈ 2.2/ωc）

**实验 4-5：SVM 几何可视化**

用 matplotlib 画出：
- 六个基本矢量 V1-V6 和零矢量
- 给定一个任意的 Vref，画出它落在哪个扇区
- 计算 T1、T2、T0 三个时间并标注

---

## 4.6 常见困惑

**Q1：为什么要做延时补偿？**

A：ADC 采样、控制计算、PWM 更新发生在不同时刻。高速时电角度变化快，不补偿会导致实际施加的电压方向与期望有偏差，降低控制精度甚至振荡。

**Q2：为什么 `mod_to_V = (2/3) · Vbus`？**

A：这是幅值不变 Clarke 变换下"最大线性调制电压" = (2/3) · Vbus 的系数。SVM 相对于正弦 PWM 多 15.5% 电压利用率。

**Q3：抗饱和衰减因子 0.99 怎么来的？**

A：经验值。0.99 对应"每 125 μs 衰减 1%"，时间常数约 12.5 ms。太快会积分跳变，太慢会 windup。正式工程中有更复杂的 back-calculation 公式。

**Q4：为什么电感测量要用方波而不是正弦？**

A：方波产生的 di/dt 是直线斜率，最容易测。正弦需要做 FFT 或滤波，更复杂。

**Q5：Iq_ref 从哪里来？**

A：
- 正常闭环：来自 `Controller::torque_output_` / torque_constant
- 校准：来自 `OpenLoopController`（开环施加电流）
- 电阻测量：来自 `ResistanceMeasurementControlLaw`（特殊控制律）

---

## 4.7 检验标志

- [ ] 能在纸上写出完整的 FOC 信号流（从 ADC 到 PWM）
- [ ] 能解释延时补偿为什么重要、代码里怎么做的
- [ ] 能说出 `current_control_bandwidth` 与 `Kp/Ki` 的关系
- [ ] 能识别代码中的抗饱和逻辑
- [ ] 能解释电阻 / 电感自动校准的原理
- [ ] 能说出 `final_v_alpha_` / `final_v_beta_` 的用途（给 sensorless 用）

---

下一步 → [06-phase5-controller.md：上层控制器与状态机](06-phase5-controller.md)

上一步 ← [04-phase3-encoder.md：编码器与位置估计](04-phase3-encoder.md)
