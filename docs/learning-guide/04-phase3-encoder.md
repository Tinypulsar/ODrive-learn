# 阶段三 · 编码器与位置估计（第 5-6 周）

> **目标**：掌握位置/速度反馈的完整链路，包括**有传感器**（增量、绝对、霍尔、sincos）和**无传感器**（反电动势观测器）两大类方案。

---

## 3.1 学习目标

- 理解各种编码器的原理与优劣
- 掌握 ODrive 中 **PLL（锁相环）位置/速度估计器** 的数学原理
- 看懂 **编码器偏移校准** 流程（为什么需要、怎么做）
- 理解 Lee-Ortega 非线性磁链观测器的思想
- 能在代码和论文公式之间建立对应关系

---

## 3.2 前置理论

### 各类位置传感器对比

| 类型 | 输出 | 分辨率 | 绝对位置 | 成本 | ODrive 支持 |
|------|------|--------|---------|------|------|
| **增量编码器**（AB + index） | 两路方波脉冲 | 2000-10000 ppr | 需索引脉冲 | 低 | MODE_INCREMENTAL |
| **霍尔传感器** | 3 路方波（120° 相差）| 6 states / 电周期 | 是（但粗）| 很低 | MODE_HALL |
| **Sin/Cos 编码器** | 模拟正弦余弦 | 连续 | 否 | 中 | MODE_SINCOS |
| **SPI 绝对值编码器**（AMS/RLS/CUI/AEAT/MA732） | 数字串行 | 12-14 bit | 是 | 中-高 | MODE_SPI_ABS_* |

### 锁相环（PLL）原理

PLL 是一个反馈系统，目的是让"估计位置"跟踪"测量位置"：

```
            e = x_meas - x_est
 x_meas ──►+⊖──► Kp ──►────────► x_est
           ▲            │
           │            ↓
           │         [积分]
           │            │
           │            ▼     ←── Ki 分支
           │─── 其实还有 vel 状态
```

ODrive 实现的是**二阶 PLL**：
- 状态：`pos_estimate`（位置）、`vel_estimate`（速度）
- 每个控制周期：
  1. 预测：`pos_estimate += Ts * vel_estimate`
  2. 误差：`err = pos_measured - pos_estimate`
  3. 校正：`pos_estimate += Ts * Kp * err`
  4. `vel_estimate += Ts * Ki * err`
- **临界阻尼**设计：`Ki = Kp² / 4`，由 `bandwidth` 参数决定：
  - `Kp = 2 * bandwidth`
  - `Ki = Kp² / 4`

**优点**：
- 同时估计位置和速度（速度是副产品，不需要差分）
- 抑制测量噪声
- 可调带宽 trade-off 响应速度 vs 噪声

---

## 3.3 阅读顺序

### 第 1 天：Encoder 配置和主框架

**源文件**：`Firmware/MotorControl/encoder.hpp`

关注 `Encoder::Config_t` 中的关键字段：
```cpp
Mode mode = MODE_INCREMENTAL;       // 编码器类型
float bandwidth = 1000.0f;          // PLL 带宽 [rad/s]
int32_t cpr = 2048 * 4;             // counts per revolution（4x 解码后）
int32_t phase_offset = 0;           // 编码器和电机电角度的偏移
float phase_offset_float = 0.0f;    // 亚计数级的偏移（插值用）
bool use_index = false;             // 是否用索引脉冲
bool pre_calibrated = false;        // 偏移是否已存储
```

还要看 **状态变量**：
```cpp
int32_t shadow_count_ = 0;          // 未模 cpr 的累积计数
int32_t count_in_cpr_ = 0;          // [0, cpr) 范围内的计数
float pos_estimate_counts_ = 0.0f;  // PLL 估计的位置（浮点 count）
float vel_estimate_counts_ = 0.0f;  // PLL 估计的速度
OutputPort<float> phase_;           // 电角度 [rad] → 给 FOC
OutputPort<float> phase_vel_;       // 电角速度 [rad/s]
OutputPort<float> pos_estimate_;    // 位置 [turn]
OutputPort<float> vel_estimate_;    // 速度 [turn/s]
```

### 第 2-3 天：PLL 主循环

**源文件**：`Firmware/MotorControl/encoder.cpp` 中的 `Encoder::update()`（约 650 行开始）

这是本文件最重要的函数。按以下子结构阅读：

#### 步骤 1：从传感器读取计数
```cpp
switch (mode_) {
    case MODE_INCREMENTAL: {
        delta_enc = tim_cnt_sample_ - shadow_count_;  // 差值
    } break;
    
    case MODE_HALL: {
        // 解码霍尔状态（3 位）→ 0-5 扇区
        decode_hall_samples();
        if (decode_hall(hall_state_ ^ config_.hall_polarity, &hall_cnt)) {
            delta_enc = hall_cnt - count_in_cpr_;
            delta_enc = mod(delta_enc, 6);      // 6 个扇区
            if (delta_enc > 3) delta_enc -= 6;
        }
    } break;
    
    case MODE_SINCOS: {
        float phase = fast_atan2(sample_s, sample_c);
        int fake_count = (int)(1000.0f * phase);  // CPR = 2π × 1000 ≈ 6283
        delta_enc = mod(fake_count - count_in_cpr_, 6283);
    } break;
    
    case MODE_SPI_ABS_*: {
        // 异步 SPI 读取，pos_abs_ 由 DMA 回调更新
        delta_enc = pos_abs_latched - count_in_cpr_;
        delta_enc = mod(delta_enc, config_.cpr);
    } break;
}
shadow_count_ += delta_enc;
count_in_cpr_ = mod(count_in_cpr_ + delta_enc, config_.cpr);
```

#### 步骤 2：PLL 更新
```cpp
// 预测
pos_estimate_counts_ += current_meas_period * vel_estimate_counts_;
pos_cpr_counts_      += current_meas_period * vel_estimate_counts_;

// 相位检测器（误差）
float delta_pos_counts = shadow_count_ - encoder_model(pos_estimate_counts_);
float delta_pos_cpr_counts = count_in_cpr_ - encoder_model(pos_cpr_counts_);
delta_pos_cpr_counts = wrap_pm(delta_pos_cpr_counts, config_.cpr);  // 处理 cpr 环绕

// 校正
pos_estimate_counts_ += current_meas_period * pll_kp_ * delta_pos_counts;
pos_cpr_counts_ += current_meas_period * pll_kp_ * delta_pos_cpr_counts;
vel_estimate_counts_ += current_meas_period * pll_ki_ * delta_pos_cpr_counts;
```

#### 步骤 3：输出
```cpp
pos_estimate_ = pos_estimate_counts_ / config_.cpr;
vel_estimate_ = vel_estimate_counts_ / config_.cpr;

// 电角度输出（给 FOC）
int32_t corrected_enc = count_in_cpr_ - config_.phase_offset;
// ... 插值 ...
phase_ = ((float)corrected_enc + interpolation_) * elec_rad_per_enc;
phase_vel_ = vel_estimate_counts_ * elec_rad_per_enc;
```

**关键细节：插值（interpolation）**
增量编码器每个 count 之间有"空白"，如果原样输出会造成相位台阶。ODrive 根据速度和 PWM 时刻**线性插值**填补这个空白：
```cpp
if (delta_enc > 0) {
    interpolation_ = 0.0f;  // 刚跨过边沿，插值归 0
} else {
    interpolation_ += current_meas_period * vel_estimate_counts_;
}
```

### 第 4 天：校准流程

#### `run_offset_calibration()`

目标：找到 `phase_offset`，使得"编码器 count = 0 + phase_offset"对应"电角度 = 0"。

算法：
1. 给电机 d 轴（即对齐相位为 0）**施加直流电流**（通过 `open_loop_controller`）
2. 让转子慢慢转到对齐位置（锁定）
3. 读取此时的编码器 count，就是 `phase_offset`
4. 再向另一方向扫描确认重复性

对应代码段：`encoder.cpp::run_offset_calibration()`

#### `run_index_search()`

目标：找到编码器的 Z 脉冲位置，作为"绝对零位"。

算法：
1. 开环低速旋转（`run_lockin_spin`）
2. 订阅 index GPIO 的外部中断
3. 中断触发时记录当前 `shadow_count_`
4. 把这个值作为后续的 `index_offset`

#### 霍尔校准

**`run_hall_polarity_calibration()`**：测量霍尔传感器的输入极性（某些电机霍尔接线顺序不一致）

**`run_hall_phase_calibration()`**：测量每个霍尔状态切换对应的电角度，存入 `hall_edge_phcnt[6]`

### 第 5-6 天：绝对值编码器的 SPI 实现

**函数**：
- `Encoder::abs_spi_start_transaction()` — 发起读请求
- `Encoder::abs_spi_cb()` — DMA 完成回调

**核心技巧**：
- 使用 SPI DMA，不阻塞主循环
- 通过 `Stm32SpiArbiter` 排队请求
- 根据编码器厂家解析不同的位格式（AMS / RLS / CUI / AEAT / MA732 各不同）
- 失败率 `spi_error_rate_` 做低通滤波，超过 5% 报错

### 第 7 天：无传感器观测器

**源文件**：`Firmware/MotorControl/sensorless_estimator.cpp`

这是 ODrive 最精华的算法之一。基于论文：

> Lee, Hong, Nam, Ortega, Praly, Astolfi, "Sensorless Control of Surface-Mount Permanent-Magnet Synchronous Motors Based on a Nonlinear Observer," IEEE Transactions on Power Electronics, 2010.

#### 算法数学推导

表面贴装永磁电机（SPMSM）在 αβ 坐标系下的磁链方程：
```
dλ_α/dt = v_α - R·i_α
dλ_β/dt = v_β - R·i_β
```

其中磁链 `λ = L·i + η`，`η = Ψm · [cos(θ), sin(θ)]` 是永磁磁链。

定义估计的磁链 `x̂`，观测器方程：
```
dx̂/dt = -R·i + v + (γ/2)·(Ψm² - ||η̂||²)·η̂
```

最后一项是**非线性校正项**：它驱使估计磁链幅值收敛到永磁磁链 `Ψm`。

#### 代码对应

```cpp
// 1. Clarke 变换
float I_alpha_beta[2] = {
    current_meas->phA,
    one_by_sqrt3 * (current_meas->phB - current_meas->phC)
};

// 2. 磁链积分预测（论文 eqn 4）
for (int i = 0; i < 2; ++i) {
    float y = -R * I_alpha_beta[i] + V_alpha_beta_memory_[i];
    flux_state_[i] += y * Ts;
    eta[i] = flux_state_[i] - L * I_alpha_beta[i];   // eqn 6
}

// 3. 非线性校正（论文 eqn 8）
float est_pm_flux_sqr = eta[0]² + eta[1]²;
float bandwidth_factor = 1.0f / Ψm²;
float eta_factor = 0.5f * observer_gain * bandwidth_factor * (Ψm² - est_pm_flux_sqr);

for (int i = 0; i < 2; ++i) {
    flux_state_[i] += eta_factor * eta[i] * Ts;
    eta[i] = flux_state_[i] - L * I_alpha_beta[i];   // 更新 eta
}

// 4. 从 eta 中用 PLL 提取相位
phase = atan2(eta[1], eta[0]);
delta_phase = wrap_pm_pi(phase - pll_pos_);
pll_pos_ += Ts * pll_kp * delta_phase;
phase_vel += Ts * pll_ki * delta_phase;
```

**三个调参**：
- `observer_gain`：校正项增益（太大易震荡、太小跟不上）
- `pll_bandwidth`：PLL 带宽
- `pm_flux_linkage`：永磁磁链 Ψm（必须正确，否则发散）

**局限性**：
- **低速不行**：反电动势 ∝ 速度，低速时 SNR 太差
- 电阻 R 温漂会引入误差（可以加**在线辨识**）
- 仅适用于 SPMSM，IPMSM 需要考虑交叉耦合项

---

## 3.4 动手实验

**实验 3-1：推导 PLL 增益**

给定 `bandwidth = 1000 rad/s`，临界阻尼：
- `Kp = 2 × 1000 = 2000`
- `Ki = Kp² / 4 = 1,000,000`
- 离散化稳定性：`Ts × Kp < 1` → `125e-6 × 2000 = 0.25 < 1` ✓

如果 `bandwidth = 5000`，稳定性还满足吗？

**实验 3-2：仿真 PLL**

在 Python 中实现一个 PLL，输入是加噪的位置信号（带一定频率的正弦），观察 `bandwidth` 对噪声抑制和响应速度的影响。

```python
import numpy as np
import matplotlib.pyplot as plt

dt = 1/8000
T = 1.0
t = np.arange(0, T, dt)
pos_true = 10 * np.sin(2 * np.pi * 5 * t)  # 5 Hz 振荡
pos_meas = pos_true + 0.01 * np.random.randn(len(t))

def pll(bandwidth):
    Kp = 2 * bandwidth
    Ki = Kp**2 / 4
    pos_est, vel_est = 0, 0
    log = []
    for p in pos_meas:
        pos_est += dt * vel_est
        err = p - pos_est
        pos_est += dt * Kp * err
        vel_est += dt * Ki * err
        log.append(pos_est)
    return np.array(log)

for bw in [100, 1000, 5000]:
    plt.plot(t, pll(bw), label=f'bw={bw}')
plt.plot(t, pos_meas, '.', alpha=0.2, label='meas')
plt.plot(t, pos_true, 'k--', label='true')
plt.legend(); plt.show()
```

**实验 3-3：画状态机**

用 PlantUML 或任何工具画出 `AXIS_STATE_ENCODER_OFFSET_CALIBRATION` 的完整流程图。

**实验 3-4：无传感器仿真**

用 `analysis/Simulation/MotorSim.py` 给自己做一个仿真测试：实现 Lee-Ortega 观测器，输入仿真的 v_αβ 和 i_αβ，观察在不同速度下的估计误差。

---

## 3.5 常见问题

**Q1：为什么需要 `shadow_count_` 和 `count_in_cpr_` 两个变量？**

A：
- `shadow_count_` 累积不模 cpr，用于追踪绝对位置
- `count_in_cpr_` 模 cpr，用于电角度输出（电角度本来就是周期性的）
- 两个变量互相配合避免 32 位溢出问题

**Q2：霍尔传感器只有 6 个状态，怎么做连续位置估计？**

A：
- 霍尔本身只在状态切换的瞬间给出"硬"信息
- PLL 根据前面几次切换的时间估计速度
- 然后用速度 × 时间去"外推"位置
- 这就是为什么霍尔低速抖、高速平稳

**Q3：绝对值编码器的 SPI 读怎么保证不丢数据？**

A：
- SPI 用 DMA 异步读取
- 每个 PWM 周期发起一次新请求
- 如果前一次还没完成就进新周期，不发起
- `spi_error_rate_` 做长时间统计，超阈值才报错

**Q4：无传感器启动为什么需要 lockin spin？**

A：
- 刚上电时转子位置未知
- 观测器需要磁链积分达到稳态，这需要时间
- 所以先用**开环电流**拖着转子加速到一定速度（反电动势足够大）
- 然后观测器才能可靠工作
- 最后切换到闭环

---

## 3.6 检验标志

- [ ] 能推导出 PLL 增益与带宽的关系
- [ ] 能解释 "插值" 对高速位置估计的作用
- [ ] 能说清编码器偏移校准为什么必要、怎么做
- [ ] 能对照论文 eqn 4、6、8 找到 `sensorless_estimator.cpp` 中的对应代码
- [ ] 能解释为什么无传感器不能零速启动
- [ ] 能列出各种编码器模式的优缺点

---

下一步 → [05-phase4-foc.md：FOC 核心与电机控制](05-phase4-foc.md)

上一步 ← [03-phase2-hal.md：硬件抽象层](03-phase2-hal.md)
