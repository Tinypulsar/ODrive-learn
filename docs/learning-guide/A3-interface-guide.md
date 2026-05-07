# 附录 A3 · ODrive 核心接口速查与问答

> 基于 `Firmware/odrive-interface.yaml` 的阅读笔记。
> 覆盖四大核心接口：**Axis → Controller → Motor → Encoder**，
> 以及阅读过程中产生的全部问答记录。

---

## 一、ODrive.Axis 接口

Axis 是 ODrive 中**最顶层的控制单元**，每个 Axis 聚合了一整套子组件（Motor、Controller、Encoder 等），并通过状态机协调它们的工作。ODrive v3 有两个 Axis（axis0、axis1），分别驱动两个电机。

### 1.1 错误标志

| 错误 | 含义 | 说明 |
|------|------|------|
| `INVALID_STATE` | 请求了非法状态 | 比如编码器没校准就进闭环 |
| `MOTOR_FAILED` | 电机子系统错误 | 查 `motor.error` 获取详情 |
| `ENCODER_FAILED` | 编码器子系统错误 | 查 `encoder.error` 获取详情 |
| `CONTROLLER_FAILED` | 控制器子系统错误 | 查 `controller.error` 获取详情 |
| `SENSORLESS_ESTIMATOR_FAILED` | 无传感器观测器错误 | — |
| `WATCHDOG_TIMER_EXPIRED` | 看门狗超时 | 超过 `config.watchdog_timeout` 没喂狗 |
| `MIN_ENDSTOP_PRESSED` | 碰到最小限位开关 | — |
| `MAX_ENDSTOP_PRESSED` | 碰到最大限位开关 | — |
| `ESTOP_REQUESTED` | 紧急停止（CAN 命令） | Emergency Stop |
| `HOMING_WITHOUT_ENDSTOP` | 归位时没有使能限位开关 | — |
| `OVER_TEMP` | 过温 | 查 `motor.error` 获取详情 |
| `UNKNOWN_POSITION` | 没有有效的位置估计 | — |

### 1.2 运行时状态

| 属性 | 类型 | 含义 |
|------|------|------|
| `current_state` | AxisState | 当前状态机状态 |
| `requested_state` | AxisState | 用户请求的目标状态（写入后被消费，读回通常为 UNDEFINED） |
| `is_homed` | bool | 是否已完成归位 |
| `step_dir_active` | bool | Step/Dir 脉冲模式是否激活 |
| `steps` | int64 | Step/Dir 模式下的累计脉冲数 |
| `last_drv_fault` | uint32 | DRV8301 栅极驱动芯片的故障码 |

### 1.3 AxisState 状态枚举

| 状态 | 值 | 含义 |
|------|-----|------|
| `UNDEFINED` | 0 | 未定义，自动切到 IDLE |
| `IDLE` | 1 | 空闲，PWM 关闭 |
| `STARTUP_SEQUENCE` | 2 | 上电启动序列（由 `config.startup_*` 标志决定） |
| `FULL_CALIBRATION_SEQUENCE` | 3 | 完整校准（电机 + 编码器） |
| `MOTOR_CALIBRATION` | 4 | 测量电机电阻和电感 |
| `ENCODER_INDEX_SEARCH` | 6 | 搜索编码器 Z 索引脉冲 |
| `ENCODER_OFFSET_CALIBRATION` | 7 | 编码器偏移校准 |
| `CLOSED_LOOP_CONTROL` | 8 | **闭环控制（正常运行状态）** |
| `LOCKIN_SPIN` | 9 | 锁相旋转（校准 / 无传感器启动） |
| `ENCODER_DIR_FIND` | 10 | 检测编码器方向 |
| `HOMING` | 11 | 归位（需使能限位开关） |
| `ENCODER_HALL_POLARITY_CALIBRATION` | 12 | 霍尔极性校准 |
| `ENCODER_HALL_PHASE_CALIBRATION` | 13 | 霍尔相位校准 |

典型流程：`IDLE → FULL_CALIBRATION_SEQUENCE → CLOSED_LOOP_CONTROL`

### 1.4 配置参数

| 参数 | 类型 | 含义 |
|------|------|------|
| `startup_motor_calibration` | bool | 上电时自动执行电机校准 |
| `startup_encoder_index_search` | bool | 上电时自动搜索 Z 脉冲 |
| `startup_encoder_offset_calibration` | bool | 上电时自动执行编码器偏移校准 |
| `startup_closed_loop_control` | bool | 校准完成后自动进入闭环 |
| `startup_homing` | bool | 校准完成后自动归位 |
| `enable_step_dir` | bool | 使能 Step/Dir 脉冲输入 |
| `enable_sensorless_mode` | bool | 使能无传感器模式 |
| `enable_watchdog` | bool | 使能看门狗 |
| `watchdog_timeout` | float [s] | 看门狗超时时间 |
| `step_gpio_pin` | uint16 | Step 脉冲输入引脚 |
| `dir_gpio_pin` | uint16 | Dir 方向输入引脚 |
| `calibration_lockin` | 子结构 | 校准时锁相旋转参数 |
| `sensorless_ramp` | LockinConfig | 无传感器启动的锁相配置 |
| `general_lockin` | LockinConfig | 通用锁相配置 |
| `can` | CanConfig | CAN 总线配置 |

### 1.5 子组件聚合

```
Axis
 ├── motor: Motor                        # 电机驱动与 FOC
 ├── controller: Controller              # 位置/速度/力矩控制
 ├── encoder: Encoder                    # 编码器处理
 ├── acim_estimator: AcimEstimator       # 异步电机估计器
 ├── sensorless_estimator: SensorlessEstimator  # 无传感器观测器
 ├── trap_traj: TrapezoidalTrajectory    # 梯形轨迹规划
 ├── min_endstop / max_endstop: Endstop  # 限位开关
 └── mechanical_brake: MechanicalBrake   # 机械制动器
```

### 1.6 函数

| 函数 | 含义 |
|------|------|
| `watchdog_feed()` | 喂看门狗，防止超时 |

---

## 二、ODrive.Controller 接口

Controller 实现了 ODrive 的**级联 PID 控制**：位置环（P）→ 速度环（PI）→ 力矩输出（送给 FOC 电流环）。它是你日常调参最频繁的接口。

### 2.1 错误标志

| 错误 | 含义 |
|------|------|
| `OVERSPEED` | 速度超过 `vel_limit × vel_limit_tolerance` |
| `INVALID_INPUT_MODE` | 输入模式设置不合法 |
| `UNSTABLE_GAIN` | 带宽过高导致控制器不稳定 |
| `INVALID_MIRROR_AXIS` | 镜像轴编号不存在 |
| `INVALID_LOAD_ENCODER` | 负载编码器轴编号不合法 |
| `INVALID_ESTIMATE` | 编码器位置/速度估计无效 |
| `INVALID_CIRCULAR_RANGE` | 编码器圆形位置估计无效 |
| `SPINOUT_DETECTED` | 打滑检测：机械功率与电气功率不匹配 |

### 2.2 输入与设定值

| 属性 | 类型 | 含义 |
|------|------|------|
| `input_pos` | float [turn] | 位置输入（位置控制的目标 / 调谐模式的直流偏移） |
| `input_vel` | float [turn/s] | 速度输入（速度控制的目标 / 位置控制的速度前馈） |
| `input_torque` | float [Nm] | 力矩输入（力矩控制的目标 / 其他模式的力矩前馈） |
| `pos_setpoint` | readonly float [turn] | 经过 InputMode 处理后的实际位置设定值 |
| `vel_setpoint` | readonly float [turn/s] | 经过 InputMode 处理后的实际速度设定值 |
| `torque_setpoint` | readonly float [Nm] | 经过 InputMode 处理后的实际力矩设定值 |
| `vel_integrator_torque` | float [Nm] | 速度环积分器累积值 |
| `trajectory_done` | readonly bool | 梯形轨迹运动是否完成 |
| `anticogging_valid` | bool | 抗齿槽标定数据是否有效 |

**`input_xxx` 和 `xxx_setpoint` 的区别**：`input_xxx` 是你写入的原始指令，`xxx_setpoint` 是经过 InputMode 处理（滤波、斜坡、轨迹规划等）后的实际设定值。

### 2.3 ControlMode（控制模式）

| 模式 | 使用的控制环 | 有效输入 |
|------|------------|---------|
| `VOLTAGE_CONTROL` | 仅电压 FOC（不常用，用 GIMBAL 电机类型替代） | — |
| `TORQUE_CONTROL` | 力矩环 | `input_torque` |
| `VELOCITY_CONTROL` | 力矩环 + 速度环 | `input_vel`、`input_torque`（前馈） |
| `POSITION_CONTROL` | 力矩环 + 速度环 + 位置环 | `input_pos`、`input_vel`（前馈）、`input_torque`（前馈） |

### 2.4 InputMode（输入模式）

| 模式 | 含义 | 配合的 ControlMode |
|------|------|-------------------|
| `INACTIVE` | 禁用输入，设定值保持最后值 | 所有 |
| `PASSTHROUGH` | `input_xxx` 直接透传到 `xxx_setpoint` | 所有 |
| `VEL_RAMP` | 速度斜坡（按 `vel_ramp_rate` 逐渐加速） | VELOCITY |
| `POS_FILTER` | 二阶位置跟踪滤波器（平滑 Step/Dir 输入） | POSITION |
| `TRAP_TRAJ` | 在线梯形轨迹规划 | POSITION |
| `TORQUE_RAMP` | 力矩斜坡（按 `torque_ramp_rate` 逐渐加力） | TORQUE |
| `MIRROR` | 电子镜像（跟随另一个轴的运动） | POSITION / VELOCITY / TORQUE |
| `TUNING` | 调谐模式（发送正弦波用于频率响应测试） | 所有 |
| `MIX_CHANNELS` | 未实现 | — |

### 2.5 配置参数

**核心增益**：

| 参数 | 默认 | 单位 | 含义 |
|------|------|------|------|
| `pos_gain` | 20.0 | (turn/s)/turn | 位置环 P 增益 |
| `vel_gain` | 1/6 | Nm/(turn/s) | 速度环 P 增益 |
| `vel_integrator_gain` | 2/6 | Nm/(turn/s·s) | 速度环 I 增益 |
| `vel_integrator_limit` | ∞ | Nm | 积分器输出钳位 |

**限幅与保护**：

| 参数 | 含义 |
|------|------|
| `vel_limit` | 速度上限 [turn/s]（默认 2，即 120 RPM） |
| `vel_limit_tolerance` | 超速容忍比例（默认 1.2 = 允许超 20%） |
| `enable_overspeed_error` | 是否使能超速报错 |
| `enable_vel_limit` | 是否使能速度限幅 |
| `enable_torque_mode_vel_limit` | 力矩模式下是否限速 |

**斜坡与滤波**：

| 参数 | 含义 |
|------|------|
| `vel_ramp_rate` | VEL_RAMP 模式的加速率 [turn/s²] |
| `torque_ramp_rate` | TORQUE_RAMP 模式的力矩变化率 [Nm/s] |
| `input_filter_bandwidth` | POS_FILTER 模式的滤波带宽 [rad/s] |
| `inertia` | 负载惯量前馈 [Nm/(turn/s²)]，设 0 禁用 |

**特殊功能**：

| 参数 | 含义 |
|------|------|
| `enable_gain_scheduling` | V 形增益调度（anti-hunt），误差小时降低增益防抖 |
| `gain_scheduling_width` | V 形调度的半宽 |
| `circular_setpoints` | 圆形位置模式（适用于连续旋转场景） |
| `circular_setpoint_range` | 圆形位置范围 [turn] |
| `homing_speed` | 归位速度 [turn/s] |
| `axis_to_mirror` | MIRROR 模式下跟随的轴编号 |
| `mirror_ratio` | 镜像比例（负值 = 反向） |
| `load_encoder_axis` | 使用另一个轴的编码器作为负载反馈 |

**抗齿槽 (anticogging)**：

| 参数 | 含义 |
|------|------|
| `anticogging.pre_calibrated` | 标定数据是否预存 |
| `anticogging.anticogging_enabled` | 是否使能抗齿槽补偿 |
| `anticogging.calib_pos_threshold` | 标定时位置收敛阈值 |
| `anticogging.calib_vel_threshold` | 标定时速度收敛阈值 |

**Spinout 检测**：

| 参数 | 含义 |
|------|------|
| `mechanical_power_bandwidth` | 机械功率低通滤波带宽 [rad/s] |
| `electrical_power_bandwidth` | 电气功率低通滤波带宽 [rad/s] |
| `spinout_mechanical_power_threshold` | 机械功率阈值 [W]（应为负值） |
| `spinout_electrical_power_threshold` | 电气功率阈值 [W]（应为正值） |

### 2.6 运行时功率监测

| 属性 | 含义 |
|------|------|
| `mechanical_power` | 机械功率 = 力矩 × 角速度 [W] |
| `electrical_power` | 电气功率 = Vdq · Idq [W] |

### 2.7 函数

| 函数 | 含义 |
|------|------|
| `move_incremental(displacement, from_input_pos)` | 相对位移运动 |
| `start_anticogging_calibration()` | 开始 3600 点抗齿槽标定 |
| `remove_anticogging_bias()` | 去除标定数据中的直流偏置 |
| `get_anticogging_value(index)` | 读取某个位置的补偿力矩值 |

---

## 三、ODrive.Motor 接口

Motor 接口涵盖**电机驱动的底层**：FOC 电流控制、电机参数、校准、温度保护。

### 3.1 错误标志

| 错误 | 含义 | 常见原因 |
|------|------|---------|
| `PHASE_RESISTANCE_OUT_OF_RANGE` | 校准电阻值超出合理范围 | 电机没接好或不匹配 |
| `PHASE_INDUCTANCE_OUT_OF_RANGE` | 校准电感值超出合理范围 | 同上 |
| `CURRENT_LIMIT_VIOLATION` | 电流超过 `current_lim + current_lim_margin` | 过载、短路 |
| `MODULATION_MAGNITUDE` | SVM 调制率超限 | 高速时反电动势接近母线电压 |
| `UNKNOWN_PHASE_ESTIMATE` | FOC 没有有效的相位估计 | 编码器/观测器没就绪 |
| `UNKNOWN_TORQUE` | 没有有效的力矩输入 | 上游 Controller 未就绪 |
| `UNKNOWN_CURRENT_COMMAND` | 电流设定值无效 | Controller 配置问题 |
| `UNKNOWN_CURRENT_MEASUREMENT` | 电流采样无效 | ADC 硬件问题 |
| `UNKNOWN_VBUS_VOLTAGE` | 母线电压无效 | ADC 未启动 |
| `UNKNOWN_VOLTAGE_COMMAND` | 电压前馈设定无效 | — |
| `UNKNOWN_GAINS` | PI 增益未配置 | 没有执行电机校准 |
| `CONTROLLER_INITIALIZING` | 控制器初始化中 | 内部状态，等待完成 |
| `UNBALANCED_PHASES` | 三相电流不平衡 | 某一相断线 |
| `DRV_FAULT` | DRV8301 栅极驱动故障 | 硬件过流/过温 |
| `BRAKE_RESISTOR_DISARMED` | 制动电阻未就绪 | — |

### 3.2 运行时状态

| 属性 | 类型 | 含义 |
|------|------|------|
| `is_armed` | bool | PWM 是否在输出 |
| `is_calibrated` | bool | 电阻/电感是否已校准 |
| `current_meas_phA/B/C` | float [A] | 三相实测电流 |
| `DC_calib_phA/B/C` | float [A] | 三相电流采样的直流偏置 |
| `I_bus` | float [A] | 直流母线电流（≈ 电源电流） |
| `effective_current_lim` | float [A] | 综合温度保护后的实际电流上限 |
| `max_allowed_current` | float [A] | 硬件能测量的最大电流 |
| `fet_thermistor` | 子接口 | FET 温度传感器 |
| `motor_thermistor` | 子接口 | 电机温度传感器 |

### 3.3 current_control 子接口（FOC 内部状态）

| 属性 | 含义 |
|------|------|
| `p_gain` / `i_gain` | 电流 PI 控制器增益（由 `current_control_bandwidth` 自动计算） |
| `Id_setpoint` / `Iq_setpoint` | d/q 轴电流设定值 [A] |
| `Id_measured` / `Iq_measured` | d/q 轴实测电流 [A]。**Iq 是产生力矩的电流**：`torque ≈ torque_constant × Iq` |
| `Vd_setpoint` / `Vq_setpoint` | d/q 轴电压指令 [V] |
| `Ialpha_measured` / `Ibeta_measured` | αβ 坐标系电流（Clarke 变换后） |
| `phase` / `phase_vel` | 当前电角度 [rad] 和电角速度 [rad/s] |
| `power` | 电气功率 [W] |
| `v_current_control_integral_d/q` | PI 积分器累积电压 |
| `final_v_alpha` / `final_v_beta` | 最终输出的 αβ 电压指令 |

### 3.4 配置参数

**电机物理参数**：

| 参数 | 含义 |
|------|------|
| `pole_pairs` | 极对数 = 磁铁数 ÷ 2 |
| `phase_resistance` | 相电阻 [Ω]，校准自动测量 |
| `phase_inductance` | 相电感 [H]，校准自动测量 |
| `torque_constant` | 力矩常数 [Nm/A]，**力矩 = torque_constant × Iq** |
| `motor_type` | 电机类型（HIGH_CURRENT / GIMBAL / ACIM） |
| `pre_calibrated` | 设为 True 则上电不再重新校准 |

**电流保护**：

| 参数 | 含义 |
|------|------|
| `current_lim` | 最大允许相电流 [A] |
| `current_lim_margin` | 超出 `current_lim` 多少就报错 [A] |
| `torque_lim` | 力矩上限 [Nm] |
| `requested_current_range` | 电流采样量程（影响 ADC 增益） |
| `I_bus_hard_min` / `I_bus_hard_max` | 母线电流硬限制 [A] |
| `I_leak_max` | 三相电流之和的最大允许值 [A]（检测漏电） |

**控制参数**：

| 参数 | 含义 |
|------|------|
| `current_control_bandwidth` | 电流环带宽 [rad/s]。自动推导 PI 增益：`Kp = L × bw`，`Ki = R × bw` |
| `calibration_current` | 校准时使用的电流 [A] |
| `resistance_calib_max_voltage` | 校准最大电压（< 0.5 × Vbus） |
| `R_wL_FF_enable` | 使能 RωL 前馈（改善高速电流跟踪） |
| `bEMF_FF_enable` | 使能反电动势前馈 |

**ACIM 专用**（异步电机，不用 ACIM 可忽略）：

| 参数 | 含义 |
|------|------|
| `acim_gain_min_flux` | 最小磁链增益 |
| `acim_autoflux_enable` | 自动磁链控制 |
| `acim_autoflux_min_Id` | 最小励磁电流 |

**关键关系**：`current_control_bandwidth` 是你唯一需要调的电流环参数。设好带宽后 PI 增益自动计算，不需要手动调 `p_gain` / `i_gain`。

---

## 四、ODrive.Encoder 接口

Encoder 接口处理位置和速度反馈，支持多种编码器类型，通过 PLL 输出平滑的位置/速度估计。

### 4.1 错误标志

| 错误 | 含义 | 常见原因 |
|------|------|---------|
| `CPR_POLEPAIRS_MISMATCH` | CPR 和极对数不匹配 | CPR 填错（PPR×4=CPR）、齿槽力矩干扰校准 |
| `NO_RESPONSE` | 编码器无响应 | 接线问题 |
| `ILLEGAL_HALL_STATE` | 霍尔状态非法（8 种组合中有 2 种无效） | 噪声或硬件故障，加 22nF 滤波电容 |
| `ABS_SPI_TIMEOUT` | SPI 绝对值编码器超时 | SPI 接线问题 |
| `ABS_SPI_COM_FAIL` | SPI 通信失败 | SPI 接线或 CS 引脚配置 |
| `ABS_SPI_NOT_READY` | SPI 编码器未就绪 | — |
| `UNSTABLE_GAIN` | PLL 增益不稳定 | `bandwidth` 设太高 |
| `UNSUPPORTED_ENCODER_MODE` | 编码器模式不支持 | — |
| `INDEX_NOT_FOUND_YET` | 尚未找到 Z 索引脉冲 | 编码器没有 Z 脉冲输出 |
| `HALL_NOT_CALIBRATED_YET` | 霍尔未校准 | — |

### 4.2 编码器模式 (Encoder.Mode)

| 模式 | 值 | 说明 |
|------|-----|------|
| `INCREMENTAL` | 0 | 增量编码器（A/B/Z 正交脉冲） |
| `HALL` | 1 | 霍尔传感器（3 线，6 状态） |
| `SINCOS` | 2 | 正弦余弦模拟编码器 |
| `SPI_ABS_CUI` | 0x100 | CUI AMT23xx 系列 |
| `SPI_ABS_AMS` | 0x101 | AMS AS5047P / AS5048A/B |
| `SPI_ABS_AEAT` | 0x102 | AEAT-8800 |
| `SPI_ABS_RLS` | 0x103 | RLS Orbis |
| `SPI_ABS_MA732` | 0x104 | MagAlpha MA732 磁编码器 |

### 4.3 运行时状态

| 属性 | 含义 |
|------|------|
| `pos_estimate` | **线性位置估计** [turns]，多圈累积，Controller 位置环的输入 |
| `pos_circular` | **单圈位置** [0, 1)，用于圆形位置控制 |
| `vel_estimate` | **速度估计** [turn/s]，PLL 输出，Controller 速度环的输入 |
| `shadow_count` | 原始线性计数 [counts] |
| `count_in_cpr` | 圈内计数 [0, cpr) |
| `phase` | 电角度（给 FOC 用的） |
| `hall_state` | 霍尔状态寄存器值 |
| `index_found` | 是否找到了 Z 索引脉冲 |
| `pos_abs` | 绝对值编码器的原始位置 |
| `spi_error_rate` | SPI 通信错误率 |
| `is_ready` | 编码器是否就绪（校准完成） |

**核心数据流**：`shadow_count → PLL → pos_estimate + vel_estimate → 输出给 Controller`

### 4.4 配置参数

| 参数 | 含义 |
|------|------|
| `mode` | 编码器类型（见 4.2） |
| `cpr` | **每转计数数**。增量编码器 = PPR × 4；绝对值编码器 = 分辨率 |
| `bandwidth` | PLL 带宽 [rad/s]，决定估计值的滤波程度 |
| `use_index` | 使用 Z 索引脉冲（提高精度） |
| `index_offset` | 找到 Z 脉冲后将位置设为此值 |
| `use_index_offset` | 是否启用 index_offset |
| `pre_calibrated` | 已预校准（跳过开机校准） |
| `enable_phase_interpolation` | 使能相位插值（提高低速平滑性） |
| `phase_offset` | 电角度偏移（校准得到） |
| `direction` | 编码器方向 |
| `calib_scan_distance` | 校准时电机转过的电角度距离 [rad]，默认 16π |
| `calib_scan_omega` | 校准时的电角速度 [rad/s]，默认 4π |
| `calib_range` | 校准误差阈值 [turn] |
| `abs_spi_cs_gpio_pin` | 绝对值 SPI 编码器的 CS 引脚 |
| `ignore_illegal_hall_state` | 忽略非法霍尔状态错误 |
| `sincos_gpio_pin_sin` / `sincos_gpio_pin_cos` | sin/cos 编码器引脚 |

### 4.5 函数

| 函数 | 含义 |
|------|------|
| `set_linear_count(count)` | 手动设置当前位置计数值。归位完成后调用此函数建立绝对位置参考 |

---

## 五、四大接口的连接关系

```
                    ┌──────────────────────────────────────────┐
                    │                  Axis                     │
                    │          (状态机协调所有子组件)            │
                    └──────────────────────────────────────────┘
                                       │
           ┌───────────────────────────┼───────────────────────────┐
           ▼                           ▼                           ▼
      ┌─────────┐              ┌─────────────┐              ┌──────────┐
      │ Encoder │              │ Controller  │              │  Motor   │
      │         │──pos_est───► │             │──torque────► │  (FOC)   │
      │         │──vel_est───► │  位置环(P)  │              │          │
      │         │──phase─────┐ │  速度环(PI) │              │ Clarke   │
      └─────────┘            │ └─────────────┘              │ Park     │
                             │                              │ PI 电流  │
                             └──────────────────────────────► SVM     │
                                  (电角度给 FOC)             └──────────┘
```

这三条信号线在 `axis.cpp: start_closed_loop_control()` 中通过 `connect_to()` 动态连接。

---

## 六、阅读 odrive-interface.yaml 问答记录

> 以下问答产生于实际阅读 `Firmware/odrive-interface.yaml` 的过程中。

---

### 第一部分：Axis 接口（Q1 ~ Q15）

---

#### Q1：dictionary: [ODrive] 说明了什么？

这是给**代码生成器**用的"词典"。生成器会自动把驼峰命名拆成单词（如 `AxisState` → `Axis` + `State`），用于生成 Python 风格的下划线命名（`axis_state`）。`ODrive` 如果不加词典会被拆成 `O` + `Drive` → `o_drive`，加了词典后保持 `odrive` 整体。纯粹是命名规范问题，不影响功能。

---

#### Q2：这个文件的作用是什么？

这个文件定义了固件暴露给外部（Python / CAN / GUI）的**所有**可读写属性和可调用方法。它是"单一真理源"——改这个文件，C++ 接口、Python 枚举、CAN 数据库全部自动重新生成。

---

#### Q3：userdata 和 Iph_ABC_t 是什么？

`userdata.c_preamble` 是在生成 C++ 头文件时插入的前置代码：

```cpp
using float2D = std::pair<float, float>;        // 二维浮点对（如 Idq）
struct Iph_ABC_t { float phA; float phB; float phC; };  // 三相电流采样值
```

`Iph_ABC_t` 就是三相电流（phase A / B / C），在 ADC 采样回调中使用。

---

#### Q4：c_is_class 是什么？

`c_is_class: True` 告诉代码生成器：这个 interface 对应 C++ 中的一个 `class`（而不是全局函数或 namespace）。生成的代码会用 `this->` 访问成员。

---

#### Q5：nullflag: NONE 是什么意思？

**定义一个值为 0 的标志位叫 `NONE`，表示"没有错误"**。

```cpp
enum AxisError {
    AXIS_ERROR_NONE = 0x00,          // ← nullflag
    AXIS_ERROR_INVALID_STATE = 0x01,
    AXIS_ERROR_MOTOR_FAILED = 0x40,
    ...
};
```

`flags` 类型是位掩码，可以同时存在多个错误（`error |= new_error`）。

---

#### Q6：ENDSTOP、ESTOP、homing 是什么？

| 术语 | 含义 |
|------|------|
| **Endstop**（限位开关） | 物理开关，安装在运动行程两端，碰到后触发停止 |
| **E-Stop**（紧急停止） | Emergency Stop，一键让电机立刻停转并 disarm |
| **Homing**（归位/回零） | 电机朝 MIN_ENDSTOP 慢速运动，碰到后把当前位置设为 0 |

```
运动行程示意：
  MIN_ENDSTOP ◄────────────────────► MAX_ENDSTOP
       │                                    │
       └─ Homing 时电机朝这里走              │
          碰到后 encoder 清零                │
                                            └─ 碰到后触发保护停止
```

---

#### Q7：step_dir_active 是什么？

**Step/Dir 脉冲模式**的激活标志。ODrive 可以接收外部 Step/Dir 信号（与步进电机驱动器相同的接口），每收到一个脉冲电机走一步。

应用场景：用 CNC 控制板（GRBL）驱动 ODrive、用 BLDC 伺服替代步进电机、3D 打印机主板控制等。

---

#### Q8：last_drv_fault 是什么？

**DRV8301 栅极驱动芯片**上次报告的故障码（不是"上一轮控制周期"）。DRV8301 有自己的故障检测（过流、过温、欠压等），通过 SPI 读取故障寄存器。

---

#### Q9：steps 是什么？

Step/Dir 模式下的累计脉冲计数。每收到一个 Step 脉冲，`steps` 加 1 或减 1（取决于 Dir 引脚电平）。Controller 用 `steps / steps_per_circular_range` 换算成位置设定值。

---

#### Q10：状态枚举在哪看？有哪些状态？

见本文 1.3 节的 AxisState 状态枚举表。

典型流程：`IDLE → FULL_CALIBRATION_SEQUENCE → CLOSED_LOOP_CONTROL`

---

#### Q11：is_homed 标志

表示该轴是否已完成归位（Homing）。只有执行过 `AXIS_STATE_HOMING` 并成功碰到限位开关后才会置 `True`。归位后编码器位置被清零，`pos_estimate` 才有绝对物理意义。

---

#### Q12：config 类的作用

`Axis.config` 包含这个轴所有需要在运行前配置的参数。可以通过 Python 修改并保存到 Flash：

```python
odrv0.axis0.config.enable_step_dir = True
odrv0.axis0.config.startup_closed_loop_control = True
odrv0.save_configuration()  # 写入 Flash
```

---

#### Q13：step_gpio_pin 和 dir_gpio_pin 的用途

MCU 上用于接收外部 Step/Dir 脉冲信号的 GPIO 引脚编号。ODrive 要兼容传统 CNC/3D 打印的控制方式——上位机发 Step/Dir 脉冲，ODrive 接收并转换为位置指令驱动 BLDC 电机。

---

#### Q14：LockinConfig 是什么？

**锁相旋转配置**。用于两个场景：

1. **无传感器启动**：先开环"强拖"电机旋转到一定速度，让反电动势足够大，观测器才能锁定相位
2. **编码器校准**：校准偏移时需要让电机开环旋转

参数包括：锁相电流、加速时间、目标速度、结束条件（达到速度 / 转够距离 / 找到 Index 脉冲）等。

---

#### Q15：还需要重点看哪些接口？

按学习优先级排序：

| 优先级 | 接口 | 原因 |
|--------|------|------|
| ★★★★★ | `ODrive.Controller` | 位置/速度/力矩控制参数，最常调 |
| ★★★★★ | `ODrive.Motor` | 电机物理参数 |
| ★★★★★ | `ODrive.Encoder` | 编码器配置 |
| ★★★★ | `ODrive.Axis` | 状态机、启动配置 |
| ★★★ | `ODrive.TrapezoidalTrajectory` | 轨迹规划参数 |
| ★★★ | `ODrive.SensorlessEstimator` | 无传感器观测器 |
| ★★ | `ODrive.Config` | 顶层配置（母线电压等） |
| ★ | `ODrive.AcimEstimator` | 异步电机（可跳过） |

---

### 第二部分：Controller 接口（Q16 ~ Q24）

---

#### Q16：UNSTABLE_GAIN 是如何判断带宽过高的？

电流环 PI 控制器的增益由 `motor.config.current_control_bandwidth` 推导：

```
Kp = bandwidth × L（电感）
Ki = bandwidth × R（电阻）
```

当 `bandwidth` 设得过高时，`Kp` 会大到让控制器不稳定（采样频率 ~8kHz，奈奎斯特极限 ~4kHz）。固件检查：如果计算出的增益会导致离散化后的极点在单位圆外，就报 `UNSTABLE_GAIN`。

简单理解：**带宽不能超过控制频率的一半左右，否则系统会震荡**。

---

#### Q17：INVALID_MIRROR_AXIS / 镜像轴是什么概念？

**镜像模式**：让一个轴**跟随另一个轴的运动**。

```
Axis 0（主轴）       Axis 1（从轴，镜像模式）
  转 10 圈    ──►     自动跟着转 10 圈
  速度 5 r/s  ──►     自动跟着 5 r/s
```

应用场景：
- **双轴同步**：双轮平衡车、对称机械臂
- **电子齿轮**：`mirror_ratio = 2.0` 表示从轴转速是主轴的 2 倍
- **反向镜像**：`mirror_ratio = -1.0` 表示方向相反

`INVALID_MIRROR_AXIS` 就是设了不存在的轴编号（ODrive v3 只有 axis0 和 axis1）。

---

#### Q18：SPINOUT_DETECTED 是什么？

**打滑检测**。固件同时监测两个功率：

```
机械功率 = 力矩 × 转速        （电机实际输出多少功）
电气功率 = Vdq · Idq           （电机实际消耗多少电）

正常：两者基本匹配
  机械功率 ≈ 电气功率 - 铜损

异常（Spinout）：
  电气功率很高（电机在费劲地推）
  机械功率很低或为负（但电机没有真的在转）

  原因：编码器打滑了！
  → FOC 施加的电流方向错了
  → 产生制动力矩而不是驱动力矩
  → 恶性循环
```

常见触发原因：编码器联轴器松动、偏移校准不准、Index 脉冲受噪声干扰。

---

#### Q19：desired output 和 feed-forward 参数的区别？

同一个变量在不同控制模式下含义不同：

```
以 input_vel 为例：

速度控制模式：input_vel = 目标速度（desired output）
  "请把速度控制到 5 r/s"

位置控制模式：input_vel = 速度前馈（feed-forward）
  "除了位置环算出来的速度指令，再额外叠加这个速度"
```

**前馈的价值**：如果你知道轨迹规划的速度，直接告诉速度环，它就不需要等误差积累再反应，**响应更快、跟踪更准**。

```
无前馈：  位置环 → 位置误差 → 速度指令 → 速度环 → 力矩

有前馈：  位置环 → 位置误差 → 速度指令 ─┐
          轨迹规划 → 速度前馈 ──────────┘→ 相加 → 速度环 → 力矩
```

---

#### Q20：什么是 gain_scheduling 模式？

**增益调度**：根据位置误差大小动态缩放控制增益。ODrive 用 **V 形调度**：

```
增益倍率
  1.0 ──────────╱╲──────────
               ╱  ╲
              ╱    ╲
  0.0 ──────╱──────╲──────
          -W    0    +W       ← 位置误差
          W = gain_scheduling_width
```

- 误差大（远离目标）→ 增益 100% → 快速追赶
- 误差小（接近目标）→ 增益降低 → 避免到位震荡

也叫 **anti-hunt**（防抖动）。适合高精度定位。

---

#### Q21：inertia 是什么？

**不是电机本身的转动惯量**，而是你告诉控制器的**负载等效惯量**，用于**前馈补偿**。

```
单位：N·m / (turn/s²)
用途：控制器根据加速度和惯量计算前馈力矩
     torque_feedforward = acceleration × inertia
设为 0 = 不用前馈（默认）
```

不设也能工作，只是靠 PID 反馈补偿加速度，响应稍慢。

---

#### Q22：anticogging 介绍

**齿槽力矩补偿**。PMSM 电机由于永磁体和定子齿槽的几何结构，低速运行时会有"一格一格"的顿挫感。

ODrive 的方案：

```
校准阶段（离线，一次性）：
  1. 让电机极慢速转一整圈
  2. 每 0.1° 记录维持匀速需要的补偿力矩
  3. 共 3600 个点存入 cogging_map[3600]

运行阶段（在线，每个控制周期）：
  torque_total = torque_from_PID + cogging_map[position_index]
```

效果：低速运行**速度明显更平滑**。

---

#### Q23：mechanical power 和 electrical power

```
electrical power = Vd×Id + Vq×Iq      （电机消耗的总电功率）
mechanical power = torque × velocity    （电机轴输出的机械功）

正常：mechanical ≈ electrical - 铜损
异常（Spinout）：electrical > 0 但 mechanical < 0
  → 说明编码器打滑，电流方向搞反
```

两个值都经过低通滤波（`bandwidth` 控制），避免瞬态波动误触发 Spinout 检测。

---

#### Q24：最后的 4 个 function 是什么？

这不是"最终要调用的接口"，而是 Controller 对外暴露的**辅助方法**：

- `move_incremental`：相对位移（"从当前位置再走 2 圈"）
- `start_anticogging_calibration`：触发抗齿槽标定
- `remove_anticogging_bias`：去除标定数据的直流偏置
- `get_anticogging_value`：读取某位置的补偿值

日常使用中，最常用的是**直接写属性**：

```python
odrv0.axis0.controller.input_pos = 10        # 设位置
odrv0.axis0.controller.input_vel = 5         # 设速度
odrv0.axis0.controller.config.vel_gain = 0.2 # 改增益
```

**真正的控制循环**（`Controller::update()`）是固件内部以 8kHz 自动调用的，不需要从外部调用。

---

### 第三部分：Motor 和 Encoder 接口

> 以下是对 Motor 和 Encoder 接口的精要介绍，筛选出学习者最需要了解的要点。

---

#### Q25：Motor 接口中最重要的配置参数是哪些？

| 参数 | 为什么重要 |
|------|-----------|
| `pole_pairs` | 极对数决定电角度和机械角度的换算，填错则 FOC 完全失效 |
| `torque_constant` | 决定力矩和电流的换算关系，直接影响力矩控制精度 |
| `current_lim` | 保护电机和驱动器不过流烧毁 |
| `current_control_bandwidth` | 电流环唯一需要调的参数，PI 增益自动推导 |

`phase_resistance` 和 `phase_inductance` 通常由电机校准自动测量，设 `pre_calibrated = True` 后可跳过。

---

#### Q26：current_control 子接口怎么理解？

这是 **FOC 电流环的内部状态**，平时不需要配置，主要用于调试和监控：

- **`Iq_measured`** 是产生力矩的电流，`torque ≈ torque_constant × Iq`
- **`Id_measured`** 在表贴式永磁电机中应接近 0（没有弱磁需求时）
- **`power`** 是实时电气功率
- **`p_gain` / `i_gain`** 由 `current_control_bandwidth` 自动算出

---

#### Q27：Encoder 的 pos_estimate 和 pos_circular 有什么区别？

| | `pos_estimate` | `pos_circular` |
|---|---|---|
| **范围** | -∞ ~ +∞（多圈累积） | [0, 1)（单圈） |
| **用途** | 线性定位（如 "移到第 10 圈"） | 连续旋转（如 "保持 0.5 圈位置"） |
| **别名** | multi-turn position | single-turn position |

Controller 的 `pos_estimate_linear_src_` 连到 `pos_estimate`，`pos_estimate_circular_src_` 连到 `pos_circular`。

---

#### Q28：bandwidth 参数对 Encoder 有什么影响？

`bandwidth` 控制 PLL（锁相环）的带宽：

- **设高**：`pos_estimate` 和 `vel_estimate` 响应快、延迟小，但噪声大
- **设低**：估计值平滑，但响应慢、延迟大

典型值：1000~2000 rad/s。需要在**噪声抑制**和**跟踪延迟**之间取平衡。

---

#### Q29：Motor 和 Encoder 之间的数据流是怎样的？

```
Encoder ──pos_estimate──► Controller ──torque_output──► Motor
         ──vel_estimate──►                              ▲
         ──phase────────────────────────────────────────┘
                                                    (FOC 需要电角度)
```

- **Encoder → Controller**：`pos_estimate` 和 `vel_estimate` 是位置环/速度环的反馈
- **Encoder → Motor(FOC)**：`phase` 是电角度，Park/逆 Park 变换需要
- **Controller → Motor**：`torque_output` 转成 `Iq_setpoint`，驱动 FOC 电流环

这三条线在 `axis.cpp: start_closed_loop_control()` 中通过 `connect_to()` 动态连接。

---

下一步 → [06-phase5-controller.md](06-phase5-controller.md)（上层控制器与状态机）

上一步 ← [A2-qa-record.md](A2-qa-record.md)（学习问答记录）
