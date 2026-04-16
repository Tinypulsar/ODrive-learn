# 阶段二 · 硬件抽象层与底层驱动（第 3-4 周）

> **目标**：理解 STM32 外设如何被 ODrive 封装使用——PWM 生成、电流采样、栅极驱动芯片通信、安全机制。这是整栋楼的"地基"。

---

## 2.1 学习目标

完成本阶段后，你将能够：
- 说清楚 PWM / ADC / SPI 在 ODrive 中的配置与用途
- 理解 **PWM 中心对齐 + ADC 同步采样** 的原理
- 看懂 DRV8301 栅极驱动芯片的 SPI 配置流程
- 识别代码中的"安全关键"路径（`safety_critical_*` 系列函数）
- 理解 STM32 HAL 被 C++ 封装后的结构

---

## 2.2 前置理论

### PWM 中心对齐模式
STM32 的高级定时器（TIM1/TIM8）支持三种对齐：
- **边沿对齐（Edge-aligned）**：计数器从 0 递增到 ARR，比较值决定占空比
- **中心对齐（Center-aligned）**：计数器 0 → ARR → 0 循环，生成**上下对称**的 PWM

ODrive 用的是**中心对齐**，因为：
1. 三相对称性好（减少共模电压）
2. **ADC 可以在 PWM 的中心（即转换器开关的"中点"）采样电流**——此时采样的是最接近平均相电流的值

### PWM 同步 ADC 采样
关键时序：
```
TIM1 (PWM 计数)：
      ARR  ┌──┐              ┌──┐
           │  │              │  │
           │  │   Valley     │  │
       0  ─┴──┴────────▲──────┴──┴─
                       │
                       触发 ADC 采样 (ADC trigger from TIM1 TRGO event)
                       └── 此时下桥臂导通（所有 MOSFET 开通），电流流过分流电阻最稳定
```

- 在 PWM **波谷**（所有下桥臂都开）或 **波峰**（所有上桥臂都开）时采样电流
- 避开开关瞬态（MOSFET 切换时电流振铃）
- ODrive 用 DRV8301 的电流反馈接到 ADC2 / ADC3

### 分流电阻电流采样
```
       VM (母线)
        │
       ┌┴┐
       │ │ (高侧 MOSFET)
       └┬┘
        ├──────► 电机相线
       ┌┴┐
       │ │ (低侧 MOSFET)
       └┬┘
        │
       ┌┴┐
       │ │ (分流电阻 Rshunt，约 0.5 mΩ)
       └┬┘
        │
       GND
         │
         └──► 运放 → ADC 测量压降 → 换算成电流
```

ODrive v3.x 使用 **低侧分流（low-side shunt）** + DRV8301 内置的电流感应放大器。

### SPI 协议简要
- 4 线同步串行：CLK（时钟）、MOSI（主出从入）、MISO（主入从出）、CS（片选）
- 主从模式，全双工
- DRV8301 有 11-bit 寄存器，通过 SPI 读写配置（增益、工作模式等）
- 绝对值编码器（AS5047 / MA732 等）也通过 SPI 读取位置

---

## 2.3 阅读顺序

### 第 1-2 天：PWM 与底层安全路径

**源文件**：`Firmware/MotorControl/low_level.cpp` / `low_level.h`

这个文件很大（1000+ 行），分段阅读：

1. **全局变量**（文件顶部）：
   ```cpp
   float vbus_voltage = 12.0f;          // 母线电压
   float ibus_ = 0.0f;                  // 母线电流
   bool brake_resistor_armed = false;   // 制动电阻启用状态
   ```

2. **Safety Critical 段**（搜索 `Safety critical functions`）：
   - `safety_critical_arm_brake_resistor()` / `safety_critical_disarm_brake_resistor()`
   - `safety_critical_apply_brake_resistor_timings()`
   - 这些函数**直接操作硬件寄存器**（`htim2.Instance->CCR3 = ...`），任何错误会导致硬件损坏
   - 使用 `CRITICAL_SECTION()` 宏保护临界区

3. **ADC 回调**（搜索 `pwm_trig_adc_cb`）：
   - 由 PWM 触发的 ADC 中断
   - 读取三相电流 + 母线电压
   - 调用对应 Motor 的 `current_meas_cb()` / `dc_calib_cb()`

4. **PWM 更新回调**（搜索 `update_brake_current` / `pwm_update_cb`）：
   - 计算制动电阻占空比
   - 在 PWM 下一个周期应用

**必读要点**：
- 文件开头的"Safety critical assumptions"注释——列出了所有安全关键的硬件寄存器
- 为什么需要"Safety Critical"？因为一个 bit 写错可能烧毁 MOSFET 或电机

### 第 3 天：栅极驱动 DRV8301

**源文件**：
- `Firmware/Drivers/DRV8301/drv8301.hpp` - 接口
- `Firmware/Drivers/DRV8301/drv8301.cpp` - 实现

关注：

1. **配置寄存器**：
   - Register 1：控制模式、栅极驱动电流
   - Register 2：OCP（过流保护）阈值
   - Register 3：电流放大器增益（`GAIN_10` / `GAIN_20` / `GAIN_40` / `GAIN_80` V/V）
   - Register 4：开漏 nFAULT 设置

2. **关键方法**：
   - `config()`：根据请求的电流量程自动选择增益
   - `init()`：SPI 初始化 + 寄存器写入
   - `check_for_errors()`：读取 nFAULT 引脚 + 寄存器错误位

3. **SPI 交互**：
   - 读：先发送 `(1 << 15) | addr`，第二次读数据
   - 写：发送 `addr | data`
   - 通过 `Stm32SpiArbiter` 仲裁（M0 和 M1 共用 SPI3）

### 第 4 天：STM32 外设 C++ 封装

**源文件**：`Firmware/Drivers/STM32/`

1. **`stm32_gpio.cpp/.hpp`**：
   - `Stm32Gpio` 类：对 HAL 的 `GPIO_TypeDef*` + 引脚号做封装
   - 方法：`config()` / `read()` / `write()` / `subscribe()`（外部中断订阅）
   - **关注**：它如何管理中断共享（多个外设注册同一个 EXTI）

2. **`stm32_timer.hpp`**：
   - 定时器模式定义
   - PWM 输出、输入捕获封装

3. **`stm32_spi_arbiter.cpp/.hpp`**：
   - 多个 SPI 设备共享同一条 SPI 总线的仲裁器
   - 异步任务队列
   - DMA + 中断完成回调
   - **核心技术**：避免 M0 和 M1 的 DRV8301 同时占用 SPI3 时冲突

4. **`stm32_nvm.c/.h` / `stm32_nvm.cpp`**：
   - STM32 Flash 编程（用于保存配置）
   - 处理双扇区备份、CRC 校验、写保护

### 第 5 天：温度保护

**源文件**：
- `Firmware/MotorControl/thermistor.hpp`
- `Firmware/MotorControl/thermistor.cpp`

关注：

1. **热敏电阻模型**（NTC）：
   - 电阻-温度曲线用多项式拟合
   - `board.cpp` 中定义了 3 阶多项式系数（`fet_thermistor_poly_coeffs`）

2. **两类温度限流**：
   - `OnboardThermistorCurrentLimiter`：板载 FET 温度
   - `OffboardThermistorCurrentLimiter`：外部电机温度

3. **线性降额**：
   - `temp_limit_lower`（开始限流）
   - `temp_limit_upper`（完全限流到 0）
   - 线性映射

### 第 6-7 天：板级初始化

**源文件**：
- `Firmware/Board/v3/board.cpp`
- `Firmware/Board/v3/Inc/*.h`
- `Firmware/Board/v3/Src/*.c`（STM32CubeMX 生成）

关注：

1. **GPIO 初始化**：`gpio.c` - 引脚复用配置
2. **TIM 初始化**：`tim.c` - PWM 参数、编码器模式
3. **ADC 初始化**：`adc.c` - 通道配置、DMA
4. **SPI 初始化**：`spi.c`
5. **USB 初始化**：`usb_device.c`、`usbd_*.c` 系列

**不需要全部读懂** — STM32CubeMX 生成的代码很规整，知道在哪里查即可。

---

## 2.4 关键代码片段解析

### 片段 1：PWM 定时器配置（`board.cpp`）

```cpp
// 两个电机的 PWM 定时器
TIM_HandleTypeDef* motor_timers[] = {&htim1, &htim8};

// 定时器配置要点：
// - 中心对齐模式（TIM_COUNTERMODE_CENTERALIGNED3）
// - ARR = TIM_1_8_PERIOD_CLOCKS ≈ 10500，对应 ~8 kHz PWM
// - 死区时间 ≈ 120 ns
// - MOE（主输出使能）：由 safety_critical_arm_motor_pwm 设置
```

### 片段 2：电流采样回调（`low_level.cpp`）

```cpp
// 伪代码示意（实际代码更复杂）
void pwm_trig_adc_cb(ADC_HandleTypeDef* hadc, bool injected) {
    uint32_t timestamp = get_timestamp();
    
    // 读取 ADC 值
    int adc_phB = ADC2->JDR1;
    int adc_phC = ADC2->JDR2;
    int adc_vbus = ADC1->JDR1;
    
    // 换算成物理量
    Iph_ABC_t currents;
    currents.phB = (adc_phB - offset_b) * adc_to_amps;
    currents.phC = (adc_phC - offset_c) * adc_to_amps;
    currents.phA = -currents.phB - currents.phC;  // 基尔霍夫电流定律
    
    // 交给电机模块处理
    motors[axis_num].current_meas_cb(timestamp, currents);
}
```

**为什么 phA 要用 -phB - phC 计算？**
- 三相电流和为 0（星型接法）：`iA + iB + iC = 0`
- ODrive v3.x 只采样两相（B/C），节省 ADC
- v3.6-56V 支持全三相采样

### 片段 3：制动电阻控制

当电机**再生**（减速时反电动势充电回母线）时，母线电压会上升。如果没有地方消耗能量，会触发过压保护或损坏电容。

```cpp
// 伪代码
void update_brake_current(float dc_bus_current_to_dissipate) {
    // 转换为 PWM 占空比
    float duty = (dc_bus_current_to_dissipate * brake_resistance) / vbus_voltage;
    duty = clamp(duty, 0.0f, 1.0f);
    
    uint32_t low_off = duty * PERIOD;
    uint32_t high_on = (1.0f - duty) * PERIOD;
    
    safety_critical_apply_brake_resistor_timings(low_off, high_on);
}
```

**死区违规检测**：`if (high_on - low_off < TIM_APB1_DEADTIME_CLOCKS)` → 立即 disarm。

---

## 2.5 动手实验

**实验 2-1：数 ADC 触发时机**

阅读 `low_level.cpp` 中 ADC 配置代码，回答：
- PWM 触发 ADC 用的是哪个事件（TRGO / Update / CC4）？
- 一个 PWM 周期内 ADC 触发几次？
- "Injected" 和 "Regular" ADC 分别采什么？

**实验 2-2：DRV8301 配置流程**

跟踪 `Motor::setup()` 中对 `gate_driver_.config()` 的调用：
- 根据 `requested_current_range` 选择了哪个增益？
- OCP 阈值是多少？
- 如果把 `requested_current_range` 设成 200 A 会怎样？

**实验 2-3：模拟一个过压场景**

假设电机以 1000 rpm 速度被强制减速（机械拖动），估算：
- 反电动势大小（给定 Kv = 270 rpm/V）
- 母线电压会上升到多少（给定母线电容 500 μF，电机转动惯量 0.001 kg·m²）
- 制动电阻（2 Ω）需要多大占空比才能稳住 48 V 母线

**实验 2-4：画 PWM + ADC 时序图**

用时间轴画出：
- PWM 计数器的三角波
- PWM 输出电平
- ADC 触发时刻
- ADC 转换完成中断
- `current_meas_cb()` 被调用

---

## 2.6 常见困惑解答

**Q1：为什么需要 `safety_critical_` 前缀的函数？**

A：这些函数直接写硬件寄存器，一个错误就可能让 MOSFET 爆炸、母线电容爆炸、电机绕组烧毁。通过命名规范强制在 Code Review 时引起特殊注意。

**Q2：为什么不用 STM32 HAL 库提供的 `HAL_TIM_PWM_Start()` 等高层 API？**

A：HAL 库带有状态机和锁，开销大且不可预测；安全关键代码需要直接操作寄存器才能保证时序确定性。

**Q3：两个电机共用 SPI3 会不会冲突？**

A：`Stm32SpiArbiter` 把 SPI 请求排队串行化。栅极驱动配置只在启动时做一次，后续正常运行中 DRV8301 SPI 极少访问；绝对值编码器的 SPI 请求也经过仲裁器。

**Q4：ADC 采样精度够吗？**

A：12 位 ADC × 3.3V 量程 = 0.8 mV / LSB；对应分流 0.5 mΩ + 放大器 40 倍，约 40 mA / LSB。在 40A 量程下分辨率约 0.1%。

---

## 2.7 检验标志

- [ ] 能画出 PWM / ADC 时序图（电流何时采样）
- [ ] 能解释 "Safety Critical" 函数的意义
- [ ] 能说清 DRV8301 的几个关键寄存器作用
- [ ] 知道两个电机如何共享 SPI3
- [ ] 能解释为什么制动电阻需要 dead-time 检测
- [ ] 理解温度限流的线性降额策略

---

下一步 → [04-phase3-encoder.md：编码器与位置估计](04-phase3-encoder.md)

上一步 ← [02-phase1-foundation.md：基础理论](02-phase1-foundation.md)
