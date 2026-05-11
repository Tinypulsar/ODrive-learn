# 附录 A4 · 源码阅读问答记录

> 在阅读 ODrive 固件源码过程中产生的问题与解答汇总。

---

## 一、main.cpp 阅读笔记（Q34-Q40）

> 以下问题来自对 `Firmware/MotorControl/main.cpp` 的逐行阅读，包含原始理解和勘误。

---

### Q34：USB 序列号 —— 为什么正常模式和 DFU 模式要相同？

**原始理解**：知道代码在配置 USB 序列号，但不清楚意义。

**解答**：代码在 `main.cpp:644-658` 从芯片 96-bit Unique ID（`UID_BASE`）算出 64-bit 序列号，再转成 12 位十六进制字符串。

USB serial number 是操作系统**识别和区分 USB 设备**的关键标识。如果正常模式和 DFU 模式的序列号不一致：
- 操作系统会认为是**两个不同的设备**，分配不同的 COM 口或设备节点
- `odrivetool` 等工具无法自动关联"正在升级的设备"和"之前连接的设备"
- 多板环境下，用户无法确定哪块板在 DFU 模式

代码注释明确说明：序列号算法必须与 STM32 内置 USB bootloader 的算法一致，保证两种模式下序列号相同。

---

### Q35：HAL 初始化（system_init）

**原始理解**："通过 HAL 封装的初始化函数，对芯片的时钟和 OTP 烧录等进行了初始化"

**勘误**：`system_init()` 的实际代码在 `Board/v3/board.cpp:272-290`，做三件事：

1. **`HAL_Init()`** —— 初始化 HAL 库（SysTick 定时器、Flash 预取缓冲区），不包括 OTP
2. **`SystemClock_Config()`** —— 配置系统时钟树（HSE → PLL → 168MHz SYSCLK）
3. **OTP 读取（不是烧录）** —— OTP 是在生产时一次性写入的，启动时只是读取其中的硬件版本号进行校验。如果 OTP 为空白（`0xff`），则使用 RAM 中的 `fake_otp` 替代

**关键纠正**：OTP（One-Time Programmable）这里是**只读操作**，不是"烧录"。

---

### Q36：NVM 配置系统

**原始理解**："NVM 类似于电脑 PC 中的内存"

**重要纠正：NVM ≠ 电脑中的内存（RAM）**

| | NVM (Non-Volatile Memory) | 电脑内存 (RAM) |
|---|---|---|
| **掉电** | 数据保留 | 数据丢失 |
| **类比** | 更像硬盘/SSD | —— |
| **在 ODrive 中** | STM32 内部 Flash | STM32 SRAM |
| **用途** | 持久化存储用户配置 | 运行时变量 |

**具体实现**（`Drivers/STM32/stm32_nvm.c`）：
- 使用 STM32F405 Flash 的最后两个扇区（Sector 10 和 Sector 11，各 128KB）
- 采用 **A/B 双扇区交替写入**策略：写新数据到另一个扇区，再擦除旧扇区，保证掉电安全
- 配置数据带 **CRC16 校验**确保完整性

**启动加载流程**（`main.cpp:666-676`）：
1. `config_manager.start_load()` → 从 Flash 定位最新配置
2. `config_read_all()` → 按顺序读出所有组件的 config 结构体
3. `config_manager.finish_load()` → 验证 CRC
4. `config_apply_all()` → 将配置应用到各组件
5. 任何一步失败 → `config_clear_all()` 恢复出厂默认

**运行时行为**：用户通过 `odrv0.save_configuration()` 把 RAM 中的参数写到 Flash。如果改了参数但没 save，掉电后丢失，恢复为上次保存的值。

---

### Q37：CubeMX 用于外设初始化是否合理？

**原始理解**："如果我要做自己的硬件，这部分可能要基于 CubeMX 进行改变"

**解答**：思路可行，但需了解 ODrive 的实际情况：

ODrive 的 `MX_xxx_Init()` 函数虽然看起来像 CubeMX 生成的，但实际是**手写维护的**，集中在 `Board/v3/board.cpp`。ODrive 项目没有使用 CubeMX 项目文件（`.ioc`），因为自动生成的代码在精细控制（如 PWM+ADC 同步触发）方面不够灵活。

**自己开发硬件的三种路径**：

| 方法 | 优点 | 缺点 |
|------|------|------|
| CubeMX 生成 + 手工修改 | 初始化快速，可视化配置 | 重新生成后可能覆盖手工修改 |
| 纯手写 HAL 调用（ODrive 做法） | 完全控制，精确配置 | 门槛高，需要熟悉参考手册 |
| CubeMX 生成框架 + 迁移到手写 | 两者结合，推荐 | 需要理解生成代码含义 |

**建议**：先用 CubeMX 搭建框架确认引脚和时钟无冲突，然后把初始化代码提取出来手写维护。

---

### Q38：Board-specific GPIO 配置

**原始理解**："根据 Board 的板子配置，对驱动板的预留 GPIO 进行了相应的配置"

**理解正确**。补充关键点：

- `main.cpp:689-818` 根据 `odrv.config_.gpio_modes[]` 数组初始化每个 GPIO
- **`alternate_functions[]` 表**：预定义每个 GPIO 支持哪些复用功能，不是所有 GPIO 都支持所有模式
- **`misconfigured_` 标志**：用户设置不支持的模式时，设置标志而非崩溃（防御性设计）
- **设计理念**：GPIO 功能通过**运行时配置**（非编译时），用户可通过接口灵活改变引脚功能，重启后生效

---

### Q39：FreeRTOS 同步原语 —— semaphore、message queue、thread

**原始理解**："创建了关于 UART、USB、CAN 的子线程...有主线程和 semaphore，是否是对应的？"

**勘误**：信号量和线程不是"对应"关系，而是"**生产者-消费者**"关系。

`main.cpp:821-840` 中创建的 FreeRTOS 对象：

#### 1) `sem_usb_irq` — 二值信号量（Binary Semaphore）
```c
osSemaphoreDef(sem_usb_irq);
sem_usb_irq = osSemaphoreCreate(osSemaphore(sem_usb_irq), 1);
osSemaphoreWait(sem_usb_irq, 0);  // 拿走初始 token，变成"空"
```
- **作用**：USB 硬件中断 → USB 线程的同步
- **原理**：ISR 释放信号量，线程等待信号量后处理数据
- **只传递"有事件"信息**，数据在硬件缓冲区中

#### 2) `uart_event_queue` — 消息队列（深度 4）
```c
osMessageQDef(uart_event_queue, 4, uint32_t);
uart_event_queue = osMessageCreate(osMessageQ(uart_event_queue), NULL);
```
- **作用**：UART 中断 → UART 线程的同步 + 数据传递
- **为什么用队列**：需要传递具体事件类型（接收完成、错误等）

#### 3) `usb_event_queue` — 消息队列（深度 7）
- USB 协议栈的事件通知

#### 4) `sem_can` — 二值信号量
- CAN 中断 → CAN 线程的同步，与 `sem_usb_irq` 工作方式相同

#### 5) `defaultTask` (rtos_main) — 主线程
- `main()` 中创建的**唯一线程**，其他线程都在 `rtos_main()` 内部创建

**信号量 vs 消息队列的区别**：

| | 信号量 (Semaphore) | 消息队列 (Message Queue) |
|---|---|---|
| **携带数据** | 不携带，只有"有/无" | 携带 uint32_t 数据 |
| **适用场景** | 纯同步通知 | 需要传递事件类型或参数 |
| **ODrive 中** | USB/CAN 中断通知 | UART/USB 事件 |

**中断到线程的同步模型**：
```
硬件中断(ISR) ──释放信号量/发送消息──> 信号量/队列 ──等待──> FreeRTOS线程
```

#### ODrive 完整线程列表

| 线程 | 创建位置 | 优先级 | 功能 |
|------|---------|--------|------|
| defaultTask (rtos_main) | main.cpp:840 | Normal | 启动初始化，完成后自删除 |
| usb_thread | interface_usb.cpp:236 | Normal | USB 通信处理 |
| uart_thread | interface_uart.cpp:193 | Normal | UART 通信处理 |
| can_thread | odrive_can.cpp:40 | Normal | CAN 通信处理 |
| analog_thread | low_level.cpp:410 | Low | 模拟量轮询采集 |
| axis[0].thread | axis.cpp:99 | High | M0 状态机循环 |
| axis[1].thread | axis.cpp:99 | High | M1 状态机循环 |

**注意**：FOC 电流环**不是线程**，运行在定时器中断中（优先级高于所有线程），保证最低时序抖动。

---

### Q40：osKernelStart() 和 CMSIS-RTOS

**原始理解**："osKernelStart 在 cmsis_os.c 中定义，这是否是固定的？"

**解答**：是的，`osKernelStart()` 是 CMSIS-RTOS 标准 API。

**架构层次**：
```
应用代码 (main.cpp)
    ↓ 调用
CMSIS-RTOS API (cmsis_os.h / cmsis_os.c)    ← ARM 定义的抽象层
    ↓ 内部调用
FreeRTOS API (task.h / queue.h / semphr.h)   ← 实际 RTOS 实现
```

**CMSIS-RTOS 是 ARM 定义的 RTOS 抽象层标准**：
- `osKernelStart()` 内部调用 FreeRTOS 的 `vTaskStartScheduler()`
- `osThreadCreate()` 内部调用 `xTaskCreate()`
- `osSemaphoreCreate()` 内部调用 `xSemaphoreCreateBinary()`

**意义**：如果将来切换到 RT-Thread 或 ThreadX，只要它们实现了 CMSIS-RTOS 接口，应用代码不用改。

**调用后发生了什么**：调度器接管 CPU，`osKernelStart()` **永远不会返回**。后面的 `for(;;);` 只是安全保护，防止调度器启动失败后 `main()` 返回导致未定义行为。

**真正的应用入口是 `rtos_main()`**（`main.cpp:519-590`），依次完成：
1. USB 初始化
2. ADC 启动
3. 通信初始化（创建 USB/UART/CAN 线程）
4. 编码器 SPI CS 设置
5. 电机/编码器 setup
6. PWM + ADC 中断启动
7. 等待电流采样收敛（最多 2 秒）
8. 启动 Axis 状态机线程
9. **自删除**（`vTaskDelete(defaultTaskHandle)`）

---

## 二、main.cpp 关键架构总结

### 启动流程全景

```
上电
 ├─ early_start_checks()     [startup.s 调用，在 C++ 静态初始化之前]
 │   └─ 处理 DFU bootloader 跳转（_reboot_cookie 魔数）
 │
 ├─ C++ 全局对象构造          [odrv, config_manager 等]
 │
 └─ main()
     ├─ USB 序列号生成        [从 UID 算出，与 DFU 模式一致]
     ├─ system_init()          [HAL_Init + 时钟配置 + OTP 校验]
     ├─ NVM 配置加载           [Flash → RAM，CRC 校验]
     ├─ board_init()           [外设初始化：GPIO/DMA/ADC/TIM/SPI]
     ├─ GPIO 模式配置          [根据用户配置设置引脚功能]
     ├─ FreeRTOS 对象创建      [信号量 + 消息队列]
     ├─ 创建 rtos_main 线程
     └─ osKernelStart()        [调度器启动，永不返回]
          └─ rtos_main()
              ├─ USB 初始化
              ├─ ADC 启动
              ├─ 通信初始化（创建通信线程）
              ├─ 编码器/电机 setup
              ├─ PWM + ADC 中断启动
              ├─ 等待电流采样收敛
              ├─ 启动 Axis 状态机线程
              └─ 自删除
```

### 控制环运行层级

```
优先级从高到低：

[最高] 定时器中断 → sampling_cb()     采样编码器
                  → control_loop_cb()  控制环（PID + FOC + 电流环）
                       └─ osSignalSet() 通知 Axis 线程

[高]   Axis 线程  → 状态机循环          响应控制环通知，执行状态转换

[中]   通信线程   → USB/UART/CAN       处理外部指令

[低]   analog_thread → 模拟量轮询       ADC 采集（温度等）

[空闲] IdleHook   → 系统统计            堆栈水位、运行时间
```

---

下一步 → 继续阅读 Axis 状态机代码 (`axis.cpp`)

上一步 ← [A2-qa-record.md：学习问答记录](A2-qa-record.md)
