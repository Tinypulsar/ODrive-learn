# 附录 A1 · 开发环境搭建完整流程

> 从 VMware 虚拟机登录到编译烧录的全流程手册。基于 Ubuntu 22.04 + STM32F405 + ST-Link SWD。

---

## 0. 前置准备

| 项目 | 要求 |
|------|------|
| 虚拟机 | VMware Workstation / Player，已安装 Ubuntu 22.04 LTS |
| 硬件 | STM32F405RGT6 开发板 + ST-Link V2 调试器 + USB 数据线 |
| 接线 | ST-Link SWDIO↔SWDIO、SWCLK↔SWCLK、GND↔GND、3.3V↔3.3V（如需供电） |
| 网络 | 虚拟机能访问外网（apt / git clone） |

---

## 1. 登录虚拟机并安装工具链

```bash
# =============================================
# 步骤 1.1：更新系统
# =============================================
sudo apt update && sudo apt upgrade -y

# =============================================
# 步骤 1.2：安装编译工具链
# =============================================
# arm-none-eabi-gcc : ARM 交叉编译器，把 C/C++ 编译成 STM32 能运行的机器码
# tup               : ODrive 使用的构建系统（类似 Make 但更快）
# make              : Makefile 代理（调用 tup）
# openocd           : 通过 SWD 与芯片通信的烧录/调试工具
# gdb-multiarch     : 支持 ARM 的 GDB 调试器
# git               : 版本控制
# python3 + pip     : Python 工具和接口代码生成

sudo apt install -y \
    gcc-arm-none-eabi \
    tup \
    make \
    openocd \
    gdb-multiarch \
    git \
    python3 \
    python3-pip \
    python3-venv

# =============================================
# 步骤 1.3：验证安装
# =============================================
arm-none-eabi-gcc --version     # 应输出版本号，如 10.3.1
tup --version                   # 应输出版本号
openocd --version               # 应输出 Open On-Chip Debugger 0.x.x
python3 --version               # 应输出 Python 3.10+
git --version                   # 应输出 git version 2.x
```

---

## 2. 创建学习目录结构

```bash
# =============================================
# 步骤 2.1：创建顶层学习目录
# =============================================
# 所有 ODrive 学习内容放在一个统一的根目录下
mkdir -p ~/odrive-study
cd ~/odrive-study

# 最终目录结构预览：
# ~/odrive-study/
# ├── ODrive-learn/          ← 主仓库（从 GitHub 克隆）
# ├── .venv/                 ← Python 虚拟环境
# └── notes/                 ← 你的学习笔记（可选）
```

---

## 3. 克隆仓库

```bash
# =============================================
# 步骤 3.1：克隆你 fork 的仓库
# =============================================
cd ~/odrive-study
git clone https://github.com/Tinypulsar/ODrive-learn.git

# =============================================
# 步骤 3.2：确认仓库内容
# =============================================
cd ODrive-learn
ls
# 应看到：Firmware/  tools/  GUI/  analysis/  docs/  Arduino/  ...

# 查看固件核心文件
ls Firmware/MotorControl/
# 应看到：main.cpp  axis.cpp  foc.cpp  controller.cpp  encoder.cpp  motor.cpp  ...
```

---

## 4. 创建 Python 虚拟环境

```bash
# =============================================
# 步骤 4.1：创建虚拟环境（在 odrive-study 根目录下）
# =============================================
cd ~/odrive-study
python3 -m venv .venv

# =============================================
# 步骤 4.2：激活虚拟环境
# =============================================
source .venv/bin/activate
# 激活后命令行前面会出现 (.venv) 标志：
# (.venv) user@ubuntu:~/odrive-study$

# =============================================
# 步骤 4.3：安装 ODrive Python 工具（开发模式）
# =============================================
cd ODrive-learn/tools
pip install -e .
# -e 表示"editable"开发模式
# 改了 tools/odrive/*.py 的代码会立刻生效，不需要重新安装

# =============================================
# 步骤 4.4：验证安装
# =============================================
odrivetool --help
# 应显示 odrivetool 的帮助信息

# =============================================
# 步骤 4.5：安装仿真分析依赖（可选，跑 MotorSim.py 用）
# =============================================
pip install numpy matplotlib scipy

# =============================================
# 以后每次打开终端都需要重新激活虚拟环境：
# source ~/odrive-study/.venv/bin/activate
#
# 可以加到 .bashrc 自动激活（可选）：
# echo 'source ~/odrive-study/.venv/bin/activate' >> ~/.bashrc
# =============================================
```

---

## 5. 配置并编译固件

```bash
# =============================================
# 步骤 5.1：进入固件目录
# =============================================
cd ~/odrive-study/ODrive-learn/Firmware

# =============================================
# 步骤 5.2：配置板型
# =============================================
cp tup.config.default tup.config
nano tup.config

# 在文件中找到 CONFIG_BOARD_VERSION 这一行，改成你的板型：
#
# 常见板型：
#   v3.4-24V   （24V 版）
#   v3.5-24V   （24V 版）
#   v3.5-48V   （48V 版）
#   v3.6-24V   （24V 版）
#   v3.6-56V   （56V 版，最常见）
#
# 改完保存：Ctrl+O → Enter → Ctrl+X

# =============================================
# 步骤 5.3：初始化构建系统
# =============================================
tup init
# 只需执行一次，会创建 .tup/ 目录

# 如果报 FUSE 错误（虚拟机中偶尔出现）：
# tup generate build.sh && bash build.sh

# =============================================
# 步骤 5.4：编译
# =============================================
make

# 编译过程约 1-3 分钟，成功后输出类似：
#   [  1%] Compiling Firmware/...
#   ...
#   [100%] Linking build/ODriveFirmware.elf
#
# 生成文件：
#   build/ODriveFirmware.elf   ← 含调试信息（GDB 用）
#   build/ODriveFirmware.hex   ← Intel HEX（烧录用）
#   build/ODriveFirmware.bin   ← 纯二进制（烧录用）

# =============================================
# 步骤 5.5：确认编译产物
# =============================================
ls -lh build/ODriveFirmware.*
# 应看到 .elf（约 500KB-1MB）、.hex、.bin 三个文件
```

---

## 6. 挂载 ST-Link 并烧录

```bash
# =============================================
# 步骤 6.1：在 VMware 中挂载 ST-Link
# =============================================
# 把 ST-Link 插到电脑 USB 口
# VMware 菜单栏 → 虚拟机(M) → 可移动设备
#   → STMicroelectronics ST-LINK/V2
#   → 连接（断开与主机的连接）
#
# 如果弹出"此设备即将从主机断开"，点确定

# =============================================
# 步骤 6.2：验证 Linux 能识别 ST-Link
# =============================================
lsusb | grep ST-LINK
# 应输出：Bus 00x Device 00x: ID 0483:3748 STMicroelectronics ST-LINK/V2
#
# 如果没有输出，检查：
#   - VMware 是否成功挂载（菜单中应显示"断开连接"而非"连接"）
#   - USB 线是否是数据线（不是纯充电线）
#   - ST-Link 上的指示灯是否亮

# =============================================
# 步骤 6.3：烧录
# =============================================
cd ~/odrive-study/ODrive-learn/Firmware

# 方式 A：用 Makefile 封装的命令
make flash

# 方式 B：手动执行 OpenOCD（效果相同，但能看到更多细节）
openocd -f interface/stlink.cfg \
        -f target/stm32f4x.cfg \
        -c "program build/ODriveFirmware.hex verify reset exit"

# 成功输出：
#   ** Programming Started **
#   ** Programming Finished **
#   ** Verify Started **
#   ** Verified OK **
#   ** Resetting Target **

# =============================================
# 步骤 6.4：如果需要擦除旧配置
# =============================================
make erase_config
# 清除 Flash 最后一个扇区中保存的用户配置
```

---

## 7. 一张图总结完整流程

```
 ┌─ VMware Ubuntu 虚拟机 ──────────────────────────────────────────┐
 │                                                                 │
 │  ~/odrive-study/                                                │
 │  ├── .venv/                    ← Python 虚拟环境                │
 │  │   └── (odrivetool, numpy, matplotlib...)                     │
 │  │                                                              │
 │  └── ODrive-learn/             ← Git 仓库（主分支: master）     │
 │      ├── Firmware/                                              │
 │      │   ├── tup.config        ← 板型配置                      │
 │      │   ├── MotorControl/     ← 源码                          │
 │      │   └── build/                                             │
 │      │       ├── .elf          ← 编译产物（调试用）             │
 │      │       └── .hex          ← 编译产物（烧录用）             │
 │      ├── tools/                ← Python 工具（pip install -e .）│
 │      └── analysis/             ← 仿真脚本                      │
 │                                                                 │
 │  工具链：                                                       │
 │    arm-none-eabi-gcc → 编译                                     │
 │    tup / make        → 构建                                     │
 │    openocd           → 烧录 ──────────────┐                     │
 │    gdb-multiarch     → 调试                │                    │
 │                                            │ SWD                │
 └────────────────────────────────────────────┼────────────────────┘
                                              │
                                              ▼
                                        ┌──────────┐
                                        │ ST-Link  │
                                        └────┬─────┘
                                             │ SWDIO/SWCLK
                                             ▼
                                        ┌──────────┐
                                        │ STM32F405│
                                        └──────────┘
```

---

## 8. 实验分支管理

### 8.1 创建新实验

```bash
# 确保在主仓库目录
cd ~/odrive-study/ODrive-learn

# 确保当前在 master 分支，且代码干净
git checkout master
git status
# 应显示 "working tree clean"

# 创建并切换到新实验分支
git checkout -b exp/01-pid-tuning
# 分支命名规范：exp/编号-简短描述
```

### 8.2 在实验中修改代码

```bash
# 修改代码（比如改 controller.cpp 中的 PID 增益）
nano Firmware/MotorControl/controller.cpp

# 编译验证
cd Firmware && make

# 烧录测试
make flash

# 确认修改有效后，提交
cd ~/odrive-study/ODrive-learn
git add Firmware/MotorControl/controller.cpp
git commit -m "exp01: 将 pos_gain 从 20 改为 50，观察阶跃响应

目的：验证位置环比例增益对超调和稳定时间的影响
修改：controller.hpp line 27, pos_gain = 50.0f
结果：超调增大到 15%，稳定时间缩短到 50ms"
```

### 8.3 切换实验

```bash
# 回到干净的 master
git checkout master

# 开始新实验
git checkout -b exp/02-smo-observer

# ... 修改、编译、测试、提交 ...

# 想回去看实验 1？
git checkout exp/01-pid-tuning

# 想回到实验 2？
git checkout exp/02-smo-observer
```

### 8.4 查看所有实验

```bash
# 列出所有分支
git branch
#   exp/01-pid-tuning
#   exp/02-smo-observer
# * master                    ← * 表示当前所在分支

# 查看某个实验改了什么
git log master..exp/01-pid-tuning --oneline
# 显示实验 1 相对于 master 的所有提交

# 对比两个实验的代码差异
git diff exp/01-pid-tuning exp/02-smo-observer -- Firmware/MotorControl/
```

### 8.5 实验完成后打标签存档

```bash
# 给完成的实验打标签（永久标记）
git tag -a exp01-done -m "实验1完成：PID增益调优" exp/01-pid-tuning

# 以后可以通过标签快速定位
git checkout exp01-done
```

### 8.6 推送实验到 GitHub 备份

```bash
# 推送单个实验分支
git push -u origin exp/01-pid-tuning

# 推送所有实验分支
git push --all origin

# 推送所有标签
git push --tags origin
```

---

## 9. 日常工作流速查

```bash
# ==================== 每次开机 ====================
# 1. 打开 VMware → 启动 Ubuntu
# 2. 打开终端
source ~/odrive-study/.venv/bin/activate     # 激活 Python 环境
cd ~/odrive-study/ODrive-learn               # 进入仓库

# ==================== 开始新实验 ====================
git checkout master                          # 回到主分支
git checkout -b exp/03-xxx                   # 创建新实验
# ... 改代码 ...

# ==================== 编译烧录 ====================
cd Firmware
make                                         # 编译
# （插上 ST-Link，VMware 挂载 USB）
make flash                                   # 烧录

# ==================== 保存进度 ====================
cd ~/odrive-study/ODrive-learn
git add -A                                   # 暂存修改
git commit -m "exp03: 描述你的修改"           # 提交
git push -u origin exp/03-xxx                # 推送备份

# ==================== 跑仿真 ====================
cd analysis/Simulation
python3 MotorSim.py                          # Python 电机仿真
```

---

下一步 → [01-overview.md：仓库全景与架构概览](01-overview.md)
