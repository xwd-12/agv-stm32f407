# AGV 智能搬运机器人

> 2026 全国大学生嵌入式芯片与系统设计竞赛 — ST 赛道参赛作品

## 项目简介

基于 STM32F407ZGTx (168MHz) 的四轮差速驱动 AGV，搭载 4-DOF 机械臂与 OpenMV 视觉模块，实现**自主巡线、QR 二维码识别、AI 视觉分类、颜色对准抓取、视觉伺服对接**的全流程智能搬运。系统采用 100Hz 定时中断作为控制心跳，6 个状态机协同工作，支持串口实时调参与 CSV 数据遥测。

**应用场景**：
- **医院物流**：端侧 AI 离线运行，无需网络，隐私安全
- **快递中转场**：从车拖挂 + 视觉伺服高精度对接（±1cm）

---

## 硬件架构

```
STM32F407ZGTx (168MHz)
├── 4× 直流减速电机 (PWM 1kHz, 四轮差速驱动)
├── 4× 霍尔编码器 (正交解码, TIM2/4/5/8)
├── 4× 舵机 (腰座/大臂/小臂/夹爪, 50Hz PWM)
├── 5× 红外巡线传感器 (PC0-PC3+PA4, 低电平有效)
├── OpenMV Cam M7 (USART3, 115200, 视觉+AI)
└── USB-TTL 调试串口 (UART5, 115200)
```

### 硬件引脚分配

| 子系统 | 外设 | 引脚 |
|--------|------|------|
| 电机 0 左前 | TIM3_CH2 | PA7 (PWM), PE3/PE4 (IN1/IN2) |
| 电机 1 右前 | TIM3_CH1 | PA6 (PWM), PE5/PE6 (IN1/IN2) |
| 电机 2 左后 | TIM3_CH4 | PB1 (PWM), PC8/PC4 (IN1/IN2) |
| 电机 3 右后 | TIM12_CH2 | PB15 (PWM), PD3/PD4 (IN1/IN2) |
| 舵机 0 腰座 | TIM1_CH1 | PA8 |
| 舵机 1 大臂 | TIM1_CH2 | PA9 |
| 舵机 2 小臂 | TIM9_CH1 | PA2 |
| 舵机 3 夹爪 | TIM1_CH4 | PA11 |
| 编码器 0 左前 | TIM5 | PA0/PA1 |
| 编码器 1 右前 | TIM2 | PA15/PB3 |
| 编码器 2 左后 | TIM4 | PB6/PB7 |
| 编码器 3 右后 | TIM8 | PC6/PC7 |
| OpenMV | USART3 | PC10(TX), PC11(RX) |
| 调试串口 | UART5 | PC12(TX), PD2(RX) |

---

## 软件架构

### 模块总览

| 模块 | 文件 | 功能 |
|------|------|------|
| 主循环 | `main.c` | 入口 + AI 自动扫描流水线 + 串口命令处理 |
| 巡线控制 | `line_follow.c/h` | 5 路红外传感器 PID 巡线 @ 100Hz ISR |
| 视觉伺服 | `visual_servo.c/h` | AprilTag 双 PID 精确对接 (横向+纵向解耦) |
| PID 控制器 | `pid.c/h` | 位置式 + 增量式 PID, 支持积分分离/抗饱和/梯形积分/低通滤波 |
| 机械臂动作 | `action_group.c/h` | 预编程动作序列 (抓取/放置/复位/挂钩) |
| 舵机控制 | `servo.c/h` + `smooth_servo.c/h` | PWM 驱动 + 五次样条平滑插值 (jerk-limited) |
| 命令解析 | `command.c/h` | 串口命令解析器 (~40 条调试命令) |
| OpenMV 通讯 | `commend_openmv.c/h` | USART3 环形缓冲 + `$TAG`/`$QR`/`$CLS` 协议解析 |
| 状态机 | `state_machine.c/h` | ArmSM + TaskNav + TaskQueue 三状态机 |
| 视觉任务 | `vision_task.c/h` | 颜色搜索/QR 搜索/AI 扫描 任务编排 |
| 电机驱动 | `motor.c/h` | 方向+速度 PWM 输出, 紧急制动 |
| 编码器 | `enconder.c/h` | 正交解码 + 速度滤波 + 故障检测 |
| 定时器 | `Timer.c/h` | TIM6 100Hz 系统心跳 + PVD 低压检测 |
| 串口 | `uart.c/h` | UART5 环形缓冲 + ISR 收发 |

### 中断优先级

| ISR | 优先级 (pre,sub) | 职责 |
|-----|-----------------|------|
| PVD_IRQHandler | 0,0 | 低压检测 → 立即制动 |
| UART5_IRQHandler | 0,0 | 调试串口 RX 环形缓冲 |
| TIM6_DAC_IRQHandler | 1,0 | **100Hz 控制心跳** |
| USART3_IRQHandler | 2,0 | OpenMV 数据接收 |

---

## 控制系统

### 核心控制循环 (TIM6 ISR @ 100Hz)

系统以 TIM6 100Hz 中断为心跳，每次中断执行：
1. 编码器速度计算 + 滤波
2. 根据 `work_mode` 分发控制逻辑
3. 速度 PID 计算 → PWM 输出
4. 舵机平滑插值更新
5. 机械臂状态机计时

### 四级嵌套 PID

```
位置 PID (20ms, 外环) → 速度 PID (10ms, 内环) → PWM 输出
                                                      ↑
                         巡线 PID (10ms, 差速转向修正)
```

### 四种工作模式

| 模式 | 值 | 触发方式 | 说明 |
|------|---|---------|------|
| `MODE_MANUAL` | 0 | 串口 `mode 0` | 串口直驱 `target_speed[]` |
| `MODE_LINE_FOLLOW` | 1 | 上电默认 | 巡线传感器 PID, 自主循迹 |
| `MODE_POSITION` | 2 | 串口 `mode 2` | 编码器位置外环 PID |
| `MODE_VISUAL_SERVO` | 3 | 串口 `mode 3` | OpenMV 视觉双 PID 对接 |

### 巡线 PID 特性

- 5 路传感器加权误差：`{-2, -1, 0, +1, +2}`
- 传感器消抖：3 帧多数表决
- 积分分离：|error| > 1.5 时清零积分
- 梯形积分 + EMA 微分滤波 (70% 旧 + 30% 新)
- 死区：|error| < 0.1 → 置零
- 转向输出钳制：±600
- 速度输出钳制：[0, 1000] (只前进不倒退)
- 丢线处理：500ms 惯性滑行后停车
- 弯道检测：≤4 路检测到线连续 3 帧 → `curve_ready` 标志

### 默认控制参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 巡线 base_speed | 180 | 巡线基础速度 |
| 巡线 Kp/Ki/Kd | 180 / 2.0 / 2.0 | 巡线 PID 增益 |
| 速度 PID (电机 0,1) | Kp=5.5 Ki=0.16 Kd=0 | 前轮速度环 |
| 速度 PID (电机 2,3) | Kp=5.0 Ki=0.14 Kd=0 | 后轮速度环 |
| 位置 PID | Kp=0.5 Ki=0.001 Kd=0 | 位置外环 |
| 视觉伺服横向 Kp/Ki/Kd | 3.0 / 0.05 / 0.5 | 左右转向 |
| 视觉伺服纵向 Kp/Ki/Kd | 2.0 / 0.02 / 0.3 | 前后速度 |
| 视觉伺服目标 | cx=160, dist=15cm | 图像中心 + 15cm 距离 |

---

## OpenMV 视觉模块

### 通讯协议

OpenMV 通过 USART3 (115200) 与 STM32 通讯，使用环形缓冲 + ISR 收发模式。

**接收数据包**：
| 格式 | 说明 |
|------|------|
| `$TAG,<id>,<cx>,<cy>,<dist>,<angle>,<w>` | AprilTag/颜色色块 |
| `$QR,<payload>` | QR 码解码文本 |
| `$CLS,<class_id>,<confidence>` | AI 分类结果 |
| `$HB` | 心跳 (活跃检测) |
| `$OK,BOOT` | AI 模型加载成功 |
| `$ERR,...` | 错误信息 |

**发送指令**：`$CMD,MODE,<mode>\r\n`

### 四种视觉模式

| 模式 | 用途 | 数据输出 |
|------|------|---------|
| `APRILTAG` | 视觉伺服对接 | `$TAG` (tag_id, cx, cy, dist) |
| `QRCODE` | 二维码识别 | `$QR` (payload) |
| `AI` | 端侧 AI 分类 | `$CLS` (class_id, confidence) |
| `COLOR` | 颜色追踪 | `$TAG` (color_id, cx, cy, dist) |

### AI 模型

- **架构**：MobileNetV2 α=0.35
- **量化**：INT8 (TFLite)
- **输入**：64×64 RGB
- **分类**：3 类

| class_id | 标签 | 颜色 | 对应 COLOR ID |
|:--:|------|------|:--:|
| 0 | red_hexagon | 🔴 红色六边形 | 1 |
| 1 | green_circle | 🟢 绿色圆形 | 2 |
| 2 | yellow_rect | 🟡 黄色矩形 | 3 |

---

## 状态机系统 (6 个)

| 状态机 | 运行位置 | 触发方式 | 功能 |
|--------|---------|---------|------|
| **ArmSM** | TIM6 ISR (100Hz) | 串口/任务 请求 | 机械臂忙闲锁 + 超时保护 |
| **VisualServo** | TIM6 ISR (100Hz) | work_mode=3 | 双 PID 视觉对接底层控制 |
| **AI Pipeline** | 主循环 | 自动 (定时触发) | AI 扫描→颜色对准→抓取→继续巡线 |
| **VisionTask** | 主循环 (50ms 节流) | 串口命令 | 颜色/QR/AI 搜索→靠近→执行 |
| **TaskQueue** | 主循环 | API 调用 | 可编程 N 步骤任务序列 |
| **TaskNav** | 主循环 (50ms 节流) | 串口 `task_start` | 传统 2 段式 去→抓→回→放 任务 |

### AI 自动扫描流水线 (核心自主任务)

```
Phase 0 (冷却):  巡线中 → 5s 稳定 → 15s 冷却
     ↓
Phase 1 (AI扫描): 停车 → 腰座转 ~60° → OpenMV AI 分类
                  过滤: cls.class_id == mission_class[mission_idx] AND confidence ≥ 25%
                  8s 超时 → 重新巡线
     ↓ 目标检测到
Phase 2 (对准):  class_id → color_id 映射 → OpenMV COLOR 模式
                  子阶段 0: 腰座比例转向 (±5°), 停车对准
                  子阶段 1: 车身前后 ±50 + 腰座微调
                  对齐条件: |cx_err| ≤ 15px AND |dist_err| ≤ 8cm, 连续 3 帧
                  10s 硬超时
     ↓
Phase 3 (执行):  机械臂抓取 → 保存/恢复腰座角度
                  OpenMV → QRCODE → `MODE_LINE_FOLLOW`
                  15s 冷却 → Phase 0
```

### 机械臂状态机 (ArmSM)

- **状态**：IDLE → GRASPING/PLACING/RESETTING → IDLE
- **消抖**：50ms (5 ticks @ 100Hz)
- **超时**：抓取 30s / 放置 30s / 复位 30s → 自动 ESTOP
- **互斥**：任何代码调用 `Action_Grasp()`/`Action_Place()` 前必须检查 `ArmSM_IsBusy()`

### 舵机平滑插值

- 五次多项式 (quintic) smoothstep：位置/速度/加速度均连续
- 最大角速度 180°/s，超限自动延长时间
- 智能断电：到达目标后按关节类型执行不同策略
  - 大臂 (ID 1)：始终保持通电（抗重力）
  - 腰座 (ID 0)：到位后永久断电
  - 小臂 (ID 2)：周期通断 (通电 500ms → 断电 50ms)

---

## 串口调试系统

40+ 条串口命令，115200 波特率，UART5。启动后前 3 秒静默（抑制 USB-TTL 噪声），之后正常回显。

### 命令分类

| 类别 | 示例命令 |
|------|---------|
| 运动 | `spd <L> <R>`, `stop`, `mode <0-3>`, `reset`, `enable <0|1>` |
| PID 调参 | `kp/ki/kd <id> <val>`, `lkp/lki/lkd <val>`, `pos_kp/pos_ki/pos_kd <val>` |
| 编码器 | `enc_pos`, `enc_speed`, `enc_max`, `enc_dir`, `enc_clear` |
| 舵机 | `servo <id> <angle>`, `servo_on/off`, `a_set <id> <angle>` |
| 机械臂 | `grasp`, `place <waist>`, `reset_arm`, `show` |
| OpenMV | `vmode <mode>`, `vcolor <id>`, `vtag <id>`, `vqr`, `vdata`, `vcls` |
| 任务 | `task_start`, `task_stop`, `task_status` |
| 系统 | `help`, `save`/`load` (预留 EEPROM 接口) |

---

## 项目目录结构

```
├── User/                    # STM32 应用层全部源码
│   ├── main.c               # 入口 + AI 流水线 + 主循环
│   ├── line_follow.c/h      # 巡线 PID
│   ├── visual_servo.c/h     # 视觉伺服双 PID
│   ├── pid.c/h               # PID 控制器库
│   ├── action_group.c/h     # 机械臂动作序列
│   ├── command.c/h          # 串口命令解析
│   ├── commend_openmv.c/h   # OpenMV 通讯协议
│   ├── state_machine.c/h    # 3 状态机 (ArmSM/TaskNav/TaskQueue)
│   ├── vision_task.c/h      # 视觉任务编排
│   ├── servo.c/h            # 舵机 PWM 驱动
│   ├── smooth_servo.c/h     # 五次样条平滑插值
│   ├── motor.c/h            # 电机驱动
│   ├── enconder.c/h         # 编码器 (正交解码 + 速度滤波)
│   ├── pwm.c/h              # PWM 初始化 (TIM1/3/9/12)
│   ├── uart.c/h             # UART5 环形缓冲
│   ├── Timer.c/h            # TIM6 100Hz ISR + PVD
│   ├── sys.c/h              # NVIC 配置
│   ├── DELAY.c/h            # SysTick 阻塞/非阻塞延时
│   ├── arm_config.h         # 机械臂零点配置
│   └── stm32f4xx_it.c/h     # 中断服务覆盖
│
├── OpenMV_scripts/          # OpenMV 视觉模块
│   ├── main.py              # 多模式固件 (AprilTag/QR/AI/Color)
│   ├── train_mid.py         # MobileNetV2 α=0.35 INT8 训练脚本
│   ├── train_small.py       # 轻量 CNN 训练脚本 (内存友好)
│   ├── train.py             # 原始训练脚本
│   ├── labels.txt           # 类别标签
│   └── dataset/             # 三分类训练数据集
│
├── Project/                 # Keil MDK 工程文件
│   └── RVMDK（uv5）/SICV_F407.uvprojx
│
├── Output/                  # 编译产物
│   └── LED.hex              # 烧录固件
│
└── Libraries/               # 官方库 (只读)
    ├── CMSIS/               # ARM Cortex-M4 CMSIS
    └── STM32F4xx_StdPeriph_Driver/  # ST 标准外设库
```

---

## 开发环境

| 项目 | 版本/工具 |
|------|----------|
| IDE | Keil MDK-ARM V5.06 |
| 编译器 | ARM Compiler 5 (C90) |
| MCU | STM32F407ZGTx, 168MHz |
| 下载器 | ST-Link V2 (SWD: PA13/PA14) |
| 串口终端 | 115200 baud, 8N1 |
| OpenMV IDE | OpenMV Cam M7 固件 |

**编译流程**：Keil 打开 `Project/RVMDK（uv5）/SICV_F407.uvprojx` → F7 编译 → F8 下载。

---

## 关键设计特性

### 安全性
- **PVD 低压检测** (2.9V)：最高优先级中断，自动紧急制动
- **机械臂超时保护**：30s 动作超时自动 ESTOP
- **丢标签停车**：视觉伺服 200ms 内无数据自动停车
- **启动安全**：上电立即紧急制动，防止 GPIO 浮空导致电机误动
- **编码器故障检测**：delta > 10× 最大值连续 10 次 → 强制重同步

### 实时性
- **100Hz 控制心跳**：TIM6 中断驱动，10ms 周期
- **环形缓冲**：UART5 + USART3 均使用 ISR 写 + 主循环读的环形缓冲，杜绝丢包
- **非阻塞延时**：`Delay_Start()`/`Delay_Check()` 模式支持非阻塞等待

### 独创性
- **腰座比例转向**：AI 对准阶段用腰座舵机而非差速转向，精度更高
- **五次样条舵机插值**：位置/速度/加速度三阶连续，无冲击
- **LLM PID Tuner 框架**：CSV 遥测 → LLM 自动调参（Python 配套工具）

---

## 快速开始

### 烧录运行
1. Keil 打开 `Project/RVMDK（uv5）/SICV_F407.uvprojx`
2. F7 编译，F8 下载到 STM32F407
3. 上电自动进入巡线模式
4. 串口连接 (115200)，等待 3 秒后发送 `help` 查看命令列表

### 常用操作
```bash
help          # 查看全部命令
vpid          # 查看视觉伺服 PID 参数
mode 1        # 切换到巡线模式
mode 3        # 切换到视觉伺服模式
grasp         # 手动抓取
stop          # 紧急停车
```

---

## 演示视频

链接待补充（B站 / 视频文件）
