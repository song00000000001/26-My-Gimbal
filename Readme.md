# 26 自制云台

> 基于 STM32F405RGT6 的两轴云台控制系统，搭载 MPU6050 IMU，驱动本末科技 M0603A 驱控一体电机，实现 Pitch / Yaw 双轴位置环 PID 稳定控制。

---

## 目录

- [项目简介](#项目简介)
- [硬件组成](#硬件组成)
- [软件架构](#软件架构)
- [通信接口](#通信接口)
- [电机驱动库](#电机驱动库)
- [PID 控制](#pid-控制)
- [目录结构](#目录结构)
- [快速上手](#快速上手)
- [开发进度](#开发进度)
- [待办事项](#待办事项)

---

## 项目简介

本项目是一个自制两轴云台电控系统，主控芯片为 STM32F405RGT6，IMU 使用板载 MPU6050，电机采用本末科技 M0603A 驱控一体电机（通过自制通信转接板连接），运行 FreeRTOS 实时操作系统，使用 SRML（SCUT Robotlab Middlewares Layer Library）中间件库作为底层支撑，在位置环模式下对 Pitch 和 Yaw 两轴进行 PID 闭环稳定控制。

---

## 硬件组成

| 部件 | 型号 / 说明 |
|------|------------|
| 主控芯片 | STM32F405RGT6（LQFP64，168 MHz，Cortex-M4） |
| IMU | MPU6050（板载，提供角度与角速度反馈） |
| 电机 × 2 | 本末科技 M0603A 驱控一体电机（Pitch + Yaw） |
| 通信转接板 | 自制电机通信转接板（UART 转电机专有协议） |
| 遥控器 | 富斯 FS-I6X（DBUS 协议，USART2） |
| 调试工具 | OpenLog 日志模块（UART5） |
| LED | WS2812 可编程 RGB 灯带（TIM3 PWM DMA 驱动） |

**主控时钟配置：**
- 外部晶振 HSE = 25 MHz
- PLL 倍频后系统时钟 SYSCLK = 168 MHz
- AHB = 168 MHz，APB1 = 42 MHz，APB2 = 84 MHz

---

## 软件架构

```
FreeRTOS 任务
├── task_motor_ctrl      电机控制任务（位置环 PID + 电机指令收发，7 ms 周期）
├── task_state_machine   状态机任务（2 ms 周期）
├── armer_ctrl_task      机械臂/其他控制任务（2 ms 周期，预留）
├── Task_CAN1/2Transmit  CAN 发送任务（Realtime 优先级）
├── Task_CAN1/2Receive   CAN 接收任务（Realtime 优先级）
├── Task_UsartTransmit   UART DMA 发送任务
└── Task_UsartReceive    UART 接收任务（派发各串口数据）

中间件层（SRML 子模块）
├── drv_uart / drv_can   底层外设驱动
├── FS_I6X               富斯遥控器解析
├── MPU6050              IMU 数据获取
└── PID / 滤波器          算法模块

用户层（USP）
├── Drivers/             自定义驱动
│   ├── motor_m0603A_driver   本末 M0603A 电机驱动库
│   ├── motor_ctrl_driver     电机控制层
│   ├── remote_ctrl_driver    遥控器驱动适配
│   └── ws2812_ctrl_driver    WS2812 LED 驱动
└── Application/
    ├── Robot_Module/    机器人业务逻辑（全局数据、状态机、电机控制任务）
    ├── Service_Communication.cpp   通信服务（CAN/UART 队列管理）
    ├── Service_Device.cpp          设备初始化
    └── Service_Debug.cpp           调试服务
```

---

## 通信接口

| 接口 | 功能 | 波特率 / 参数 |
|------|------|--------------|
| CAN1 | 预留（原电机通信）| 1 Mbps |
| CAN2 | 分控通信 | 1 Mbps |
| USART1 | 电机反馈接收（当前） | 38400 |
| USART2 | 富斯 FS-I6X 遥控器（RX only）| 100000，9位，偶校验，DBUS |
| USART3 | 调试输出（预留）| 115200 |
| UART4 | 电机指令发送（主通信口）| 38400 |
| UART5 | OpenLog 日志 | — |
| USART6 | 预留 | — |
| USB CDC | 虚拟串口调试 | — |

---

## 电机驱动库

自定义 `BenMoMotor` 驱动类，封装本末科技 M0603A 的串口通信协议（10 字节定长包，CRC-8/MAXIM 校验）。

**核心接口：**

```cpp
BenMoMotor motor(uint8_t motor_id);

// 注册底层发送函数（仅调用一次）
BenMoMotor::registerSendFunction(send_func);

// 发送控制指令
motor.sendEnableCmd(bool enable);               // 使能 / 失能
motor.sendModeCmd(MotorMode mode);              // 切换运行模式
motor.sendPositionCtrl(float deg, uint8_t acc); // 位置环控制（单位：度）
motor.sendSpeedCtrl(float rpm, uint8_t acc);    // 速度环控制
motor.sendCurrentCtrl(float current);           // 电流环控制

// 解析反馈数据
motor.parseDriveFeedback(const uint8_t* buf);   // 解析速度/电流/温度
motor.parseExtraFeedback(const uint8_t* buf);   // 解析里程/精确位置/模式

// 读取状态
motor.getSpeed();       // 当前转速（×0.1 RPM）
motor.getCurrent();     // 当前电流（-4A ~ 4A）
motor.getTemp();        // 绕组温度（℃）
motor.getFaultCode();   // 故障码
motor.getPosition();    // 当前位置（0~32767 对应 0~360°）
motor.getMileage();     // 累计里程圈数
```

**支持的运行模式（`MotorMode`）：**

| 枚举值 | 模式 |
|--------|------|
| `OPEN_LOOP` | 开环模式 |
| `CURRENT_LOOP` | 电流环（力矩控制） |
| `SPEED_LOOP` | 速度环 |
| `POSITION_LOOP` | 位置环 |

---

## PID 控制

云台采用**增量式位置环 PID**（`MY_PID_MODE_GIMBAL_INC_POS`）进行姿态稳定控制：

- **反馈源：** MPU6050 提供 `imu_angle_deg[]`（角度）和 `imu_gyro_dps[]`（角速度）
- **控制周期：** 7 ms（与电机通信周期同步）
- **双轴：** `PITCH (index 0)` 和 `YAW (index 1)` 独立 PID 实例
- **限幅配置：**
  - 目标输出范围：0 ~ 360°
  - 积分限幅：±10°
  - 单次增量限幅：±2°

---

## 目录结构

```
26-My-Gimbal/
├── Core/                    STM32CubeMX 生成的 HAL 初始化代码
│   ├── Inc/                 头文件（main.h, usart.h, can.h 等）
│   └── Src/                 源文件（main.c, freertos.c, usart.c 等）
├── Drivers/                 STM32 HAL/CMSIS 驱动库
├── MDK-ARM/                 Keil MDK 工程文件
├── Middlewares/             FreeRTOS 及其他中间件
├── SRML/                    SCUT Robotlab 中间件库（Git 子模块）
├── USB_DEVICE/              USB CDC 设备描述符
├── USP/                     用户业务层
│   ├── Application/         应用层任务与服务
│   │   └── Robot_Module/    机器人核心逻辑
│   ├── Drivers/             自定义驱动
│   │   ├── motor_m0603A_driver.*   本末电机驱动
│   │   ├── motor_ctrl_driver.*     电机控制封装
│   │   ├── remote_ctrl_driver.*    遥控器驱动
│   │   └── ws2812_ctrl_driver.*    WS2812 LED 驱动
│   └── Services/            服务层（预留）
├── Template.ioc             STM32CubeMX 工程配置文件
└── Readme.md                本文件
```

---

## 快速上手

### 环境要求

- **IDE：** Keil MDK-ARM V5.32+
- **STM32CubeMX：** 6.15.0+（如需重新生成 HAL 代码）
- **工具链：** ARM Compiler V5 / V6

### 克隆工程

```bash
git clone <repository_url>
cd 26-My-Gimbal
git submodule update --init --recursive   # 拉取 SRML 子模块
```

### 编译与烧录

1. 使用 Keil MDK 打开 `MDK-ARM/` 下的 `.uvprojx` 工程文件
2. 编译工程（F7）
3. 通过 SWD 接口（PA13/PA14）连接调试器烧录固件

### 硬件接线要点

- UART4 (PA0 TX / PA1 RX)：连接电机通信转接板（38400 baud）
- USART1 (PA9 TX / PA10 RX)：电机反馈接收（38400 baud）
- USART2 (PA2 TX / PA3 RX)：FS-I6X 遥控器接收（DBUS）
- CAN1 (PA11 RX / PA12 TX)：预留 CAN 总线
- CAN2 (PB12 RX / PB13 TX)：分控通信

---

## 开发进度

- [x] STM32F405 基础工程搭建（FreeRTOS + HAL + SRML）
- [x] 本末科技 M0603A 电机驱动库（基于官方手册编写）
- [x] 电机使能 / 失能测试通过
- [x] 角度环模式切换测试通过
- [x] 角度环目标设置测试通过
- [x] 增量式位置环 PID 控制器接入
- [x] WS2812 LED 驱动
- [x] FS-I6X 遥控器接入（通过 SRML）
- [ ] 额外反馈解码（里程 / 精确位置）——调试中，波形与协议待核查
- [ ] 速度环 / 电流环测试
- [ ] PID 参数整定与性能验证
- [ ] 重构电机驱动通信层（上层只调接口，驱动内部处理协议细节）

---

## 待办事项

- [ ] 排查额外反馈（0x74 指令）解码失败问题（对照手册核查波形）
- [ ] 测试速度环和电流环控制模式
- [ ] 验证 AI 辅助编写的 PID 库逻辑正确性
- [ ] 考虑在通信层按 4 ms 周期统一调度电机指令，降低丢包率
- [ ] 评估双电机同时占用总线时的最短安全通信间隔
- [ ] 添加重传机制以进一步降低丢包率（当前 7 ms 周期下约 1%~2%）
