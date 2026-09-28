# Gimbal Motor Driver

云台电机驱动板固件，基于 STM32F103，通过 CAN 总线接收主控指令，驱动云台及发射机构共 4 路电机。

## 硬件平台

- **MCU**：STM32F103C6T6
- **CAN 总线**：CAN1, 1 Mbps（PA11 RX, PA12 TX）
- **控制频率**：100 Hz（10ms 周期）

## 电机配置

| 编号 | 电机型号 | 用途 | 控制方式 | PWM 定时器 | 编码器 |
|------|----------|------|----------|------------|--------|
| 1 | 520 电机 | 云台 Pitch 轴 | 位置-速度双环 PID，双 PWM | TIM3 CH1(PA6) + CH2(PA7) | TIM1 编码器 |
| 2 | 370 电机 | 摩擦轮 1 | 开环，单 PWM + GPIO 方向 | TIM2 CH1(PA0) | PA4（方向） |
| 3 | 370 电机 | 摩擦轮 2 | 开环，单 PWM + GPIO 方向 | TIM2 CH2(PA1) | PA5（方向） |
| 4 | N20 电机 | 拨弹盘 | 开环，双 PWM | TIM2 CH3(PA2) + CH4(PA3) | — |

## CAN 通信协议

### 下发帧：主控 → 驱动板

**CAN ID：`0x113`**，标准帧，数据长度 8 字节。

| 字节偏移 | 数据类型 | 字段 | 说明 |
|----------|----------|------|------|
| 0 - 3 | `int32_t` | 目标位置 (`pitch_target_pos`) | 520 Pitch 轴目标位置，单位：0.01°（如 9000 = 90.00°） |
| 4 - 5 | `int16_t` | 摩擦轮占空比 (`fric_duty`) | 两个摩擦轮共用，范围 [0, 999] |
| 6 - 7 | `int16_t` | 拨弹盘占空比 (`feed_duty`) | N20 拨弹盘，范围 [-999, 999]，正负控制方向 |

> **注意**：两个摩擦轮 370 电机硬件接线相同，转速相同、方向相反。驱动板通过 PA4/PA5 GPIO 控制正反转，主控只需下发一个正的占空比值即可。

### 上报帧：驱动板 → 主控（状态）

**CAN ID：`0x206`**，标准帧，数据长度 8 字节。

| 字节偏移 | 数据类型 | 字段 | 说明 |
|----------|----------|------|------|
| 0 - 1 | `int16_t` | 当前速度 (`pitch_current_speed`) | 520 Pitch 轴当前速度，单位：输出轴 RPM |
| 2 - 5 | `int32_t` | 当前位置 (`pitch_current_pos`) | 520 Pitch 轴当前位置，单位：0.01° |
| 6 - 7 | — | 预留 | — |

### 位置换算

```
输出轴角度 (度) = 协议值 / 100
输出轴圈数       = 协议值 / 36000
```

> 所有位置值（下发 & 上报）单位为 **0.01°**，无需主控端额外转换。驱动板内部自动完成脉冲 ↔ 角度换算。

## 软件架构

```
┌─────────────────────────────────────────────────┐
│                    App 层                        │
│  ┌───────────────┐  ┌────────────────────────┐  │
│  │ app_protocol  │  │     app_control         │  │
│  │  CAN 协议解析  │  │  位置-速度串级 PID       │  │
│  │  & 上报       │  │  (仅 520 Pitch 电机)     │  │
│  └───────┬───────┘  └───────────┬────────────┘  │
├──────────┼──────────────────────┼────────────────┤
│          ▼                      ▼                │
│  ┌───────────────┐  ┌────────────────────────┐  │
│  │   bsp_can     │  │     bsp_motor           │  │
│  │  CAN 收发封装  │  │  PWM 输出 (4路电机)     │  │
│  └───────────────┘  └────────────────────────┘  │
│                     ┌────────────────────────┐  │
│                     │     bsp_encoder         │  │
│                     │  编码器读取 (TIM1)       │  │
│                     │  零偏标定               │  │
│                     └────────────────────────┘  │
│                    Bsp 层                        │
└─────────────────────────────────────────────────┘
```

### 控制流程

```
主控 CAN 下发 (0x113)
    │
    ▼
appProtocolParse()
    │
    ├── pitch_target_pos → appControlSetTargetPos()
    │        │
    │        ▼ (每 10ms 控制周期)
    │   appControlLoop()
    │        │
    │        ├── bspEncoderUpdate()      更新编码器
    │        ├── bspEncoderGetTotalPulse() 获取当前位置 (脉冲)
    │        ├── pidCalculate(位置外环)  位置 → 目标速度 (RPM)
    │        ├── bspEncoderGetDeltaPulse() × COEF → 当前速度 (RPM)
    │        ├── pidCalculate(速度内环)  速度 → PWM 占空比
    │        └── bspMotorSetDuty(MOTOR_520_PITCH)  输出 PWM
    │
    │   [协议层转换: 0.01° ↔ 脉冲]
    │
    ├── fric_duty → bspMotorSetDuty(MOTOR_370_FRIC1)
    │             → bspMotorSetDuty(MOTOR_370_FRIC2)
    │
    └── feed_duty → bspMotorSetDuty(MOTOR_N20_FEED)
```

### PID 参数（可调）

| 参数 | 位置外环 | 速度内环 |
|------|----------|----------|
| 类型 | 位置式 PID | 增量式 PID |

> PID 参数定义在 [app_control.h](App/app_control.h) 中。运行时可通过 Ozone 调试器实时修改，修改即时生效（见下方调试章节）。

---

## Ozone 在线调参

通过 SWD 连接 Ozone 调试器，在 Watch 窗口添加全局变量 `debug_data`，即可实时查看和修改 PID 参数。

### 操作步骤

1. 用 J-Link  通过 SWD 连接驱动板
2. 打开 Ozone，加载工程的 `.elf` 文件
3. 在 **Watch** 窗口中添加变量名：`debug_data`
4. 展开结构体，可看到所有参数和实时状态

### Watch 窗口字段说明

| 字段 | 类型 | 可写 | 说明 |
|------|------|------|------|
| `pos_kp` | `float` | **是** | 位置外环 Kp |
| `pos_ki` | `float` | **是** | 位置外环 Ki |
| `pos_kd` | `float` | **是** | 位置外环 Kd |
| `pos_max_out` | `float` | **是** | 位置外环输出限幅 (RPM) |
| `pos_max_integral` | `float` | **是** | 位置外环积分限幅 |
| `spd_kp` | `float` | **是** | 速度内环 Kp |
| `spd_ki` | `float` | **是** | 速度内环 Ki |
| `spd_kd` | `float` | **是** | 速度内环 Kd |
| `spd_max_out` | `float` | **是** | 速度内环输出限幅 (PWM) |
| `spd_max_integral` | `float` | **是** | 速度内环积分限幅 |
| `target_pos_pulse` | `int32_t` | 否 | 目标位置（脉冲） |
| `current_pos_pulse` | `int32_t` | 否 | 当前位置（脉冲） |
| `current_pos_deg` | `float` | 否 | 当前位置（度） |
| `target_speed_rpm` | `float` | 否 | 位置环输出的目标速度 |
| `current_speed_rpm` | `float` | 否 | 当前速度（RPM） |
| `pos_error` | `float` | 否 | 位置误差 |
| `pos_output` | `float` | 否 | 位置环输出 |
| `spd_error` | `float` | 否 | 速度误差 |
| `spd_output` | `float` | 否 | 速度环输出 |
| `motor_duty` | `int32_t` | 否 | 最终 PWM 占空比 |

> 在 Watch 窗口中双击参数值即可修改，**下一个控制周期（10ms内）立即生效**，无需重新编译烧录。

## 文件结构

```
Gimbal_Motor_Driver/
├── Bsp/                    # 板级支持包
│   ├── bsp_motor.h/c       # 电机 PWM 驱动（4路）
│   ├── bsp_encoder.h/c     # 编码器驱动（TIM1，单路）
│   └── bsp_can.h/c         # CAN 收发封装
├── App/                    # 应用层
│   ├── app_control.h/c     # Pitch 轴双环 PID 控制
│   ├── app_protocol.h/c    # CAN 协议解析与上报
│   └── app_debug.h/c       # Ozone 在线调试数据结构
├── Core/                   # HAL 库 & CubeMX 生成代码
│   ├── Inc/
│   │   ├── main.h
│   │   ├── tim.h
│   │   ├── can.h
│   │   └── gpio.h
│   └── Src/
│       ├── main.c
│       ├── tim.c
│       ├── can.c
│       └── gpio.c
└── README.md
```

## 构建与烧录

使用VS Code Makefile 打开工程编译后，通过 SWD 或串口烧录。

- 编译选项：ARM Compiler 5/6，C11 标准
- 优化等级：建议 `-O1` 或 `-Og`（调试阶段）