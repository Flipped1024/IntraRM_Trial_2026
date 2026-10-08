# Chassis Motor Driver

基于 STM32F103C6T6 的双路 520 电机驱动固件。固件通过 CAN 接收两个电机的目标速度，使用编码器反馈进行增量式速度 PID 控制，并通过 CAN 上报编码器速度增量和累计位置。

## 当前项目配置

| 项目 | 配置 |
|---|---|
| MCU | STM32F103C6T6（LQFP48） |
| 系统时钟 | HSE 12 MHz，PLL ×6，SYSCLK/HCLK 72 MHz |
| CAN 外设 | CAN1，Normal 模式，自动重发开启 |
| CAN 波特率 | 500 kbit/s |
| CAN 引脚 | PA11：CAN_RX；PA12：CAN_TX |
| 控制周期 | 10 ms（100 Hz） |
| 调试接口 | SWD，PA13/PA14 |

CAN 引脚是 MCU 侧信号，实际接入 CANH/CANL 时还需要 CAN 收发器。CAN 总线两端应根据总线拓扑配置终端电阻。

## 电机和编码器引脚

| 逻辑电机 | PWM 输出 | 编码器输入 | 计数定时器 |
|---|---|---|---|
| 电机 1 | TIM2_CH1/PA0、TIM2_CH2/PA1 | TIM1_CH1/PA8、TIM1_CH2/PA9 | TIM1 |
| 电机 2 | TIM2_CH3/PA2、TIM2_CH4/PA3 | TIM3_CH1/PA6、TIM3_CH2/PA7 | TIM3 |

PWM 定时器周期为 1000 个计数（ARR=999），PWM 输出限幅为 `-999` 到 `999`。每个电机使用两个 PWM 通道控制方向：正值使用第一个通道，负值使用第二个通道，零值关闭两个通道。

编码器配置为 TI12 正交编码模式，计数器为 16 位，输入未使用内部上下拉。电机 1/2 的机械位置由实际接线决定；固件只区分逻辑编号 `MOTOR_1` 和 `MOTOR_2`。

## CAN 通信协议

所有协议帧均为 11 位标准数据帧，约定 DLC 为 8。多字节整数采用大端格式（高字节在前）。

### 板号和 CAN ID

当前 [board_config.h](board_config.h) 中的配置为：

```c
#define BOARD_ID 1
```

| `BOARD_ID` | 接收控制帧 | 上报速度帧 | 上报位置帧 |
|---:|---:|---:|---:|
| 1 | `0x111` | `0x222` | `0x223` |
| 2 | `0x112` | `0x224` | `0x225` |

修改 `BOARD_ID` 后必须重新编译并下载固件。

### 主控 → 驱动板：速度控制帧

帧 ID 为 `0x111`（板 1）或 `0x112`（板 2）。接收过滤器使用全掩码，只接收当前板号对应的控制 ID，并将帧分配到 CAN FIFO0；FIFO0 中断回调再调用协议解析函数。

| 字节 | 类型 | 字段 | 说明 |
|---|---|---|---|
| 0–1 | `int16_t` | 电机 1 目标速度 | 输出轴 RPM 的目标值 |
| 2–3 | `int16_t` | 电机 2 目标速度 | 输出轴 RPM 的目标值 |
| 4–7 | — | 预留 | 当前未使用 |

控制数据解析顺序为：

```text
CAN FIFO0 中断
    → HAL_CAN_RxFifo0MsgPendingCallback()
    → appProtocolParse()
    → appControlSetTargetSpeed()
```

### 驱动板 → 主控：速度帧

帧 ID 为 `0x222`（板 1）或 `0x224`（板 2）。

| 字节 | 类型 | 字段 | 说明 |
|---|---|---|---|
| 0–1 | `int16_t` | 电机 1 速度 | 本控制周期的编码器增量脉冲 |
| 2–3 | `int16_t` | 电机 2 速度 | 本控制周期的编码器增量脉冲 |
| 4–7 | — | 预留 | 当前为 0 |

该帧上报的是编码器增量脉冲，不是 RPM。接收端如果需要 RPM，应使用相同的控制周期和编码器参数换算。

### 驱动板 → 主控：位置帧

帧 ID 为 `0x223`（板 1）或 `0x225`（板 2）。

| 字节 | 类型 | 字段 | 说明 |
|---|---|---|---|
| 0–3 | `int32_t` | 电机 1 累计位置 | 编码器累计脉冲数 |
| 4–7 | `int32_t` | 电机 2 累计位置 | 编码器累计脉冲数 |

## 编码器速度和位置换算

当前物理参数定义在 [Bsp/bsp_motor.h](Bsp/bsp_motor.h)：

```c
MOTOR_GEAR_RATIO   = 56
ENCODER_PPR        = 13
ENCODER_MULTIPLIER = 4
CONTROL_PERIOD_S   = 0.01
```

因此：

```text
PULSES_PER_SHAFT_REV = 13 × 4 × 56 = 2912 脉冲/输出轴一圈
PULSE_TO_RPM_COEF    = 60 / (0.01 × 2912) ≈ 2.06044 RPM/脉冲
控制环当前速度       = 本周期增量脉冲 × PULSE_TO_RPM_COEF
输出轴圈数           = 累计脉冲数 / 1560
```

电机目标速度在 CAN 接收和 Ozone 手动目标入口均限制为 ±120 RPM；`current_speed_rpm` 和 `raw_speed_rpm` 当前都来自同一个未经滤波的本周期速度采样值；当前代码没有启用速度滤波。

## 软件控制流程

SysTick 每 1 ms 先递增 HAL 时基，再调用 `HAL_SYSTICK_IRQHandler()`；该函数进入 `HAL_SYSTICK_Callback()`。回调累计 10 次后置位控制标志，主循环检测到标志后执行一次控制与状态上报周期：

```text
SysTick_Handler()
    → HAL_IncTick()
    → HAL_SYSTICK_IRQHandler()
    → HAL_SYSTICK_Callback()
    → control_flag = 1

主循环检测 control_flag
    → appControlLoop()
        → 读取 TIM1/TIM3 编码器计数
        → 计算两个电机的增量脉冲
        → 增量脉冲换算为 RPM
        → 速度一阶低通滤波
        → 目标速度变化率前馈
        → pidCalculate()
        → bspMotorSetSpeed()
        → appDebugUpdate()
    → 发送速度帧和位置帧
```

CAN 接收在中断中更新 CAN 目标速度；速度控制和 CAN 状态上报在主循环的 10 ms 周期中执行。PID 目标来源由 `debug_mode` 选择：默认值 `0` 使用 CAN 目标，值为 `1` 时使用 Ozone 手动目标。两种模式在目标选择后共用速度低通滤波、PID 和加速度前馈。`control_flag` 是单个标志位：若主循环被阻塞超过一个周期，多个待执行周期会合并为一次，而不会补执行。

每个状态周期会分别尝试将速度帧和位置帧加入 CAN 发送邮箱；速度帧排队失败不会再阻止位置帧尝试。`HAL_CAN_AddTxMessage()` 成功只表示帧已被接受进入邮箱，不等同于总线已发送或收到 ACK。

## PID 当前实现

两个电机均使用增量式 PID：

```text
e(k) = target(k) - measure(k)
Δu(k) = Kp × [e(k)-e(k-1)]
      + Ki × e(k)
      + Kd × [e(k)-2e(k-1)+e(k-2)]
u(k) = u(k-1) + Δu(k)
```

当前默认参数定义在 [App/app_control.h](App/app_control.h)：

| 参数 | 默认值 |
|---|---:|
| `MOTOR_SPEED_PID_KP` | `10.0` |
| `MOTOR_SPEED_PID_KI` | `1.0` |
| `MOTOR_SPEED_PID_KD` | `5.0` |
| 输出限幅 | `±999` |

`spd_max_integral` 会从 `debug_data` 读取并写入 PID 对象，但当前增量式 PID 分支没有使用它进行额外积分限幅。`spd_deadband` 当前仍未使用。

## Ozone 调试数据

全局调试对象为 `volatile DebugData debug_data`，在每个 10 ms 控制周期末更新。可以在 Ozone 的 Watch、Memory 或 Data Sampling/Graph 窗口中观察：

```text
debug_data
debug_data.motor[0].current_speed_rpm
debug_data.motor[1].current_speed_rpm
```

### 顶层字段

| 字段 | 类型 | 当前用途 |
|---|---|---|
| `spd_kp` | `float` | 速度环比例参数，可在线修改 |
| `spd_ki` | `float` | 速度环积分参数，可在线修改 |
| `spd_kd` | `float` | 速度环微分参数，可在线修改 |
| `spd_max_out` | `float` | PID 输出限幅，可在线修改 |
| `spd_max_integral` | `float` | 写入 PID 对象，但当前增量式 PID 未使用 |
| `spd_deadband` | `float` | 当前未使用 |
| `spd_filter_alpha` | `float` | 速度低通滤波系数，范围 `0` 到 `1`，默认 `0.25` |
| `spd_ff_accel` | `float` | 目标加速度前馈增益，可在线修改；默认 `0` |
| `debug_mode` | `uint8_t` | `0`：CAN 目标（默认）；`1`：使用 `debug_target_speed_rpm` 的 Ozone 手动目标；其他值同 `0` |

### `motor[i]` 字段

| 字段 | 类型 | 说明 |
|---|---|---|
| `target_speed_rpm` | `float` | 当前实际进入 PID 的目标速度；仅回显，不应用于写入控制 |
| `current_speed_rpm` | `float` | 当前控制周期速度，单位 RPM |
| `raw_speed_rpm` | `float` | 当前与 `current_speed_rpm` 相同 |
| `delta_pulse` | `int16_t` | 当前控制周期编码器增量脉冲 |
| `speed_error` | `float` | 目标速度与当前速度之差 |
| `speed_output` | `float` | PID 输出 |
| `feedforward_output` | `float` | 加速度前馈输出 |
| `motor_duty` | `int32_t` | 当前 PWM 控制量 |
| `total_pulse` | `int32_t` | 编码器累计脉冲 |
| `debug_target_speed_rpm` | `float` | Ozone 手动目标，只有 `debug_mode=1` 时参与 PID，单位 RPM |

注意：`current_speed_rpm` 是 10 ms 窗口的增量速度。手动缓慢转动或停止转动时，某些采样周期为 0 是正常现象；确认编码器是否累计时应优先观察 `total_pulse`。

当前 `debug_data` 没有 CAN 收发计数器。需要确认 CAN 通信时，应结合 CAN 分析仪和接收回调断点检查；发送 API 接受帧到邮箱也不能单独证明总线 ACK 或电机动作。

速度滤波使用一阶离散低通：

```text
filtered(k) = filtered(k-1) + alpha × [raw(k) - filtered(k-1)]
```

`spd_filter_alpha` 的有效范围为 `0` 到 `1`；值越小越平滑，但响应延迟越大。默认值为 `0.25`。滤波后的速度进入 PID，原始速度仍通过 `raw_speed_rpm` 提供观察。

前馈使用目标速度变化率估计轮子转动惯量带来的加速需求：

```text
target_accel = [target(k) - target(k-1)] / 0.01
feedforward  = spd_ff_accel × target_accel
control      = PID_output + feedforward
```

`spd_ff_accel` 的单位是 `PWM / (RPM/s)`，默认值为 `0`，需要在实际轮电机和负载上逐步调大。目标加速度限制为 `±3000 RPM/s`，总输出仍限制在 `±spd_max_out` 和 PWM 范围内。该前馈位于 CAN/Ozone 模式共同的控制路径中。

### Ozone 手动目标测试

默认 `debug_data.debug_mode=0`，CAN 控制路径不变。需要不经过 CAN 单独验证电机、编码器和 PID 时，可按下面顺序操作：

1. 架空底盘或断开机械负载，先确认驱动器供电和急停措施。
2. 在 Ozone 写入 `debug_data.debug_mode = 1`。
3. 写入 `debug_data.motor[0].debug_target_speed_rpm = 5.0`，电机 2 保持 `0.0`；确认方向和闭环正常后再逐步提高。
4. 观察 `target_speed_rpm`、`current_speed_rpm`、`speed_error`、`speed_output` 和 `motor_duty`。其中 `target_speed_rpm` 应回显为当前手动值，`motor_duty` 非零表示固件已输出 PWM 命令，但不单独证明 H 桥功率级或电机接线正常。
5. 停止时先将两个 `debug_target_speed_rpm` 写为 `0.0`。恢复 CAN 前，应先让主控发送零速度，再将 `debug_mode` 写回 `0`；CAN 最近接收的目标始终被保留，切回 CAN 后会立即生效。

## CAN 通信检查

建议按以下顺序检查：

1. 运行固件后，用 CAN 分析仪设置 500 kbit/s、标准帧，观察是否周期性出现当前板号的速度帧和位置帧。
2. 向板 1 发送 `0x111`，或向板 2 发送 `0x112`，检查是否进入 `HAL_CAN_RxFifo0MsgPendingCallback()` 和 `appProtocolParse()`。
3. 发送测试数据 `00 01 00 01 00 00 00 00` 后，观察两个 `target_speed_rpm` 是否变为 1。电机测试前应将机械结构架空或断开动力负载。
4. 如果没有发送帧，检查固件是否下载、目标是否运行、收发器供电、CANH/CANL 接线、波特率和终端电阻。
5. 如果能看到发送帧但接收断点不进入，检查接收 ID、PA11 接线、CAN 收发器和 CAN 过滤器配置。

CAN 正常模式需要总线上存在能够 ACK 的其他节点。只有一个设备和被动监听分析仪时，发送端可能出现 ACK 错误；这不能单独证明固件发送逻辑错误。

## 文件结构

```text
Chassis_Motor_Driver/
├── board_config.h              # 板号和 CAN ID 配置
├── Makefile                    # GNU Arm GCC 构建配置
├── STM32F103XX_FLASH.ld        # 链接脚本
├── Chassis_Motor_Driver.ioc    # CubeMX 配置
├── debug.jdebug                # Ozone 工程配置
├── App/
│   ├── app_control.h/c         # PID 和 10 ms 控制逻辑
│   ├── app_protocol.h/c        # CAN 协议解析和上报
│   └── app_debug.h/c           # Ozone 调试数据
├── Bsp/
│   ├── bsp_can.h/c             # CAN 收发封装
│   ├── bsp_encoder.h/c         # TIM1/TIM3 编码器读取
│   └── bsp_motor.h/c           # TIM2 PWM 电机驱动
├── Core/
│   ├── Inc/                    # HAL、外设和应用头文件
│   └── Src/                    # main、CAN、TIM、GPIO 和中断代码
└── build/                      # 编译生成目录，不提交到 Git
```

## 构建和下载

### 构建环境

- 工具链：GNU Arm Embedded GCC（`arm-none-eabi-gcc`）
- 构建工具：`mingw32-make`
- 当前 Makefile 优化等级：`-Og`
- 调试信息：`-g -gdwarf-2`

在 PowerShell 中执行：

```powershell
mingw32-make -j2
```

成功后生成：

```text
build/Chassis_Motor_Driver.elf
build/Chassis_Motor_Driver.hex
build/Chassis_Motor_Driver.bin
```

### Ozone

`debug.jdebug` 当前配置为 STM32F103C6、SWD、4 MHz 调试速率，并打开：

```text
build/Chassis_Motor_Driver.elf
```

重新编译后，需要在 Ozone 中重新加载最新 ELF，并将固件下载到目标板后再观察运行时变量。仅重新编译而不下载，目标板仍执行旧固件。

## 相关配置文件

- [board_config.h](board_config.h)：切换 `BOARD_ID` 和对应 CAN ID
- [Chassis_Motor_Driver.ioc](Chassis_Motor_Driver.ioc)：MCU、CAN、TIM 和引脚配置
- [Makefile](Makefile)：构建源文件、编译选项和输出文件
- [debug.jdebug](debug.jdebug)：Ozone 调试工程
