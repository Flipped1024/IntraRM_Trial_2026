# Yaw/Pitch Motor Driver

基于 STM32F103C6T6 的双轴位置电机驱动板固件。硬件布局、CAN、PWM 和编码器引脚与 `Chassis_Motor_Driver` 完全一致，但两个电机的用途改为：

- 电机 1：520 电机，Pitch 轴，位置-速度串级 PID；
- 电机 2：370 电机，Yaw 轴，位置-速度串级 PID。

两路电机均使用与本项目硬件一致的编码器接线和 **11 个脉冲/编码器轴/圈** 参数。编码器采用 TIM1/TIM3 的 TI12 正交计数，四倍频后再结合 56:1 减速箱换算输出轴位置。

## 硬件和引脚

| 项目 | 配置 |
|---|---|
| MCU | STM32F103C6T6，LQFP48 |
| 系统时钟 | 外部 HSE 12 MHz，PLL ×6，SYSCLK/HCLK 72 MHz |
| CAN | CAN1，Normal，500 kbit/s |
| CAN 引脚 | PA11：CAN_RX；PA12：CAN_TX |
| 控制周期 | 10 ms（100 Hz） |
| 调试接口 | SWD，PA13/PA14 |

CAN 引脚是 MCU 侧信号。接入 CANH/CANL 时需要外部 CAN 收发器；总线两端按实际拓扑配置 120 Ω 终端电阻。

| 逻辑轴 | 电机 | PWM 输出 | 编码器输入 | 计数定时器 |
|---|---|---|---|---|
| `MOTOR_PITCH` | 520 | TIM2_CH1/PA0、TIM2_CH2/PA1 | TIM1_CH1/PA8、TIM1_CH2/PA9 | TIM1 |
| `MOTOR_YAW` | 370 | TIM2_CH3/PA2、TIM2_CH4/PA3 | TIM3_CH1/PA6、TIM3_CH2/PA7 | TIM3 |

每一路使用两个 PWM 通道控制方向：控制量为正时使用第一个通道，为负时使用第二个通道，零值关闭两个通道。PWM ARR 为 999，最终输出限制在 `-999` 到 `999`。

> “正方向”由电机线和编码器 A/B 相接线决定。若某轴正 PWM 导致位置反馈为负，交换该电机的两根电机线，或交换编码器 A/B 相；不要同时修改两处。

## CAN 通信协议

所有帧均为 11 位标准数据帧，DLC 固定为 8。多字节整数采用**大端序**（高字节在前）。

### 主控 → 驱动板：双轴目标位置

CAN ID：`0x113`。

| 字节 | 类型 | 字段 | 说明 |
|---|---|---|---|
| 0–3 | `int32_t` | Pitch 目标位置 | 编码器累计脉冲，含四倍频和减速箱换算后的计数 |
| 4–7 | `int32_t` | Yaw 目标位置 | 编码器累计脉冲，含四倍频和减速箱换算后的计数 |

示例：目标 Pitch 为 `1320` 脉冲、Yaw 为 `-660` 脉冲时，数据为：

```text
00 00 05 28  FF FF FD 6C
```

这里的目标位置是上电后从 0 开始累计的相对位置，不是绝对角度。若主控使用角度，应先按机械传动比和编码器参数换算成脉冲：

```text
PULSES_PER_SHAFT_REV = 11 × 4 × 30 = 1320 脉冲/输出轴一圈
目标脉冲 = 目标输出轴圈数 × 1320
```

### 驱动板 → 主控：速度反馈

CAN ID：`0x206`。

| 字节 | 类型 | 字段 | 说明 |
|---|---|---|---|
| 0–1 | `int16_t` | Pitch 速度 | 本 10 ms 周期的编码器增量脉冲 |
| 2–3 | `int16_t` | Yaw 速度 | 本 10 ms 周期的编码器增量脉冲 |
| 4–7 | — | 预留 | 固定为 0 |

该帧的速度字段是周期增量脉冲，不是 RPM。按当前参数换算：

```text
PULSE_TO_RPM_COEF = 60 / (0.01 × 1320) ≈ 4.54545 RPM/脉冲
输出轴 RPM = 周期增量脉冲 × 4.54545
```

### 驱动板 → 主控：位置反馈

CAN ID：`0x207`。

| 字节 | 类型 | 字段 | 说明 |
|---|---|---|---|
| 0–3 | `int32_t` | Pitch 当前位置 | 编码器累计脉冲 |
| 4–7 | `int32_t` | Yaw 当前位置 | 编码器累计脉冲 |

反馈位置与下发目标使用相同单位和字节序。输出轴圈数为 `累计脉冲 / 1320`。

## 软件控制流程

```text
主控发送 0x113
    → CAN FIFO0 中断
    → HAL_CAN_RxFifo0MsgPendingCallback()
    → appProtocolParse()
    → 保存 Pitch/Yaw 目标位置

每 10 ms
    → 读取 TIM1/TIM3 编码器
    → 得到累计位置和周期增量
    → 位置 PID：位置误差 → 目标速度 RPM
    → 速度 PID：速度误差 → PWM 控制量
    → TIM2 双 PWM 输出
    → 发送 0x206 速度反馈和 0x207 位置反馈
```

两个轴均使用串级闭环：

1. 位置外环使用位置式 PID，输入为编码器累计脉冲，输出为目标速度 RPM；
2. 速度内环使用增量式 PID，输入为 10 ms 编码器增量换算出的 RPM，输出为 PWM 占空比。

默认参数位于 [App/app_control.h](App/app_control.h)。位置环输出限幅默认为 ±120 RPM，速度环输出限幅为 ±999 PWM。实际 Pitch/Yaw 机械结构的负载、摩擦和减速箱效率不同，参数需要在架空、低速条件下逐步调整。

## Ozone 调试

固件提供一个全局调试对象 `volatile DebugData debug_data`。在 Ozone 的 Watch 窗口中添加：

```text
debug_data
```

CAN 诊断数据位于全局对象 `can_debug_data`，也可直接加入 Ozone Watch。重点观察：

| 字段 | 含义 |
|---|---|
| `filter_status`、`start_status`、`notification_status` | 均应为 `0`，分别表示过滤器、CAN 启动、接收通知配置成功 |
| `tx_attempts`、`tx_accepted` | 应每 10 ms 左右增长；`tx_accepted` 只表示进入发送邮箱 |
| `tx_errors`、`error_callbacks`、`hal_error_code` | 应保持 `0`；非零时检查 ACK、波特率、收发器和接线 |
| `rx_frames`、`rx_ctrl_frames` | 收到帧后增长；收到主控 `0x113` 时 `rx_ctrl_frames` 应增长 |
| `rx_last_id`、`rx_last_dlc`、`rx_last_data[8]` | 最近一帧的 ID、长度和数据 |

只有 `tx_accepted` 增长不能单独证明总线通信正常；还要确认没有错误回调，并用另一节点或 CAN 分析仪观察 ACK/帧。

展开 `motor[0]` 可观察 Pitch，展开 `motor[1]` 可观察 Yaw。

| 字段 | 可写 | 说明 |
|---|---:|---|
| `pos_kp`、`pos_ki`、`pos_kd` | 是 | 位置外环 PID |
| `pos_max_out` | 是 | 位置环目标速度限幅，单位 RPM |
| `pos_max_integral` | 是 | 位置环积分限幅 |
| `spd_kp`、`spd_ki`、`spd_kd` | 是 | 速度内环 PID |
| `spd_max_out` | 是 | 速度环 PWM 限幅 |
| `spd_max_integral` | 是 | 速度 PID 保留参数 |
| `spd_deadband` | 是 | 速度误差死区，单位 RPM；误差绝对值小于该值时按 0 处理 |
| `spd_filter_alpha` | 是 | 速度低通滤波系数，范围 `(0,1]`；越小越平滑，`1` 为不滤波 |
| `debug_mode` | 是 | `0` 使用 CAN 位置；`1` 直接速度；`2` 使用 Ozone 位置 |
| `motor[i].target_pos_pulse` | 否 | 目标位置 |
| `motor[i].current_pos_pulse` | 否 | 当前累计位置 |
| `motor[i].current_pos_rev` | 否 | 当前输出轴圈数 |
| `motor[i].current_pos_angle_deg` | 否 | 当前输出轴角度，单位为度 |
| `motor[i].pos_error` | 否 | 位置环误差 |
| `motor[i].target_speed_rpm` | 否 | 位置环生成的目标速度 |
| `motor[i].current_speed_rpm` | 否 | 当前速度采样 |
| `motor[i].delta_pulse` | 否 | 当前 10 ms 增量脉冲 |
| `motor[i].speed_error` | 否 | 速度环误差 |
| `motor[i].motor_duty` | 否 | 最终 PWM 控制量 |
| `motor[i].total_pulse` | 否 | 编码器累计脉冲 |
| `motor[i].debug_target_angle_deg` | 是 | Debug 位置模式下的目标角度，单位为度 |
| `motor[i].debug_target_speed_rpm` | 是 | Debug 模式下的目标速度，单位 RPM |

Debug 直接设速步骤：

1. 在 Ozone Watch 中加入 `debug_data` 并展开 `motor[0]`（Pitch）或 `motor[1]`（Yaw）。
2. 将 `debug_data.debug_mode` 写为 `1`。
3. 写入对应的 `debug_target_speed_rpm`，例如 `10.0`；下一个 10 ms 控制周期会由速度环调节 PWM。
4. 停止测试时将目标 RPM 写为 `0.0`，再将 `debug_mode` 写为 `0`。

Debug 位置模式：将 `debug_data.debug_mode` 写为 `2`，再写入
`debug_data.motor[0].debug_target_angle_deg` 或
`debug_data.motor[1].debug_target_angle_deg`。代码按 `1320 脉冲 = 360 度` 自动换算，位置环会将位置误差转换为目标速度，再由速度环调节 PWM。

`current_speed_rpm` 是编码器在最近一个 10 ms 周期内计算出的实际速度；电机静止时读到 `0` 是正常现象。若整个 `debug_data` 不刷新，请确认 Ozone 已重新加载并下载当前构建生成的 ELF/HEX/BIN，而不是只打开旧的调试会话。

修改参数后会在下一个 10 ms 控制周期生效。重新编译后必须将新固件下载到板上，再观察运行时变量；仅生成 ELF 不会改变目标板中的旧固件。

## 与现有主控工程的同步要求

当前新板协议把 `0x113` 定义为两个 `int32_t` 位置字段。现有主控工程中若仍使用旧的“Pitch 位置 + 摩擦轮占空比 + 拨弹盘占空比”格式，则只能提供 Pitch，不能正确提供 Yaw；主控端必须同步修改为：

```c
data[0] = (uint8_t)(pitch_target_position >> 24);
data[1] = (uint8_t)(pitch_target_position >> 16);
data[2] = (uint8_t)(pitch_target_position >> 8);
data[3] = (uint8_t)pitch_target_position;
data[4] = (uint8_t)(yaw_target_position >> 24);
data[5] = (uint8_t)(yaw_target_position >> 16);
data[6] = (uint8_t)(yaw_target_position >> 8);
data[7] = (uint8_t)yaw_target_position;
```

主控接收反馈时应监听 `0x206` 和 `0x207`。`0x206` 的速度数据是周期脉冲，`0x207` 的位置数据是累计脉冲。

## 构建、下载和检查

### 构建

要求：GNU Arm Embedded GCC 和 `mingw32-make`。在 PowerShell 中执行：

```powershell
mingw32-make -j2
```

成功后生成：

```text
build/Yaw_Pitch_Motor_Driver.elf
build/Yaw_Pitch_Motor_Driver.hex
build/Yaw_Pitch_Motor_Driver.bin
```

### 上电前检查

1. 断开电机负载，确认 520 接在 Pitch 的 PWM/编码器端口，370 接在 Yaw 的 PWM/编码器端口。
2. 确认电机驱动电源和 MCU 3.3 V 共地。
3. 确认 CAN 收发器、CANH/CANL、500 kbit/s 和终端电阻。
4. 先发送目标 `0,0`，确认两个 PWM 均为 0。
5. 再发送很小的正负位置目标，观察反馈位置方向是否与约定一致。

CAN 正常模式需要总线上有能 ACK 的其他节点；只有一个设备和被动分析仪时出现 ACK 错误，并不能单独证明固件有问题。编译通过也不等于 CAN 接收、编码器方向和电机实际运动已经通过硬件验证。

## 文件结构

```text
Yaw_Pitch_Motor_Driver/
├── board_config.h                 # CAN ID
├── Yaw_Pitch_Motor_Driver.ioc     # CubeMX 配置
├── Makefile                       # GNU Arm GCC 构建配置
├── STM32F103XX_FLASH.ld           # 链接脚本
├── App/
│   ├── app_control.h/c            # 双轴位置-速度串级 PID
│   ├── app_protocol.h/c            # CAN 解析和反馈
│   └── app_debug.h/c              # Ozone 调试数据
├── Bsp/
│   ├── bsp_can.h/c                # CAN 收发封装
│   ├── bsp_encoder.h/c            # TIM1/TIM3 编码器
│   └── bsp_motor.h/c              # TIM2 双 PWM 电机输出
├── Core/                          # HAL、时钟、GPIO、CAN、TIM 和中断
└── README.md
```
