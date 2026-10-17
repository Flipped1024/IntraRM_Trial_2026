# Yaw/Pitch Motor Driver

基于 STM32F103C6T6 的双轴电机驱动板。Pitch 使用 520 电机，Yaw 使用 370 电机。正常模式下主控通过 CAN `0x114` 下发两个有符号 PWM duty；编码器同时提供速度和位置反馈。

## CAN 协议

所有帧 DLC 固定为 8，多字节整数采用大端序。

`0x114` 主控到驱动板：字节 0–1 为 Pitch duty，字节 4–5 为 Yaw duty，其他字节保留为 0。松杆时对应目标为 0。

`0x208` 驱动板到主控：字节 0–1 为 Pitch、2–3 为 Yaw 的 10 ms 编码器增量脉冲。

`0x209` 驱动板到主控：字节 0–3 为 Pitch、4–7 为 Yaw 的累计编码器脉冲。

## 控制流程

```text
CAN 速度目标 → 位置/速度目标选择 → 编码器测速 → 速度 PID → PWM
```

正常模式直接使用 CAN duty。将 `debug_data.debug_mode` 改为 `2` 后，Yaw 使用级联位置/速度环：`motor[1].debug_target_angle_deg` 是相对上电编码器零点的角度（度），可在 Ozone 中实时修改。位置环输出速度目标，速度环输出 PWM；Pitch 在该模式仍保持 CAN duty。模式 `1` 可单独调试 Yaw 速度环。编码器分辨率会影响低速反馈的量化精度，需结合实际机械负载调整 PID。

Yaw 的板级方向已在 `bspMotorSetSpeed()` 中做逻辑反向：正的逻辑目标应对应正的编码器反馈。

## Ozone 调试

观察 `volatile DebugData debug_data`：

- `motor[0]` 为 Pitch，`motor[1]` 为 Yaw。
- `target_speed_rpm` 为当前速度环目标。
- `current_speed_rpm`、`raw_speed_rpm`、`delta_pulse` 为编码器速度观测。
- `motor_duty` 为速度 PID 最终 PWM 输出。
- `debug_mode=0/1/2` 分别表示 CAN 开环、Yaw 速度调试、Yaw 位置调试。
- 位置调试时修改 `debug_data.motor[1].debug_target_angle_deg`，并观察 `current_pos_angle_deg`、`pos_error` 和 `motor_duty`。
- `can_debug_data` 用于观察 CAN 收发和错误状态。

## 构建

```powershell
mingw32-make -j2
```

产物位于 `build/`。编译通过不能替代 CAN ACK、编码器方向和实际电机动作验证。


## 混合线序版本（Pitch 正向 / Yaw 反向）

本工程仅适用于 Pitch 正向线序、Yaw 反向线序。Pitch 的 PWM 和编码器方向保持原版，Yaw 的 PWM 和编码器方向相对原版取反。CAN、开环控制与调试功能均保持不变。

- 两轴正向：Yaw_Pitch_Motor_Driver
- 两轴反向：Yaw_Pitch_Motor_Driver_Reverse
- Pitch 正向、Yaw 反向：Yaw_Pitch_Motor_Driver_Yaw_Reverse

请按实车线序选择固件；混合版本尚需烧录后验证方向。
