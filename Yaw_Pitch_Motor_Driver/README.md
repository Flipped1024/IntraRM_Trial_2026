# Yaw/Pitch Motor Driver

基于 STM32F103C6T6 的双轴速度闭环电机驱动板。Pitch 使用 520 电机，Yaw 使用 370 电机。主控通过 CAN `0x114` 下发两个有符号速度目标（RPM），驱动板用编码器反馈和速度 PID 生成 PWM。

## CAN 协议

所有帧 DLC 固定为 8，多字节整数采用大端序。

`0x114` 主控到驱动板：字节 0–1 为 Pitch 目标速度 RPM，字节 4–5 为 Yaw 目标速度 RPM，其他字节保留为 0。松杆时对应目标为 0。

`0x208` 驱动板到主控：字节 0–1 为 Pitch、2–3 为 Yaw 的 10 ms 编码器增量脉冲。

`0x209` 驱动板到主控：字节 0–3 为 Pitch、4–7 为 Yaw 的累计编码器脉冲。

## 控制流程

```text
CAN 速度目标 → 位置/速度目标选择 → 编码器测速 → 速度 PID → PWM
```

正常模式使用 CAN 速度目标和速度 PID。`debug_data.debug_mode` 可用于 Ozone 直接速度或位置调试。编码器分辨率会影响低速反馈的量化精度，需结合实际机械负载调整 PID。

## Ozone 调试

观察 `volatile DebugData debug_data`：

- `motor[0]` 为 Pitch，`motor[1]` 为 Yaw。
- `target_speed_rpm` 为当前速度环目标。
- `current_speed_rpm`、`raw_speed_rpm`、`delta_pulse` 为编码器速度观测。
- `motor_duty` 为速度 PID 最终 PWM 输出。
- `can_debug_data` 用于观察 CAN 收发和错误状态。

## 构建

```powershell
mingw32-make -j2
```

产物位于 `build/`。编译通过不能替代 CAN ACK、编码器方向和实际电机动作验证。
