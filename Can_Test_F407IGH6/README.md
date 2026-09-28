# Can_Test_F407IGH6

STM32F407IGH6 的 CAN 电机驱动测试工程，同时保留底盘驱动板、旧云台驱动板和新的 Yaw/Pitch 双轴位置驱动板三套测试协议。

## 测试目标切换

在 `Core/Inc/app_protocol.h` 中修改 `CAN_TEST_MODE`：

```c
#define CAN_TEST_MODE CAN_TEST_MODE_YAW_PITCH
```

可选值为：

- `CAN_TEST_MODE_CHASSIS`（`0`）：发送原底盘控制帧 `0x111`，使用两路目标转速参数。
- `CAN_TEST_MODE_GIMBAL`（`1`）：发送旧云台控制帧 `0x113`，使用 Pitch、摩擦轮和拨弹盘参数。
- `CAN_TEST_MODE_YAW_PITCH`（`2`）：发送新 Yaw/Pitch 双轴位置帧 `0x113`，使用两个位置脉冲参数。
- `CAN_TEST_MODE_LINK`（`3`）：与 L431 主控联机测试；F407 周期发送 `0x222`~`0x225`，同时接收 L431 的 `0x111`、`0x112`。

默认仍为旧云台模式，以保持原有配置行为。旧的 `CAN_TEST_GIMBAL=0/1` 配置方式仍兼容，但要选择 Yaw/Pitch 模式应直接定义 `CAN_TEST_MODE`。

四种模式均为标准数据帧、8 字节、10 ms（100 Hz）调度周期。联机模式每个周期发送 4 帧反馈报文。

联机模式下，在 Ozone 中观察 `debug_data`：`tx_ok_count` 应持续增加，`rx_count` 应收到 L431 的 `0x111`/`0x112`，`last_rx_id` 应在这两个 ID 间变化。F407 的 `0x222`~`0x225` 报文会被 L431 的 CAN 过滤器接收，从而为 L431 发送 `0x111`/`0x112` 提供 ACK。

## Ozone 调试

在 Ozone 的 Watch / Expressions 窗口中添加一个变量：

```text
debug_data
```

展开 `debug_data` 后即可一次看到和修改全部测试内容：

| 字段 | 说明 |
|---|---|
| `test_mode` | 当前模式，0 底盘、1 旧云台、2 Yaw/Pitch，只读 |
| `chassis_motor1_speed` / `chassis_motor2_speed` | 底盘两路目标转速 |
| `gimbal_pitch_target_pos` | 云台 Pitch 目标位置，单位 0.01° |
| `gimbal_fric_duty` / `gimbal_feed_duty` | 云台摩擦轮 / 拨弹盘占空比 |
| `yaw_pitch_pitch_target_pos` | Yaw/Pitch 板 Pitch 目标位置，单位编码器累计脉冲 |
| `yaw_pitch_yaw_target_pos` | Yaw/Pitch 板 Yaw 目标位置，单位编码器累计脉冲 |
| `tick_counter` / `send_flag` | 10 ms 周期调度状态 |
| `tx_id` / `tx_data[0..7]` | 最近一次实际准备发送的 CAN ID 和数据 |
| `can_error_code` / `can_state` | CAN 错误码和状态镜像 |
| `tx_ok_count` / `tx_fail_count` | `HAL_CAN_AddTxMessage()` 成功放入邮箱 / 失败次数 |

建议先保持当前模式的目标参数为 0，运行后确认 `debug_data.send_flag` 周期变化、`tx_id` 和 `tx_data` 内容正确。底盘模式下应看到 `tx_id = 0x111`；旧云台和 Yaw/Pitch 模式下应看到 `tx_id = 0x113`。

确认 CAN 发送建议分三层：

1. **程序层**：下载新固件后，在 Ozone 观察 `debug_data.test_mode == 2`、`tx_id == 0x113`，并确认 `tx_ok_count` 每约 10 ms 增加、`tx_fail_count` 不增加。对于 Pitch=0、Yaw=0，线上 8 字节应为 `00 00 00 00 00 00 00 00`。
2. **总线层**：用 CAN 分析仪连接 CANH、CANL 和 GND，设置 500 kbps、标准帧，观察是否周期收到 `0x113`、DLC=8；把 Ozone 中两个目标位置改成容易识别的值，例如 Pitch=1、Yaw=2，预期数据为 `00 00 00 01 00 00 00 02`。
3. **接收层**：接上双轴位置板后，观察其接收计数、状态反馈或实际位置变化。`tx_ok_count` 只能证明 STM32 已把帧放入 CAN 发送邮箱，不能单独证明总线有 ACK、对端已解析或电机已运动。没有第二个 CAN 节点时，正常模式出现 ACK/总线错误并不一定表示打包代码错误。

发送函数每 10 ms 执行一次，不建议长时间在发送函数或 `HAL_SYSTICK_Callback()` 设置断点；需要检查数据时短暂停止程序，查看 `tx_id` 和 `tx_data` 即可。

## 硬件连接与 CAN 配置

| 项目 | 配置 |
|---|---|
| F407 CAN1_RX / CAN1_TX | PD0 / PD1 |
| CAN 模式 | `CAN_MODE_NORMAL` |
| 波特率 | 500 kbps |
| APB1 / Prescaler / BS1 / BS2 | 42 MHz / 6 / 11 TQ / 2 TQ |
| 帧格式 | 标准数据帧，11-bit ID |

CANH、CANL 接对应驱动板 CANH、CANL，并共地。总线两端各接 120Ω 终端电阻。

波特率：`42 MHz / 6 / (1 + 11 + 2) = 500 kbps`。

## 底盘测试协议（原有功能）

底盘模式使用 CAN ID `0x111`，大端发送两路 `int16_t` 目标转速，后 4 字节填 0：

| 字节 | 内容 |
|---|---|
| 0~1 | 电机 1 目标转速，RPM |
| 2~3 | 电机 2 目标转速，RPM |
| 4~7 | 保留，填 0 |

参数位于 `Core/Src/app_protocol.c`：

```c
volatile int16_t chassis_motor1_speed = 100;
volatile int16_t chassis_motor2_speed = -50;
```

该协议说明与 [Chassis_Motor.md](Chassis_Motor.md) 对应。

## 旧云台测试协议

云台模式使用 CAN ID `0x113`，8 字节大端数据：

| 字节 | 字段 | 类型 | 单位 / 范围 |
|---|---|---|---|
| 0~3 | Pitch 目标位置 | `int32_t` | 0.01° |
| 4~5 | 摩擦轮占空比 | `int16_t` | `[0, 999]` |
| 6~7 | 拨弹盘占空比 | `int16_t` | `[-999, 999]` |

参数位于 `Core/Src/app_protocol.c`：

```c
volatile int32_t gimbal_pitch_target_pos = 0;
volatile int16_t gimbal_fric_duty = 0;
volatile int16_t gimbal_feed_duty = 0;
```

例如 `9000 / 300 / -100` 的数据为：`00 00 23 28 01 2C FF 9C`。
云台协议详情见 [Gimbal_Motor.md](Gimbal_Motor.md)。驱动板状态反馈 ID 为 `0x206`，本测试程序目前只发送、不接收。

## Yaw/Pitch 双轴位置测试协议

该模式用于测试 [Yaw_Pitch_Motor_Driver](../Yaw_Pitch_Motor_Driver/README.md)。使用 CAN ID `0x113`，8 字节大端数据：

| 字节 | 字段 | 类型 | 单位 |
|---|---|---|---|
| 0~3 | Pitch 目标位置 | `int32_t` | 编码器累计脉冲 |
| 4~7 | Yaw 目标位置 | `int32_t` | 编码器累计脉冲 |

新驱动板的参数为 11 PPR、四倍频、30:1 减速箱，因此输出轴一圈为 `11 × 4 × 30 = 1320` 脉冲。位置目标是驱动板上电后从 0 开始累计的相对位置，不是角度值。

参数位于 `Core/Src/app_protocol.c` 初始化的 `debug_data` 中：

```c
debug_data.yaw_pitch_pitch_target_pos = 0;
debug_data.yaw_pitch_yaw_target_pos = 0;
```

示例：Pitch 为 `1320` 脉冲、Yaw 为 `-660` 脉冲时，发送数据为：

```text
00 00 05 28 FF FF FD 6C
```

新驱动板周期上报速度帧 `0x206` 和位置帧 `0x207`；本测试工程当前只发送目标位置，不接收反馈。

## 软件流程

```text
SysTick_Handler（1 ms） -> 每 10 ms 置 send_flag
main() -> APP_Protocol_Task()
       -> 根据 CAN_TEST_MODE 选择协议并打包
       -> 发送 0x111（底盘）或 0x113（旧云台/Yaw-Pitch）
```

## 编译与下载

```bash
make clean
make -j4
```

在 Ozone/J-Link 中加载 `build/Can_Test_F407IGH6.elf`。正常模式下必须连接对应驱动板以提供 CAN ACK；单机验证可临时使用 `CAN_MODE_LOOPBACK`，真实测试前恢复为 `CAN_MODE_NORMAL`。
