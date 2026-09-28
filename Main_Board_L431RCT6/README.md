# STM32L431RCT6 全向轮小车主控

本项目是运行于 STM32L431RCT6 的全向轮小车主控固件。主控通过 CAN 总线连接两块底盘电机驱动板和云台驱动板，并通过 ESP32-S3 手柄接收机控制底盘、Pitch 云台、摩擦轮和拨弹盘。

## 功能

- CAN1 标准帧通信：下发底盘电机目标速度，接收编码器速度和位置反馈，并与云台驱动板交换控制数据。
- USART3 + 循环 DMA + UART 空闲事件：接收 ESP32-S3 输出的 Xbox 手柄控制帧。
- 100 Hz 主控制循环：四轮全向运动学解算、底盘控制、Pitch 云台标定与控制、摩擦轮和拨弹盘控制。
- `g_debug_` 调试模式：支持底盘测试和 CAN 直接发送测试。

## 手柄通信

USART3 接收固定 14 字节二进制帧，通信参数为 115200、8N1、无硬件流控，协议详见[手柄接收机通信协议.md](手柄接收机通信协议.md)。

```text
C3 3C | sequence | flags | left_x | left_y | right_x | right_y |
left_trigger | right_trigger | dpad | buttons_1 | buttons_2 | crc8
```

CRC-8/ATM 校验正确且 `flags & 0x03 == 0x03` 时，控制数据才会生效；连续 150 ms 未收到有效帧时，底盘速度和发射机构进入安全状态。

主控中的全局结构体 `controller_state` 适合直接添加到 Ozone Watch 窗口查看：

| 字段 | 含义 |
|---|---|
| `left_stick_horizontal_` / `left_stick_vertical_` | 左摇杆水平/垂直原始值；左推、前推分别为负，范围 -127~127 |
| `right_stick_horizontal_` / `right_stick_vertical_` | 右摇杆水平/垂直原始值；右推、前推分别为负，范围 -127~127 |
| `left_trigger_` / `right_trigger_` | LT、RT 原始值，范围 0~255 |
| `fire_requested_` | RT 大于 0 时为 1，控制开火；RT 松开立即为 0 |
| `dpad_direction_` | 方向枚举：上=1、右=3、下=5、左=7 |
| `dpad_.up_pressed_` 等 | 各方向键的独立按下状态 |
| `buttons_.a_pressed_` 等 | A、B、X、Y、LB、RB 的独立按下状态 |
| `buttons_.back_pressed_` 等 | Back、Start、Xbox、左摇杆按下、右摇杆按下的独立状态 |
| `flags_` / `control_enabled_` | 链路标志和控制是否有效 |
| `connection_state_` | 0=断开，1=连接 |
| `safety_state_` | 0=正常，1=掉控保护，2=重连恢复中 |
| `valid_frame_count_` | CRC 正确的帧数量 |
| `crc_error_count_` | CRC 错误的候选帧数量 |
| `dropped_frame_count_` | 按 sequence 统计的丢帧数量 |

应用映射如下：左摇杆控制底盘平移，右摇杆水平控制底盘旋转，右摇杆垂直控制 Pitch；LB 按下沿切换摩擦轮，RT 按住开火、松开停火。底盘控制量采用 `vx` 向右为正、`vy` 向前为正、`vw` 逆时针为正，因此主控对前进和逆时针输入取反：左推时 `vx<0`，前推时 `vy>0`，右摇杆左推时 `vw>0`。

掉控超过 150 ms 后，主控进入安全状态：向板 1、板 2发送四个底盘电机的零速度目标，向云台板发送当前 Pitch 位置保持、摩擦轮关闭、拨弹盘关闭的指令。重新收到有效手柄帧后，先发送一个完整控制周期的安全零指令，再恢复正常控制。

## 底盘控制

底盘采用四轮全向（麦克纳姆轮）布局，四个电机由两块驱动板控制。主控在 100 Hz 控制周期内，根据 `vx`、`vy`、`vw` 进行逆运动学解算，并通过 CAN 向两块驱动板下发目标转速。

### 底盘参数

| 参数 | 符号 | 值 | 单位 |
|---|---|---:|---|
| 底盘半径 | `Lc` | 0.17888 | m |
| 车轮半径 | `R` | 0.0765 | m |
| 驱动板控制周期 | `T` | 10 | ms |
| 电机最大目标转速 | — | 178 | RPM |
| 满量程平移速度 | `vx` / `vy` | 0.3209 | m/s |
| 满量程旋转速度 | `vw` | 1.268 | rad/s |

### CAN 通信

底盘控制帧为标准帧、8 字节数据：

| CAN ID | 方向 | 内容 |
|---:|---|---|
| `0x111` | 主控 → 板 1 | FL、FR 目标转速 |
| `0x112` | 主控 → 板 2 | RL、RR 目标转速 |
| `0x222` | 板 1 → 主控 | FL、FR 速度反馈 |
| `0x223` | 板 1 → 主控 | FL、FR 累计位置 |
| `0x224` | 板 2 → 主控 | RL、RR 速度反馈 |
| `0x225` | 板 2 → 主控 | RL、RR 累计位置 |

控制帧字节布局：

| 字节 | 类型 | 内容 |
|---|---|---|
| 0~1 | `int16_t` | 电机 1 目标转速 |
| 2~3 | `int16_t` | 电机 2 目标转速 |
| 4~7 | — | 预留 |

目标转速字段按大端序发送：例如 `1 rpm` 的两个字节为 `00 01`，负值按
`int16_t` 补码发送；驱动板程序按此格式解析。

速度反馈帧的字节 0~3 为两个电机的编码器周期增量脉冲，位置反馈帧的字节 0~7 为两个电机的累计编码器脉冲。驱动板内部按照相同的控制周期和编码器参数换算速度。

### 逆运动学

```text
ω_FL = -((√2/2·(-vx - vy) + Lc·vw) / R · 60)
ω_FR = -((√2/2·(-vx + vy) + Lc·vw) / R · 60)
ω_RL = -((√2/2·( vx + vy) + Lc·vw) / R · 60)
ω_RR = -((√2/2·( vx - vy) + Lc·vw) / R · 60)
```

手柄摇杆的 `-127~127` 原始值会分别缩放到上述满量程平移、旋转速度。
当平移和旋转叠加导致任一轮超过 `178 rpm` 时，四个轮速按同一比例缩放，
保持运动方向和各轮之间的比例不变。

实际电机分配与接线为：

| 驱动板 | CAN ID | 电机 1 | 电机 2 |
|---|---:|---|---|
| 板 1 | `0x111` | 左前轮 `FL` | 右前轮 `FR` |
| 板 2 | `0x112` | 左后轮 `RL` | 右后轮 `RR` |

### 底盘控制流程

```text
手柄数据
    ↓
chassisSetTarget(vx, vy, vw)
    ↓
chassisUpdate()       逆运动学计算四个目标转速
    ↓
chassisSendCmd()      CAN 下发 0x111 / 0x112
    ↓
底盘驱动板速度闭环执行
```

掉控保护时，`chassisEmergencyStop()` 将 `target_` 和四个 `motor_cmds_` 全部置零，并持续向板 1、板 2发送零速度目标。

## CubeMX 关键配置

项目根目录中的 `Main_Board_L431RCT6.ioc` 已保存当前配置。关键项目如下：

- MCU：`STM32L431RCTx`；工具链：Makefile；启用 `Keep User Code`。
- USART3：Asynchronous，115200 Baud，8 Bits，No parity，1 Stop Bit，无硬件流控。
- USART3_RX：DMA Peripheral-to-Memory，Circular，Byte 对齐，Memory Increment，High priority。
- USART3 全局中断和对应 DMA 中断使能。
- CAN1：Normal mode，500 kbit/s，启用 RX FIFO0 message pending 中断。
- SYS：Debug 选择 Serial Wire。

重新生成代码时，请保留 `USER CODE BEGIN/END` 区域；应用逻辑位于 `App/`，串口 DMA/IDLE 接收启动和回调接入位于 `Core/Src/main.c` 的用户代码区域。

## CAN 电机分配

主控的电机分配与实际接线一致：

- 板 1：`FL / FR`
- 板 2：`RL / RR`

底盘电机控制帧使用 `0x111` 和 `0x112`，反馈帧使用 `0x222`~`0x225`；云台控制帧使用 `0x113`，反馈帧使用 `0x206`。具体字段和换算见 [Chassis.md](Chassis.md) 与 [Gimbal.md](Gimbal.md)。

## 软件结构

```text
Core/       CubeMX 生成的 HAL、启动代码和主循环入口
Bsp/        CAN、定时器、UART 空闲接收等板级封装
App/        底盘、云台、手柄协议和调试应用逻辑
```

## 编译与调试

```powershell
mingw32-make -j2
```

编译产物位于 `build/`。使用 J-Link/Ozone 加载 `build/Main_Board_L431RCT6.elf`，在 Watch 中添加 `controller_state` 即可观察手柄接收状态。
