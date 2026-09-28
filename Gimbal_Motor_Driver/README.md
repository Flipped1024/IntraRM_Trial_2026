# Gimbal Motor Driver

基于 STM32F103C6T6 的发射机构驱动板固件。新版硬件中 Pitch 云台轴由其他控制板负责，本板不再包含 Pitch 电机、编码器或位置/速度 PID，仅保留两路摩擦轮和一路拨弹盘。本文档按当前新版板线序记录。

## 硬件与电机

| 电机 | 用途 | 控制方式 | 接口 |
|---|---|---|---|
| 370 电机 1 | 摩擦轮 1 | 单 PWM + GPIO 方向 | PWM：TIM2 CH1 PA0（比较值 0 最高速、999 停止），方向：PA1（低电平接负极切换方向） |
| 370 电机 2 | 摩擦轮 2 | 单 PWM + GPIO 方向 | PWM：TIM2 CH3 PA2（比较值 0 最高速、999 停止），方向：PA3（低电平接负极切换方向） |
| N20 电机 | 拨弹盘 | 双 PWM | TIM3 CH1 PA6 + CH2 PA7 |

Pitch 相关的 TIM1 编码器和对应控制逻辑均已移除；TIM3 现在专用于 N20 双 PWM。

370 电机的两个控制口作用不同：PWM 口为低有效，比较值 `0` 时最高速，比较值 `999` 时速度为 `0`；GPIO 口接负极（低电平）只用于切换方向，不控制转速。程序上电时将 PA1/PA3 拉高，摩擦轮停止时将 PWM 比较值写为 `999`。两个摩擦轮机械安装方向相反，因此 CAN 控制中 2 号摩擦轮使用与 1 号相反的占空比符号；代码中的 `direction_invert_` 用于匹配这一机械方向差异。

### CubeMX 重新生成注意事项

- `Gimbal_Motor_Driver.ioc` 是当前线序的配置源：PA1/PA3 配置为 370 方向 GPIO，PA6/PA7 配置为 TIM3_CH1/CH2。
- `Core/Inc/main.h`、`Core/Inc/tim.h`、`Core/Src/tim.c` 中的 `USER CODE BEGIN/END` 注释保留了同一份线序说明，重新生成后请检查这些注释和 `.ioc` 是否一致。
- 应用层实际通道绑定在 `Bsp/bsp_motor.c`，CubeMX 不会生成或覆盖该文件。

## CAN 协议

CAN 总线速率为 500 kbps，使用标准帧。

### 控制帧：0x113，8 字节

| 字节 | 字段 | 说明 |
|---|---|---|
| 0–3 | 保留 | 原 Pitch 目标位置字段；本板忽略 |
| 4–5 | `fric_duty` | 两个摩擦轮占空比，范围 `[-999, 999]`；两路方向相反 |
| 6–7 | `feed_duty` | 拨弹盘占空比，范围 `[-999, 999]` |

### 状态帧：0x206，8 字节

状态帧仍按 8 字节发送，但当前全部为 0。Pitch 状态不再由本板采集或上报。

## 控制流程

主循环每 10 ms 执行一次 `appControlLoop()`。正常模式下，CAN 回调直接更新摩擦轮和拨弹盘；调试模式 `debug_mode = 1` 下，使用 `debug_data` 中的三个占空比值。

## Ozone 调试字段

可在 Watch 窗口添加全局变量 `debug_data`：

| 字段 | 作用 |
|---|---|
| `fric1_duty` / `fric2_duty` | 手动设置两路摩擦轮占空比 |
| `feed_duty` | 手动设置拨弹盘占空比 |
| `debug_mode` | `0` 为 CAN 控制，`1` 为手动调试 |
| `can_*` | CAN 接收与错误诊断信息 |

## 工程结构

- `Bsp/bsp_motor.h/c`：三路电机 PWM 与方向控制
- `Bsp/bsp_can.h/c`：CAN 收发封装
- `App/app_protocol.h/c`：CAN 协议解析与状态上报
- `App/app_control.h/c`：发射机构控制调度
- `App/app_debug.h/c`：Ozone 调试数据
- `Core/`：STM32 HAL 与 CubeMX 生成代码

## 构建

工程使用 Makefile 和 ARM GCC：

```text
mingw32-make -j2
```

生成文件位于 `build/`。烧录前请确认 CAN 收发器、摩擦轮方向和拨弹盘 PWM 接线与上表一致。
