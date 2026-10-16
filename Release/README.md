# 烧录包

每台车使用下面 5 份固件，每种烧录 35 块。整个 Release 目录可直接复制给烧录人员，无需源码或重新编译。

| 文件夹 | 对应板子 | CAN 控制帧 |
| --- | --- | --- |
| Left_Chassis_Motor | 左底盘驱动板，BOARD_ID=1 | 0x111 |
| Right_Chassis_Motor | 右底盘驱动板，BOARD_ID=2 | 0x112 |
| Fric_Feed_Motor_Driver | 摩擦轮与拨弹盘驱动板（原 Gimbal_Motor_Driver） | 0x113 |
| Yaw_Pitch_Motor_Driver | Yaw/Pitch 驱动板 | 0x114 |
| Main_Board_L431RCT6 | STM32L431RCT6 主控板 | — |

## 烧录步骤

1. 按表格选对板子和文件夹，连接 J-Link 的 SWD、GND、参考电压及板子电源。
2. 用 Ozone 打开文件夹中的 `.jdebug`，它会加载同目录下的 `.elf`。两个文件必须一起保留。
3. 选择当前连接的 J-Link，启动调试并下载程序；确认下载无错误后运行程序。
4. 断开调试器，重新上电，完成该板功能检查，再记录烧录结果。

左右底盘固件不能混用。发布包没有绑定特定 J-Link 序列号；若连接多只 J-Link，请明确选择接在目标板上的那只。首次使用本包先完成一整台车的验证，再继续批量烧录。

本包只包含正式上车的 5 块 STM32 板，不包含 F407 备用主控、测试工程、ESP32 接收机。

`manifest.json` 记录 ELF 和 Ozone 文件的 SHA256。固件源于已确认可上车的提交 `e03c195`，本次只调整工程名称和底盘板号的构建方式，未修改控制算法。

开发者可在仓库根目录运行 `powershell -ExecutionPolicy Bypass -File .\build-release.ps1` 重新生成全部包。左右底盘使用独立构建目录，构建不会修改 `board_config.h` 的默认板号。

## 反向线序固件

批量生产中若电机线束为完全反向的新线序（从右至左：电机负极、编码器正极、编码器 B 相、编码器 A 相、编码器负极、电机正极），请使用下表带 `_Reverse` 的固件；标准线序继续使用不带后缀的原固件。两种固件不能混用。

| 文件夹 | 适用板卡 | 说明 |
| --- | --- | --- |
| `Left_Chassis_Motor_Reverse` | 左底盘驱动板，`BOARD_ID=1` | 新反向线序 |
| `Right_Chassis_Motor_Reverse` | 右底盘驱动板，`BOARD_ID=2` | 新反向线序 |
| `Yaw_Pitch_Motor_Driver_Reverse` | Yaw/Pitch 驱动板 | 新反向线序 |

每个 `_Reverse` 文件夹仍包含同名 `.elf` 和 `.jdebug`，用 Ozone 打开同名 `.jdebug` 即可烧录对应 ELF。烧录前请按板卡位置和线序逐台核对。