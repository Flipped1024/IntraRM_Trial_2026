# PC_Controller 开发者说明

这是 `Main_Board_L431RCT6` 的原生 Windows C 键盘遥控工具。通过 CH340 向 UART1 发送现有 14 字节手柄协议，不依赖 Python 或 .NET。手柄 USART3 的数据格式及运动、云台、摩擦轮、拨弹映射保持原有实现。

用户接线与操作见 [使用说明.md](使用说明.md)。

## 目录与职责

```text
IntraRM_Trial_2026/
├─ Main_Board_L431RCT6/       STM32 固件项目
└─ PC_Controller/
   ├─ pc_controller.c        Win32 窗口、键盘、串口、停止、改键与 INI 配置
   ├─ pc_protocol.h          输入状态到 14 字节帧的映射及 CRC
   ├─ pc_keybind.h           自定义按键冲突检查
   ├─ pc_ports.h             SetupAPI 串口枚举、设备描述、CH340 标识
   ├─ compat/                TinyCC 缺少的 SetupAPI ABI 和 DLL 导入定义
   ├─ tests/                 HAL 替身；测试真实 app_remote.c
   ├─ build.ps1 / test.ps1
   ├─ RM_PC_Controller.exe   32 位兼容版
   ├─ RM_PC_Controller_x64.exe
   └─ keybindings.ini        用户修改按键时生成，不是构建输入

STM32 编译输出仅放在 `Main_Board_L431RCT6/build/`；电脑程序目录只放电脑程序、CH340 驱动及其源码/构建辅助文件。
```

## 协议

115200、8N1、无流控。每约 20 ms 发一帧；帧没有回传确认。

| 字节 | 含义 | 电脑映射 |
|---|---|---|
| 0–1 | 帧头 | `C3 3C` |wss
| 2 | 8 位序号 | 每帧递增并回绕 |
| 3 | flags | 启用为 `03`；停用为 `00` |
| 4 | int8 left_x | `(D-A) × amplitude` |
| 5 | int8 left_y | `(S-W) × amplitude` |
| 6 | int8 right_x | `(右-左) × 127` |
| 7 | int8 right_y | `(下-上) × 127` |
| 8 | LT | `0` |
| 9 | RT | 开火键按下 `255`，松开 `0` |
| 10 | D-pad | `0` |
| 11 | buttons_1 | bit0：摩擦轮/A；bit6：Q/LB；bit7：E/RB |
| 12 | buttons_2 | `0` |
| 13 | CRC | 前 13 字节 CRC-8/ATM |

CRC 多项式 `0x07`、初值 `0`、不反射、无末尾异或；独立向量 `123456789 → F4`。幅度选择对应 31、63、95、127，默认索引 2（75%，数值 95）。只缩放 WASD；Q/E 按原手柄 LB/RB 的全量转向输入处理。

## 控制来源与固件修改

`App/Src/app_remote.c` 中两路串口共用解码器，UART3 保留循环 DMA 接收，UART1 使用单字节中断和独立组帧。`controller_state.uart_` 仍指向 UART3，保持原接收/恢复路径。

仲裁规则：

1. CRC 正确、flags 有效的 UART1 帧启用键盘优先权，`controller_pc_active_ = 1`。
2. 优先权存在且电脑未超时时，忽略 USART3 对有效控制状态的覆盖，避免每次手柄周期把电脑输入归零。
3. flags 无效的 UART1 帧释放键盘，先设置 LOST；后续手柄有效帧经过 RECOVERING 恢复控制。
4. 没有键盘优先权时，电脑停用帧直接忽略，不更新有效手柄数据或时间戳。因此串口连接、失焦期间不断发送的停用帧不会干扰手柄。
5. 电脑超过 `CONTROLLER_TIMEOUT_MS`（150 ms）未更新时，释放优先权；有效手柄帧可直接触发来源恢复，不必等待主循环先处理超时。无有效输入则保持既有停止路径。
6. 新来源、丢失恢复必须完成主循环的一次安全输出；连续多个恢复帧不能提前将 RECOVERING 改成 NORMAL。

USART3 手柄轴、RT、A、LB/RB 的解码和下游 `app_gimbal.c` / 底盘运动逻辑没有被替换；不合并两路的轴或按键。Q/E 使用现有 `main.c` 中 `(LB-RB) × CHASSIS_MAX_ANGULAR_SPEED_RADPS` 转向路径，不另造 CAN 命令。

UART1 初始化、RX 完成/错误回调位于 `main.c` USER CODE；USART1 IRQ 与声明位于 `stm32l4xx_it.c/.h` USER CODE。原 UART1 字符 `1 → 2` 测试被移除。PB6/PB7 与 115200 已在现有 `.ioc` / `usart.c` 配置；NVIC 在应用初始化启用。若以后用 CubeMX 自动生成 USART1 IRQ，应去除保留的同名 USER CODE 函数，避免重复定义。

UART1 帧间字节间隔超过 20 ms 会丢弃残帧。CRC 错误采用滑动帧头重同步；UART 错误会中止未完成接收并重新挂接 IT。错误帧不刷新控制时间戳。

`g_debug_.mode_ != DEBUG_MODE_NONE` 的原手动测试路径保持优先；该模式不受键盘正常遥控入口约束。电脑界面的停止只停止键盘并交还手柄，不能宣称整车急停。

## Windows 界面与配置

- 用 `GetAsyncKeyState` 约每 20 ms 采集当前键态，仅在启用且前台时生效；启动、失焦、连接失败、发送调度间隔超过 100 ms 均停用键盘。
- `WriteFile` 使用同步串口和 30 ms 写超时；失败或部分写入即断开。连接/停用期间发送 flags=0，关闭前尝试发送一次停用帧。
- 改键通过 `WM_KEYDOWN/WM_SYSKEYDOWN` 捕获，并区分左右 Shift/Ctrl/Alt；先停用键盘，修改后要求重新启用。保留移动键、Q/E、方向键、Esc、Windows 键；拒绝两个功能共用一个键。
- 配置路径使用 EXE 所在目录，格式为 `[Keys]` 下的 `Fire`、`Friction`，值为十进制 Win32 VK。无配置时使用 Space / 左 Shift。非法或冲突配置回退到默认值；保存失败时仅本次有效并提示。
- SetupAPI 按 Ports 类枚举当前设备，从设备注册表 `PortName` 取真实 COM 名称，从设备属性读取友好名称、描述和硬件 ID。CH340/CH341 名称或 `VID_1A86&PID_7523` 用于标识芯片，不能识别主控身份。没有类设备描述的 COM 口通过 `QueryDosDevice` 补充。
- 列表的设备描述和打开串口的 COM 编号分开存储，不能把整个显示字符串传给 `CreateFile`。

## 构建

从本目录执行：

```powershell
# MinGW GCC：x64
./build.ps1

# 同时生成 TinyCC win32 兼容版
./build.ps1 -TccPath 'C:\你的目录\tcc\tcc.exe'

# 编译主控：从其目录执行
mingw32-make -j2
```

TinyCC 0.9.27 win32 官方发布：<https://download.savannah.nongnu.org/releases/tinycc/tcc-0.9.27-win32-bin.zip>。`compat` 中的最小 SetupAPI 声明与导入 `.def` 是其构建支持文件，运行 EXE 不需要这些文件。32 位程序只依赖 Windows 自带的 `msvcrt/setupapi/advapi32/kernel32/user32/gdi32`；x64 构建使用系统 UCRT，适用于 Windows 10/11 x64。其他系统和不同 CH340 驱动版本仍需实际兼容验证。

## 测试

```powershell
./test.ps1
```

测试直接编译项目真实 `Main_Board_L431RCT6/App/Src/app_remote.c`，HAL 替身仅模拟收字节与时间。覆盖 CRC 独立向量、WASD/方向、Q/E→LB/RB、松键、反向抵消、键盘优先、停用帧不干扰手柄、停用/超时自动交还手柄、安全恢复周期、截断/错帧重同步、UART 错误及按键冲突检查。

两种 EXE 都支持 `--self-test`：创建真实窗口，检查默认 75%、未连接启用限制、串口描述枚举、改键冲突/取消、临时 INI 保存与重读、停止和退出，不打开串口，也不改用户配置。PowerShell 等待程序完成后读取退出码，0 表示通过：

```powershell
$p = Start-Process .\RM_PC_Controller.exe -ArgumentList '--self-test' -WindowStyle Hidden -PassThru
$p.WaitForExit()
$p.ExitCode
```

本次主控与两个电脑 EXE 已编译通过；协议测试和两个窗口自检通过。主控仍有现有云台标定函数未使用警告，本次未改这些函数。未烧录或验证真实 UART/CAN/电机动作，软件测试不构成硬件验证。

Ozone 可观察 `controller_pc_active_`、`controller_pc_uart_errors_`、`controller_state` 的轴/按键/flags/时间戳/安全状态，以及 `gimbal_` 的摩擦轮和拨弹状态。手柄优先期、键盘优先期分别检查，再验证实际 CAN 与运动。
