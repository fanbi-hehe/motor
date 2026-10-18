# STM32F407 直流有刷电机 PID 速度控制（极简 Demo）

编码器测速 → PID → PWM 驱动电机，同时把「期望值 / 实际值」发给 VOFA+ 看波形。
**PID 参数可以直接在 VOFA+ 里发命令在线改**，不用重新烧录。

用户代码分 5 个模块，每个都很短：

| 文件 | 行数 | 内容 |
|---|---|---|
| `Core/Src/main.c` | ~400 | 调度、命令解析、显示（含 CubeMX 的时钟配置） |
| `Core/Src/motor.c` / `.h` | 58 | TB6612 输出（方向 + PWM） |
| `Core/Src/encoder.c` / `.h` | 37 | 编码器测速 |
| `Core/Src/pid.c` / `.h` | 61 | PID 算法 |
| `Core/Src/comm.c` / `.h` | 96 | 串口收发（JustFloat 帧 + 命令解析） |
| `Core/Src/oled.c` / `.h` | 238 | OLED 驱动（其中约 100 行是字库） |

模块之间只通过头文件里的函数打交道：`pid.c` 不知道电机存在，`encoder.c` 不碰 PWM。

---

## 1. 接线

| STM32 | 接到 | 说明 |
|---|---|---|
| PA0 | TB6612 **PWMA** | PWM 20 kHz，占空比 0~4199 |
| PA2 | TB6612 **AIN1** | 方向 |
| PA3 | TB6612 **AIN2** | 方向 |
| PA4 | TB6612 **STBY** | 高电平使能 |
| PE9 | 编码器 **A 相** | TIM1 正交解码 |
| PE11 | 编码器 **B 相** | |
| PB6 | OLED **SCL** | I2C1 400 kHz |
| PB7 | OLED **SDA** | |
| PA9 | USB-TTL **RX** | 115200 |
| PA10 | USB-TTL **TX** | |
| PA13/PA14 | ST-Link | 下载调试 |

电机电源、TB6612 电源、STM32 **必须共地**。

---

## 2. VOFA+ 设置

1. 数据引擎选 **JustFloat**
2. **通道数设成 4**（帧里不带通道数，填错就是乱曲线，这是最常见的坑）
3. 串口：115200，8 位，1 停止位，无校验

| 通道 | 内容 |
|---|---|
| ch0 | 期望转速（RPM） |
| ch1 | 实际转速（RPM） |
| ch2 | 输出占空比（±4199） |
| ch3 | 误差（RPM） |

---

## 3. 串口命令

在 VOFA+ 的发送框里直接输入，带不带换行都行：

| 命令 | 作用 |
|---|---|
| `500` | 目标转速 500 RPM（负数是反转） |
| `KP 2.5` | 改比例 |
| `KI 10` | 改积分 |
| `KD 0.02` | 改微分 |
| `START` / `STOP` | 闭环运行 / 停机 |
| `DUTY 800` | 开环直接给占空比（测接线用，不经过 PID） |

发完参数会显示在 OLED 第 4 行，第 6 行会显示最后收到的那条命令 —— 可以确认命令到底收到没有。

---

## 4. 要改东西，看这几个地方

| 想改什么 | 改哪里 |
|---|---|
| 编码器线数 / 减速比 / 转向 | `Core/Inc/encoder.h` 顶部的 4 个宏 |
| PID 上电默认值 / 目标转速 / 控制周期 | `Core/Src/main.c` 的 `【参数区】` |
| 电机 PWM 满量程 | `Core/Inc/motor.h` 的 `MOTOR_PWM_MAX` |
| 显示内容 | `Core/Src/main.c` 的 `Display_Task()` |
| 串口命令 | `Core/Src/main.c` 的 `HandleCommand()` |

```c
/* Core/Inc/encoder.h */
#define ENCODER_LINES   400.0f   /* 编码器线数 */
#define ENCODER_X4      4.0f     /* 4 倍频 */
#define GEAR_RATIO      1.0f     /* 减速比：编码器在电机轴填减速比，在输出轴填 1 */
#define ENCODER_DIR     1.0f     /* 转向反了改成 -1.0f */

/* Core/Src/main.c  【参数区】*/
#define CTRL_MS          10       /* 控制周期 ms */
#define DEFAULT_TARGET   500.0f   /* 上电目标转速 */
#define DEFAULT_KP       2.0f
#define DEFAULT_KI       10.0f
#define DEFAULT_KD       0.02f
#define OVERSPEED_RPM    3000.0f  /* 过速保护 */
```

PID 参数也能在 VOFA+ 里发 `KP 2.5` 之类在线改，改完 OLED 第 4 行立刻显示。

**算转速的公式**（`encoder.c` 里的 `Encoder_ReadRPM()`）：

```
转速(RPM) = 这段时间的计数增量 / 每转计数 / 时间(min)
10ms 窗口: delta / 1600 / (10/60000) = delta * 3.75
```

---

## 5. 程序怎么跑的

```
SysTick 每 1ms 中断
   └─ 数到 10ms → Control_Step()        ← 控制环放中断里，周期才准
        ├─ Encoder_ReadRPM()   读编码器算转速
        ├─ PID_Calc()          算输出
        └─ Motor_Set()         给方向 + PWM

main 里的 while(1)
   ├─ 每 10ms  Comm_SendFrame()   发一帧 JustFloat 给 VOFA+（20 字节）
   ├─ 轮询      Comm_GetCommand()  收到命令就 HandleCommand()
   └─ 每 20ms  Display_Task()     刷 OLED 的一行（8 行轮流，一屏 160ms）
```

PID 没什么花样，就是最普通的位置式：

```c
out = Kp * 误差 + 积分累加 - Kd * 加速度
```

只有两个小处理：

- **微分只对实际转速求导**（不是对误差），这样改目标值时输出不会突然冲击一下；
- **输出打满时不再积分**，防止积分越攒越多导致大超调。

---

## 6. 调参

1. 先看能不能转：发 `DUTY 800`，电机应该以约 1/5 满速转。不转就是接线/电源问题，跟 PID 无关。
2. 只加 P：发 `KI 0`、`KD 0`、`500`，`KP` 从 0.5 一点点加，加到响应快但还没开始振荡。
3. 加 I 消静差：`KI` 从 1 往上加，直到转速能停在目标值上。
4. 需要的话再加 D：`KD` 很小就行（0.01 起）。

默认 `2 / 10 / 0.02` 是按一台"满占空比约 1200 转"的电机仿出来的保守值，上电 1000 转阶跃大约 1.7 s 稳定、基本不过冲。换电机要重新调。

---

## 7. 编译烧录

```bash
cmake --preset Debug
cmake --build --preset Debug
```

产物：`build/Debug/motor.hex`（或 `.bin`），用 STM32CubeProgrammer 下载。

> 工程里加了 `-u _printf_float`（在 CMakeLists.txt），这样 `snprintf("%.2f")` 才能用，
> 代价是多占约 9 KB Flash。不需要在 OLED 上显示小数的话可以删掉那行。

---

## 8. 常见问题

| 现象 | 原因 |
|---|---|
| **电机不转** | 先发 `DUTY 800`。开环也不转 → TB6612 的 VM 电源 / AIN1/AIN2 / STBY / PWM 接线问题；开环能转 → 看 ch1 转速对不对 |
| **一上电就冲高速然后停** | 编码器 A/B 相接反了，反馈变成正反馈。把参数区里的 `ENCODER_DIR` 改成 `-1.0f`，或直接对调 A/B 两根线 |
| 转速一直是 0 | 编码器没信号：查 A/B 相是否接 PE9/PE11、编码器供电 |
| 转速是实际的 30 倍 | 减速比没填，改 `GEAR_RATIO` |
| VOFA+ 没波形 | 通道数是不是 4；数据引擎是不是 JustFloat；TX/RX 是不是交叉接的 |
| 串口全是乱码 | 板子晶振和 `HSE_VALUE` 对不上。本工程按 **8 MHz** 配的（`stm32f4xx_hal_conf.h`），25 MHz 的板子要改成 `25000000U` 并把 `main.c` 里 `PLLM` 改成 25 |
| OLED 不亮 | I2C 地址 0x3C/0x3D 会自动试；再检查 PB6/PB7 接反没有 |
| 电机啸叫 | 正常不会，有的话检查晶振配置（PWM 应该是 20 kHz） |
