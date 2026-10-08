# STM32F407 直流有刷电机 PID 速度控制

基于 `motor` 工程（STM32F407VGT6 + CMake + arm-none-eabi-gcc）实现的**直流有刷电机闭环速度控制**，
带 VOFA+ 虚拟示波器（FireWater 协议）和 0.98 寸 I2C OLED 本地显示。

```
        +-----------+    PWM/方向    +------------+   OUT1/OUT2   +---------+
        |           |-------------->|            |-------------->|         |
        | STM32F407 |               | TB6612FNG  |               | 直流电机 |
        |           |<--------------|            |<--------------|         |
        +-----------+   A/B 相      +------------+    编码器 A/B +---------+
          |      |
          |      +---- I2C1 (PB6/PB7) ----> 0.98" OLED
          +----------- USART1 (PA9/PA10) <-> VOFA+ 上位机
```

---

## 1. 硬件接线

| STM32 引脚 | 外设配置 | 连接到 | 说明 |
|---|---|---|---|
| PA0 | TIM2_CH1 (PWM 20 kHz) | TB6612 **PWMA** | 占空比 0~4199 |
| PA2 | GPIO 输出 | TB6612 **AIN1** | 方向位 |
| PA3 | GPIO 输出 | TB6612 **AIN2** | 方向位 |
| PA4 | GPIO 输出 | TB6612 **STBY** | 高电平使能驱动器 |
| PE9 | TIM1_CH1 (编码器) | 编码器 **A 相** | 正交解码 |
| PE11 | TIM1_CH2 (编码器) | 编码器 **B 相** | 正交解码 |
| PB6 | I2C1_SCL (400 kHz) | OLED **SCL** | 0.98" 128×64 |
| PB7 | I2C1_SDA (400 kHz) | OLED **SDA** | 地址 0x3C / 0x3D 自动识别 |
| PA9 | USART1_TX (115200) | USB-TTL **RX** | 发给 VOFA+ |
| PA10 | USART1_RX | USB-TTL **TX** | 接收 VOFA+ 命令 |
| PA13/PA14 | SWDIO/SWCLK | ST-Link | 下载调试 |

电源部分（**必须共地**）：

```
12V 电源 ──┬── TB6612 VM
           ├── TB6612 GND ──┬── STM32 GND
           └── (可选) 5V/3.3V LDO ──┬── STM32 3V3
                                   └── OLED VCC
TB6612 VCC  ── 3.3V 或 5V (逻辑电源)
TB6612 AO1/AO2 ── 电机 M+ / M-
TB6612 GND   ── 与 STM32 共地
```

> TB6612 的 STBY 悬空时驱动器不工作，本工程已在 `Motor_Init()` 中拉高。

---

## 2. 工程结构

```
motor/
├── Core/
│   ├── Inc/
│   │   ├── app.h       应用层接口、控制周期、默认整定参数
│   │   ├── pid.h       速度环 PID
│   │   ├── encoder.h   400 线 AB 相编码器参数与测速
│   │   ├── motor.h     TB6612 引脚与接口
│   │   ├── vofa.h      FireWater 上报 + 命令接收
│   │   ├── oled.h      OLED 上层接口
│   │   ├── oled_font.h 6x8 ASCII 点阵字库
│   │   └── fmt.h       不依赖 printf 浮点的数值格式化
│   └── Src/            与上面一一对应的 .c
├── tests/
│   └── test_pid_fmt.c  主机端单元测试（PC 上 gcc 直接跑）
├── motor.ioc           CubeMX 配置（已更新引脚/编码器模式/PWM 周期/I2C 速率）
└── README.md
```

### 各模块职责

| 模块 | 职责 |
|---|---|
| `encoder.c` | M 法测速：每 10 ms 读 TIM1 计数增量，换算成 RPM 并做一阶低通 |
| `pid.c` | 位置式 PID：微分先行 + 微分低通 + 积分分离 + 条件积分抗饱和 + 输出限幅 |
| `motor.c` | TB6612 方向/PWM 输出，`Motor_SetDuty(-4199..+4199)` |
| `vofa.c` | FireWater 文本帧发送 + USART1 接收中断 + 环形缓冲 + 命令行解析 |
| `oled.c` | SSD1306 显存 + 脏页刷新（只通过 I2C 推送变化的页） |
| `app.c` | 10 ms 控制环调度、命令处理、50 Hz 上报、100 ms 显示 |

---

## 3. 关键参数（改这里就够了）

| 参数 | 文件 | 默认值 | 说明 |
|---|---|---|---|
| `ENCODER_LINES` | `encoder.h` | `400.0f` | 编码器线数（铭牌值） |
| `ENCODER_MULTIPLE` | `encoder.h` | `4.0f` | 倍频，对应 TIM1 的 TI12 模式 |
| `ENCODER_GEAR_RATIO` | `encoder.h` | `1.0f` | **减速比**：编码器在电机轴上就填减速比（如 `30.0f`），在输出轴上填 `1.0f` |
| `ENCODER_FILTER_ALPHA` | `encoder.h` | `0.40f` | 测速低通系数，越小越平滑 |
| `ENCODER_DIR_INVERT` | `encoder.h` | `0` | 电机正转时读数为负就改成 `1` |
| `MOTOR_PWM_MAX` | `motor.h` | `4199` | 对应 20 kHz PWM 满占空比 |
| `APP_CONTROL_PERIOD_MS` | `app.h` | `10` | 控制周期，必须与 `ENCODER_SAMPLE_MS` 一致 |
| `APP_TELEMETRY_PERIOD_MS` | `app.h` | `20` | 上报周期（50 Hz） |
| `APP_DEFAULT_KP/KI/KD` | `app.h` | `2.0 / 10.0 / 0.02` | 上电默认整定参数 |
| `OLED_HEIGHT` | `oled.h` | `64` | 0.91 寸模块改成 `32` |
| `OLED_CTRL_SH1106` | `oled.h` | `0` | 1.3 寸 SH1106 改成 `1` |
| `huart1.Init.BaudRate` | `usart.c` | `115200` | 与 VOFA+ 保持一致 |

**换算关系**：`每转计数 = 线数 × 倍频 × 减速比 = 400 × 4 × 1 = 1600`，
10 ms 窗口下 1 个计数 = `3.75 RPM`，即最低测速分辨率是 3.75 RPM。

---

## 4. 编译与烧录

```bash
cd motor
cmake --preset Debug
cmake --build --preset Debug
```

产物：`build/Debug/motor.elf` / `motor.hex` / `motor.bin`。
本机验证使用的工具链是 `D:\toolchains\xpack-arm-none-eabi-gcc-15.2.1-1.1`（需加入 PATH）。

烧录（任选其一）：
- STM32CubeProgrammer → 打开 `motor.hex` → Download
- ST-Link Utility → `motor.hex`
- 直接把工程导入 STM32CubeIDE 编译下载

---

## 5. VOFA+ 上位机设置（FireWater）

1. 打开 VOFA+，`数据引擎` 选择 **FireWater**。
2. 连接：端口选 USB-TTL 对应的 COM 口，波特率 **115200**，数据位 8，停止位 1，无校验。
3. 打开串口后，波形图会自动出现 **4 条曲线**：

   | 通道 | 含义 |
   |---|---|
   | ch0 | 期望值 target（RPM） |
   | ch1 | 实际值 actual（RPM） |
   | ch2 | 输出占空比 duty（0~4199） |
   | ch3 | 误差 error（RPM） |

   建议只勾选 ch0/ch1 观察跟随效果；把 ch2 打开可以看到 PID 输出的变化。

4. **发送期望转速**：在 VOFA+ 下方的 `数据收发 / 发送` 输入框里直接输入一个数字，
   例如 `1500`，点击发送，电机目标转速立刻变成 1500 RPM。
   也可以输入 `TARGET 1500`、`STOP`、`KP 3` 等完整命令，见下一节。
   发送框如果支持"追加换行"，请勾选；不勾选也可以，固件在 40 ms 空闲后会自动按整行处理。

5. 数据帧格式（FireWater）：

   ```
   "ch0,ch1,ch2,ch3\n"          例如:  1500.00,1489.34,3821.00,10.66
   ```

   FireWater **必须以 `\n` 结尾**，否则 VOFA+ 缓冲区只进不出、界面不刷新。
   固件在 `VOFA_SendSamples()` 中已经自动补上。

---

## 6. 串口命令列表

| 命令 | 作用 |
|---|---|
| `1200` | 直接发数字，等价于 `TARGET 1200`（负数即反转） |
| `TARGET <rpm>` | 设置目标转速 |
| `KP <v>` / `KI <v>` / `KD <v>` | 在线修改 PID 增益 |
| `ILIMIT <v>` | 积分项限幅 |
| `LIMIT <v>` | 输出限幅（1~4199） |
| `SEP <v>` | 积分分离阈值（RPM），0 关闭 |
| `DFILT <a>` | 微分低通系数（0~1） |
| `START` / `STOP` | 使能闭环 / 停机并清零目标 |
| `RESET` | 复位 PID 积分、微分与编码器计数 |
| `OPEN` / `CLOSE` | 开环 / 闭环切换 |
| `DUTY <v>` | 开环直接给定占空比（-4199~4199） |
| `SAFE <ms>` | 通信超时保护：超过 ms 没收到命令就停机（0 = 关闭，默认关闭） |
| `SAVE` | 打印当前全部参数到 VOFA+ 文本区 |
| `HELP` | 打印命令帮助 |

上电后固件会先打印一段帮助信息，可以用 VOFA+ 的"字节/文本接收区"确认链路是否通。

---

## 7. PID 整定步骤

默认参数（Kp=2.0 / Ki=10.0 / Kd=0.02）是按一台"满占空比约 1200 RPM、时间常数 80 ms、
死区约 3%"的典型减速电机仿真出来的保守起步值：1000 RPM 阶跃约 1.7 s 稳定、基本无超调。
换电机后按下面的顺序调：

1. **先开环找量级**：`OPEN` → `DUTY 800`，看实际转速，估算"多少占空比对应多少 RPM"。
   例如 800 对应 240 RPM，说明 1000 RPM 需要约 3400 的占空比。
2. **只加 P**：`CLOSE` → `KI 0` → `KD 0` → `TARGET 1000`，逐步加大 `KP`
   （0.5 → 1 → 2 → 4 …）。目标是响应快、接近目标但允许有静差，不要出现持续振荡。
3. **加 I 消除静差**：逐步加大 `KI`（1 → 5 → 10 → 20 …）。
   若出现"过冲后反复摆动"，说明 I 太大，回退一档，或启用积分分离 `SEP 60`。
4. **需要时加 D**：编码器有量化噪声，`KD` 从 `0.01` 起试，配合 `DFILT 0.2` 抑制噪声放大。
   转速环一般可以不加 D。
5. **限幅**：`LIMIT` 默认就是满量程 4199；若电机/电源较小，可降低到
   `LIMIT 2000` 之类的值限制最大电流。
6. 参数满意后用 `SAVE` 打印当前值，再把它填回 `app.h` 的 `APP_DEFAULT_*` 就是上电默认值。

> 安全提示：整定时把电机固定在支架上；切换正反转时留足减速时间；
> 不确定时先 `SAFE 3000` 打开通信超时保护。

单步整定的推荐流程（VOFA+ 里最直观）：
`STOP` → `KP 2` → `KI 10` → `KD 0.02` → `START` → `1000`，观察 ch0/ch1 两条曲线的跟随过程，
再 `-1000` 看反向，再 `300` 看低速段（低速段受 3.75 RPM 量化限制，误差略大是正常的）。

---

## 8. 控制原理与数据流

```
SysTick 1 ms  ──► App_Tick()
   ├─ 每 10 ms  App_ControlStep()          【中断内，采样周期确定】
   │      Encoder_Update()      读 TIM1 CNT 增量 → RPM（一阶低通）
   │      PID_Compute()         e = target - speed
   │                            P = Kp·e
   │                            I += Ki·e·dt   （积分分离 + 限幅 + 条件积分抗饱和）
   │                            D = Kd·d(speed)/dt （微分先行 + 低通）
   │                            out 限幅到 ±4199
   │      Motor_SetDuty(out)    AIN1/AIN2 定向 + TIM2_CH1 占空比
   ├─ 每 20 ms  置上报标志
   └─ 每 100 ms 置显示标志

main while(1) ──► App_Loop()
   ├─ VOFA_Poll()         解析串口命令行 → 修改目标值/参数
   ├─ VOFA_SendSamples()  FireWater 帧:  "target,actual,duty,error\n"
   └─ OLED 刷新           只通过 I2C 推送脏页
```

设计要点：

- **控制环放在 SysTick 中断里**，采样周期严格 10 ms，不会被 OLED/I2C/串口阻塞打乱。
- **I2C 只推脏页**：0.98 寸 OLED 一帧 1 KB，100 kHz 下全屏推送要 100 ms 以上；
  改成"只推送被修改的页"（每页 129 字节）+ I2C 400 kHz 后，每次刷新只需几毫秒。
- **不依赖 printf 浮点**：`fmt.c` 用整数运算实现 `%f` 效果，避免 newlib-nano 下
  需要 `-u _printf_float` 而额外增大固件。
- **OLED 可选**：`OLED_Init()` 探测不到屏幕时只返回 0，不影响电机控制。
  I2C 连续 3 次失败会自动 `HAL_I2C_DeInit()` + 重新初始化，处理总线被拉死的情况。
- **抗饱和**：输出饱和且误差同向时冻结积分，避免退饱和时的巨大超调
  （单元测试里同一场景的原始积分会涨到 10000，加抗饱和后停在 100）。

---

## 9. 主机端单元测试

`pid.c` / `fmt.c` 不依赖 HAL，可以直接在 PC 上验证：

```bash
cd motor
gcc -std=c11 -Wall -Wextra -I Core/Inc \
    tests/test_pid_fmt.c Core/Src/pid.c Core/Src/fmt.c -lm -o build/test_pid_fmt
./build/test_pid_fmt
```

覆盖内容：

- `FMT_Int` / `FMT_Float` 的边界（`INT32_MIN`、四舍五入、负零、进位到整数）
- 400 线 ×4 倍频、10 ms 窗口的 RPM 换算（1600 计数/圈，3.75 RPM/计数）
- PID 比例项、输出限幅
- 积分累加、积分限幅、条件积分抗饱和、误差反向时积分释放
- 微分先行（目标值阶跃无微分冲击）、微分低通
- 积分分离
- 闭环仿真：用出厂默认参数跑一阶电机模型，检查稳态误差、超调、调节时间、
  正反转对称性与低速精度

本机运行结果：**46 passed, 0 failed**。

---

## 10. 常见问题

| 现象 | 排查 |
|---|---|
| VOFA+ 一片空白 | 数据引擎是否为 FireWater；波特率是否 115200；TX/RX 是否交叉接线；帧尾是否有 `\n` |
| 曲线数值符号相反 | 改 `ENCODER_DIR_INVERT` 为 `1`，或对调编码器 A/B 相 |
| 转速显示是实际值的 30 倍（或 1/30） | 减速比没填对，改 `ENCODER_GEAR_RATIO` |
| 转速分辨率太粗、低速抖动 | 正常现象：1600 计数/圈、10 ms 窗口 → 3.75 RPM/计数；可减小 `ENCODER_FILTER_ALPHA` 或延长测速窗口 |
| 电机不转，STBY 有 3.3V | 检查 TB6612 VM 是否有电机电源、AIN1/AIN2 是否接到 PA2/PA3、PWM 是否在 PA0 |
| 电机嗡嗡响但不转 | PWM 频率问题不大（20 kHz），多半是占空比低于死区，用 `DUTY 800` 试开环 |
| 速度上不去、积分饱和 | 加大 `LIMIT`；确认电源电压与满载电流足够 |
| OLED 花屏 | 把 `i2c.c` 的 `ClockSpeed` 从 400000 改回 100000 |
| 屏幕 32 行只显示一半 | 0.91 寸模块把 `OLED_HEIGHT` 改成 `32` |
| 屏幕整体偏移 2 列 | 1.3 寸 SH1106，把 `OLED_CTRL_SH1106` 改成 `1` |
| CubeMX 重新生成后 PWM 频率变了 | 已同步更新 `motor.ioc`（TIM2 Period=4199、TIM1 EncoderMode=TI12、PA2/3/4 输出、I2C 400 kHz），正常重新生成即可 |
| 重新生成后 I2C 变成 100 kHz | `motor.ioc` 里的 `I2C1.ClockSpeed` 在本机没有其他 I2C 工程可以交叉验证，若 CubeMX 没识别，重新生成后在 I2C1 参数页把 Speed Mode 选成 Fast Mode 即可；即使退回 100 kHz 功能也正常，只是 OLED 刷新慢一些 |

---

## 11. 本次改动清单

新增：`Core/Inc/{app,pid,encoder,motor,vofa,oled,oled_font,fmt}.h`、
`Core/Src/{app,pid,encoder,motor,vofa,oled,fmt}.c`、`tests/test_pid_fmt.c`、本 README。

修改：
- `Core/Src/main.c`：调用 `App_Init()`，主循环改为 `App_Loop()`
- `Core/Src/stm32f4xx_it.c`：SysTick 里调用 `App_Tick()`；新增 `USART1_IRQHandler`（直接读 DR 清标志）
- `Core/Src/tim.c`：TIM1 改 `TIM_ENCODERMODE_TI12`（4 倍频）；TIM2 `Period=4199`（20 kHz）
- `Core/Src/i2c.c`：I2C1 提到 400 kHz
- `Core/Src/gpio.c`：PA2/PA3/PA4 配置为推挽输出（TB6612 AIN1/AIN2/STBY）
- `CMakeLists.txt`：加入 7 个用户源文件，并新增 `.hex`/`.bin` 生成步骤
- `motor.ioc`：同步上述外设与引脚配置
