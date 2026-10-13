# STM32F407 直流有刷电机 PID 速度控制

基于 `motor` 工程（STM32F407VGT6 + CMake + arm-none-eabi-gcc）实现的**直流有刷电机闭环速度控制**，
带 VOFA+ 虚拟示波器（**JustFloat 二进制协议**，可一键切回 FireWater）和 0.98 寸 I2C OLED 本地显示。

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

> **⚠️ 晶振必须与工程配置一致（本项目踩过这个坑）**
>
> CubeMX 给 STM32F407 的 HSE 默认值是 **25 MHz**，而这块板子上是 **8 MHz** 晶振。
> 如果 `HSE_VALUE` / `PLLM` 填错，时钟树会整体缩放，但**程序照样能跑**，
> 只是所有时间基准都不对：
>
> | | 按 25 MHz 配置 + 8 MHz 实物 | 已修正为 8 MHz |
> |---|---|---|
> | SYSCLK | 53.76 MHz | 168 MHz |
> | 串口实际波特率 | 36864（上位机按 115200 收就是乱码） | 115226 ≈ 115200 |
> | PWM | 6.4 kHz（有可听啸叫） | 20 kHz |
> | 控制周期 | 31.25 ms，但 PID 里 dt 写死 10 ms | 10 ms |
> | I2C | 128 kHz | 400 kHz |
>
> 本工程已按 8 MHz 配好，换板子时务必先确认晶振丝印。详见第 10 节排查表。

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
│   │   ├── vofa.h      JustFloat/FireWater 帧构造 + 命令接收（协议开关在这里）
│   │   ├── oled.h      OLED 上层接口
│   │   ├── oled_font.h 6x8 ASCII 点阵字库
│   │   └── fmt.h       不依赖 printf 浮点的数值格式化
│   └── Src/            与上面一一对应的 .c
├── tests/
│   ├── test_pid_fmt.c  PID / 格式化 / 编码器换算 / 两种帧格式
│   ├── test_vofa.c     中断收发 + 环形缓冲时序（用 stubs 里的寄存器桩）
│   └── stubs/          hal_stub.h / main.h / usart.h，主机端替代 STM32 HAL
├── motor.ioc           CubeMX 配置（已更新引脚/编码器模式/PWM 周期/I2C 速率）
└── README.md
```

### 各模块职责

| 模块 | 职责 |
|---|---|
| `encoder.c` | M 法测速：每 10 ms 读 TIM1 计数增量，换算成 RPM 并做一阶低通 |
| `pid.c` | 位置式 PID：微分先行 + 微分低通 + 积分分离 + 条件积分抗饱和 + 输出限幅 |
| `motor.c` | TB6612 方向/PWM 输出，`Motor_SetDuty(-4199..+4199)` |
| `vofa.c` | JustFloat/FireWater 帧发送 + 中断环形缓冲收发 + 命令行解析 |
| `oled.c` | SSD1306 显存 + 脏页刷新（只通过 I2C 推送变化的页） |
| `app.c` | 10 ms 控制环调度、命令处理、100 Hz 上报、100 ms 显示 |

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
| `APP_TELEMETRY_PERIOD_MS` | `app.h` | `10` | 上报周期（100 Hz，JustFloat 下发一次仅 20 字节；切回 FireWater 建议改 `20`） |
| `APP_DEFAULT_TARGET_RPM` | `app.h` | `500.0f` | **上电默认目标转速** |
| `APP_AUTO_START` | `app.h` | `1` | `1` = 上电就跑默认转速；`0` = 需要发 `START` 才动 |
| `APP_OVERSPEED_RPM` | `app.h` | `3000.0f` | 绝对过速保护阈值 |
| `APP_STALL_RPM` | `app.h` | `50.0f` | 反馈丢失判定：输出接近满量程但转速一直低于它 |
| `VOFA_USE_JUSTFLOAT` | `vofa.h` | `1` | `1` = JustFloat 二进制，`0` = FireWater 文本 |
| `APP_DEFAULT_KP/KI/KD` | `app.h` | `2.0 / 10.0 / 0.02` | 上电默认整定参数 |
| `OLED_HEIGHT` | `oled.h` | `64` | 0.91 寸模块改成 `32` |
| `OLED_CTRL_SH1106` | `oled.h` | `0` | 1.3 寸 SH1106 改成 `1` |
| `huart1.Init.BaudRate` | `usart.c` | `115200` | 与 VOFA+ 保持一致 |
| `HSE_VALUE` | `stm32f4xx_hal_conf.h` | `8000000U` | **板上晶振频率**，填错会让整棵时钟树按错误比例缩放 |
| `PLL.PLLM` | `main.c` | `8` | HSE / PLLM 必须是 1~2 MHz；8 MHz 晶振用 8，25 MHz 晶振用 25 |

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

## 5. VOFA+ 上位机设置（JustFloat）

1. 打开 VOFA+，`数据引擎` 选择 **JustFloat**。
2. **关键一步**：把波形图的 **通道数设为 4**。
   JustFloat 帧里不携带通道个数，上位机完全靠这个设置来切分数据 ——
   填错（比如默认的 1）就会看到乱七八糟的曲线，这是该协议最常见的坑。
3. 连接：端口选 USB-TTL 对应的 COM 口，波特率 **115200**，数据位 8，停止位 1，无校验。
4. 打开串口后应出现 4 条曲线：

   | 通道 | 含义 |
   |---|---|
   | ch0 | 期望值 target（RPM） |
   | ch1 | 实际值 actual（RPM） |
   | ch2 | 输出占空比 duty（0~4199） |
   | ch3 | 误差 error（RPM） |

   建议只勾选 ch0/ch1 观察跟随效果；把 ch2 打开可以看到 PID 输出的变化。

5. **发送期望转速**：在 VOFA+ 下方的 `数据收发 / 发送` 输入框里直接输入一个数字，
   例如 `1500`，点击发送，电机目标转速立刻变成 1500 RPM。
   也可以输入 `TARGET 1500`、`STOP`、`KP 3` 等完整命令，见下一节。
   发送框如果支持"追加换行"，请勾选；不勾选也可以，固件在 40 ms 空闲后会自动按整行处理。

6. 数据帧格式（JustFloat，纯二进制）：

   ```c
   struct Frame {
       float         ch_data[4];                  // 小端, 每通道 4 字节
       unsigned char tail[4] = {0, 0, 0x80, 0x7f}; // 帧尾 = 小端 +Inf
   };                                             // 一帧固定 20 字节
   ```

   接收区建议勾选**十六进制显示**，否则只能打印乱码。
   官方协议说明：<https://vofa.plus/docs/learning/dataengines/justfloat/>

> **为什么文本回显默认是关的？**
> JustFloat 是纯二进制流，上位机只能靠帧尾切帧。任何 ASCII 文本（HELP/SAVE 的输出）
> 都会滞留在上位机缓冲区里，直到下一个帧尾到达时被当成一大帧垃圾数据，
> 官方文档也提示这会让缓冲区爆满。所以固件在 JustFloat 模式下默认静音文本，
> 想看 `HELP` / `SAVE` 的输出，先在发送框发一次 `TEXT 1`；调完再 `TEXT 0` 关掉。

### 想切回 FireWater？

把 `vofa.h` 里的 `VOFA_USE_JUSTFLOAT` 改成 `0` 重新编译即可，
帧格式变为 `"ch0,ch1,ch2,ch3\n"`（例如 `1500.00,1489.34,3821.00,10.66`），
文本回显也会自动恢复默认打开。两种格式的构造函数都在 `vofa.h` 里，都有单元测试覆盖。

---

## 6. 串口命令列表

| 命令 | 作用 |
|---|---|
| `1200` | 直接发数字，等价于 `TARGET 1200`（负数即反转）；**会自动使能驱动，不用再发 START** |
| `TARGET <rpm>` | 设置目标转速（同样自动使能） |
| `KP <v>` / `KI <v>` / `KD <v>` | 在线修改 PID 增益 |
| `ILIMIT <v>` | 积分项限幅 |
| `LIMIT <v>` | 输出限幅（1~4199） |
| `SEP <v>` | 积分分离阈值（RPM），0 关闭 |
| `DFILT <a>` | 微分低通系数（0~1） |
| `START` | 使能闭环并清除保护锁定 |
| `STOP` | 停机、目标清零、清除保护锁定 |
| `RESET` | 复位 PID 积分、微分与编码器计数，并清除保护锁定 |
| `OPEN` / `CLOSE` | 开环 / 闭环切换 |
| `DUTY <v>` | 开环直接给定占空比（-4199~4199），也会自动使能 |
| `SAFE <ms>` | 通信超时保护：超过 ms 没收到命令就停机（0 = 关闭，默认关闭） |
| `TEXT 0` / `TEXT 1` | 关闭 / 打开 HELP、SAVE 的文本回显（JustFloat 下默认关闭） |
| `SAVE` | 打印当前全部参数到 VOFA+ 文本区 |
| `HELP` | 打印命令帮助 |

上电后固件会先打印一段帮助信息，可以用 VOFA+ 的"字节/文本接收区"确认链路是否通。

---

## 7. 上电调试与 PID 整定

### 7.1 第一次上电：先确认"电机到底能不能转"

上电默认行为是 **`APP_AUTO_START = 1` + 目标 `500 RPM`**：一上电就闭环跑 500 转，
不用先发 `START`（发数字或 `TARGET` 也会自动使能）。

如果电机还是不动，按下面的顺序分层排查 —— 关键是**先把反馈环摘掉，只验证功率通路**：

| 步骤 | 发送 | 预期 | 不符合说明什么 |
|---|---|---|---|
| ① 看波形 | — | ch0 应在 500 上下、ch1 跟着爬 | 通道数没设成 4 / 波特率不对 |
| ② 开环转 | `DUTY 800` | 电机以约 1/5 满速转，**不看编码器** | 不动 → TB6612 接线/VM 供电/STBY/PWM 问题 |
| ③ 看反馈 | `OPEN` + `DUTY 800` | ch1 显示真实转速，符号与转向一致 | ch1 一直是 0 → 编码器没接/没信号；符号相反 → 改 `ENCODER_DIR_INVERT` |
| ④ 闭环 | `CLOSE` → `500` | ch1 稳定在 500 | 能开环不能闭环 → 多半是 ③ 的问题 |

`DUTY` 命令走开环，**不经过编码器也不经过 PID**，所以它不动就一定不是算法问题。

### 7.2 两个保护（自动生效）

第一次上电最容易踩的两个坑，固件会自己识别出来并停机，同时把原因写到 OLED 最后一行
（JustFloat 下想看文字要先 `TEXT 1`）：

| 保护 | 触发条件 | OLED / 文本 | 含义 |
|---|---|---|---|
| **FAULT NO FEEDBACK** | 输出已接近满量程，转速却一直低于 `APP_STALL_RPM`(50)，持续 500 ms | `FAULT NO FEEDBACK` | 编码器没信号 / 电机没供电 / 堵转 |
| **FAULT OVERSPEED** | `\|实际\| > 3000 RPM`，或 `\|实际\| > \|目标\|×2 + 200`，持续 500 ms | `FAULT OVERSPEED` | 编码器极性接反导致**正反馈**，转速一路飙升 |

过速判定用"实际是否远超目标"而不是只看绝对值，所以**正常反向不会误触发**
（反向时实际只是短暂与目标反号、幅值在下降）。

锁定后发 `START` / `RESET` / 任意新目标值即可解除。想关掉或调阈值就改 `app.h` 里
`APP_AUTO_START`、`APP_OVERSPEED_RPM`、`APP_STALL_RPM`。

### 7.3 PID 整定

默认参数（Kp=2.0 / Ki=10.0 / Kd=0.02）是按一台"满占空比约 1200 RPM、时间常数 80 ms、
死区约 3%"的典型减速电机仿真出来的保守起步值：1000 RPM 阶跃约 1.7 s 稳定、基本无超调。
换电机后按下面的顺序调：

1. **先开环找量级**：`OPEN` → `DUTY 800`，看实际转速，估算"多少占空比对应多少 RPM"。
   例如 800 对应 240 RPM，说明 1000 RPM 需要约 3400 的占空比。
2. **只加 P**：`CLOSE` → `KI 0` → `KD 0` → `1000`，逐步加大 `KP`
   （0.5 → 1 → 2 → 4 …）。目标是响应快、接近目标但允许有静差，不要出现持续振荡。
3. **加 I 消除静差**：逐步加大 `KI`（1 → 5 → 10 → 20 …）。
   若出现"过冲后反复摆动"，说明 I 太大，回退一档，或启用积分分离 `SEP 60`。
4. **需要时加 D**：编码器有量化噪声，`KD` 从 `0.01` 起试，配合 `DFILT 0.2` 抑制噪声放大。
   转速环一般可以不加 D。
5. **限幅**：`LIMIT` 默认就是满量程 4199；若电机/电源较小，可降低到
   `LIMIT 2000` 之类的值限制最大电流。
6. 参数满意后用 `SAVE` 打印当前值，再把它填回 `app.h` 的 `APP_DEFAULT_*` 就是上电默认值。

> 安全提示：整定时把电机固定在支架上；目标转速不要超过 `APP_OVERSPEED_RPM`，
> 否则会被过速保护停机（要跑更高转速就一起调大阈值）。

单步整定的推荐流程（VOFA+ 里最直观）：
`STOP` → `KP 2` → `KI 10` → `KD 0.02` → `1000`，观察 ch0/ch1 两条曲线的跟随过程，
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
   ├─ 每 10 ms  置上报标志 (100 Hz)
   └─ 每 100 ms 置显示标志

main while(1) ──► App_Loop()
   ├─ VOFA_Poll()         解析串口命令行 → 修改目标值/参数
   ├─ VOFA_SendSamples()  压入发送环形缓冲 (JustFloat 20 字节/帧)
   │        └─ USART1 TXE 中断逐字节发出, 发完自动关中断
   └─ OLED 刷新           只通过 I2C 推送脏页
```

设计要点：

- **控制环放在 SysTick 中断里**，采样周期严格 10 ms，不会被 OLED/I2C/串口阻塞打乱。
- **收发全部走中断 + 环形缓冲**：发送只是往 512 字节环形缓冲里 `memcpy`，
  由 TXE 中断逐字节发出，主循环完全不等串口。这样即使 I2C 刷一屏要占用主循环
  20~30 ms，100 Hz 的数据帧依然按点节拍发出，不会丢帧或抖动。
  缓冲放不下时**整帧丢弃**而不是发一半，避免上位机永远等不到帧尾。
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

两套测试都在 PC 上跑，不需要开发板。

```bash
cd motor

# 1) PID / 格式化 / 编码器换算 / 两种帧格式
gcc -std=c11 -Wall -Wextra -I Core/Inc \
    tests/test_pid_fmt.c Core/Src/pid.c Core/Src/fmt.c -lm -o build/test_pid_fmt
./build/test_pid_fmt

# 2) 中断收发 + 环形缓冲时序 (用 tests/stubs 里的寄存器桩代替 HAL)
gcc -std=c11 -Wall -Wextra -I tests/stubs -I Core/Inc \
    tests/test_vofa.c Core/Src/vofa.c Core/Src/fmt.c -lm -o build/test_vofa
./build/test_vofa
```

`test_pid_fmt` 覆盖：

- `FMT_Int` / `FMT_Float` 的边界（`INT32_MIN`、四舍五入、负零、进位到整数）
- 400 线 ×4 倍频、10 ms 窗口的 RPM 换算（1600 计数/圈，3.75 RPM/计数）
- PID 比例项、输出限幅、积分累加/限幅/条件积分抗饱和、误差反向时积分释放
- 微分先行（目标值阶跃无微分冲击）、微分低通、积分分离
- 闭环仿真：用出厂默认参数跑一阶电机模型，检查稳态误差、超调、调节时间、
  正反转对称性与低速精度
- **JustFloat 帧**：逐字节核对小端位型（`1.0f → 00 00 80 3F`）、帧尾 `00 00 80 7F`、
  1/4/8 通道的帧长、帧尾确实是 `+Inf`、按上位机方式扫描帧尾切帧后数值完全还原
- **FireWater 帧**：CSV 文本、结尾换行、缓冲不足时整体失败（不发半帧）

`test_vofa` 覆盖：

- `VOFA_Init()` 后的中断使能状态
- 压栈后 TXE 中断被挂起、中断没来之前一个字节都不发
- 排空后线路上的字节数与内容（含帧尾、小端位型、可被上位机还原）
- 连续压 30 帧只发出 25 个**完整**帧（512B 缓冲可用 511B），且每帧都有帧尾
- 突发之后环形缓冲能立刻恢复正常发送
- 命令组帧：`\r\n` 与 `\n` 分隔、不带换行时 40 ms 空闲自动成行、`VOFA_GetLastRxTick()` 刷新
- 文本回显开关：JustFloat 下默认静音、开了能发、关了立刻静音、数据帧不受影响
- ORE 等错误标志被正确消费，不会反复进中断

本机运行结果：**test_pid_fmt 70 passed / test_vofa 28 passed，0 failed**。

---

## 10. 常见问题

| 现象 | 排查 |
|---|---|
| **电机完全不动** | 先发 `DUTY 800`（开环，绕过编码器和 PID）。开环也不转 → TB6612 的 VM/AIN1/AIN2/STBY/PWM 接线或供电问题；开环能转 → 是编码器反馈的问题（ch1 是否为 0、符号是否相反）。详见第 7.1 节 |
| 上电转一下就停了，OLED 显示 `FAULT NO FEEDBACK` | 编码器没有信号：检查 A/B 相是否接到 PE9/PE11、编码器供电、`ENCODER_GEAR_RATIO`。这是反馈丢失保护，不是故障 |
| 一上电就冲高然后停机，OLED 显示 `FAULT OVERSPEED` | 编码器极性接反导致正反馈：把 `ENCODER_DIR_INVERT` 改成 `1`（或对调 A/B 相） |
| 发了数字但 ch0 不变 | 命令没被解析：先发 `TEXT 1`，若收不到 `TEXT REPLY ON`，是串口链路（波特率/接线）问题，见下一条 |
| **串口收到的全是 `00` / `1C` / `E0` / `FC` 这类字节，怎么都看不到 `00 00 80 7F`** | **晶振频率与 `HSE_VALUE` 不符**（本项目实测：CubeMX 默认 25 MHz、板上是 8 MHz）。验证方法：把串口工具波特率改成 **36864**（= 115200 ÷ 3.125 = 115200 ÷ 25×8），数据立刻正常就说明是这个原因；根治办法见第 1 节，改 `HSE_VALUE` 与 `PLLM`。 |
| VOFA+ 一片空白 | 数据引擎是否选对（默认 JustFloat）；**JustFloat 的通道数必须设成 4**；波特率是否 115200；TX/RX 是否交叉接线 |
| 曲线乱七八糟、数值离谱 | JustFloat 通道数填错了（帧里不带通道数，全靠上位机设置） |
| 接收区全是乱码 | JustFloat 是二进制，接收区要勾选**十六进制显示** |
| 想用 FireWater | `vofa.h` 里 `VOFA_USE_JUSTFLOAT` 改成 `0` 重新编译 |
| 发 `HELP` / `SAVE` 没反应 | JustFloat 下文本默认静音，先发 `TEXT 1` |
| 文本一出现波形就跳一下 | 文本字节混进了二进制流，发 `TEXT 0` 关掉即可 |
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
`Core/Src/{app,pid,encoder,motor,vofa,oled,fmt}.c`、
`tests/test_pid_fmt.c`、`tests/test_vofa.c`、`tests/stubs/`、本 README。

修改：
- `Core/Src/main.c`：调用 `App_Init()`，主循环改为 `App_Loop()`
- `Core/Src/stm32f4xx_it.c`：SysTick 里调用 `App_Tick()`；`USART1_IRQHandler` 改为调用 `VOFA_IrqHandler()`（收发一体，不再走 HAL 状态机）
- `Core/Src/tim.c`：TIM1 改 `TIM_ENCODERMODE_TI12`（4 倍频）；TIM2 `Period=4199`（20 kHz）
- `Core/Src/i2c.c`：I2C1 提到 400 kHz
- `Core/Src/gpio.c`：PA2/PA3/PA4 配置为推挽输出（TB6612 AIN1/AIN2/STBY）
- `CMakeLists.txt`：加入 7 个用户源文件，并新增 `.hex`/`.bin` 生成步骤
- `motor.ioc`：同步上述外设与引脚配置

### 第二轮：数据格式改为 JustFloat

- `vofa.h` / `vofa.c`：新增 `VOFA_USE_JUSTFLOAT` 协议开关（默认 JustFloat）+ `VOFA_BuildJustFloat()`
  / `VOFA_BuildFireWater()` 两个纯函数帧构造器；发送改为 **TXE 中断 + 512B 环形缓冲**，
  彻底去掉阻塞式 `HAL_UART_Transmit`
- `vofa.c`：新增文本回显开关（JustFloat 下默认静音），并提供 `VOFA_SetTextReply()`
- `app.c`：新增 `TEXT 0|1` 命令；`SAVE` 增加协议/通道数信息；开机信息按协议区分
- `app.h`：上报周期 20 ms → 10 ms（100 Hz）
- `tests/test_pid_fmt.c`：新增 JustFloat / FireWater 帧构造测试
- `tests/test_vofa.c` + `tests/stubs/`：新增中断收发链路的主机端测试

### 第三轮：修正晶振配置（上位机只收到乱码的根因）

现象：VOFA+ 选 JustFloat 后收不到波形，串口抓包只有 `00 / 1C / E0 / FC`，从不出现帧尾 `00 00 80 7F`。

定位：按不同波特率比模拟 UART 采样，只有 **r = 3.125 = 25 ÷ 8** 能复现出这套字节
（`00` 的连续段 + `FC 1C` 取代 `80 7F`）→ 说明 MCU 实际波特率是 36864 而非 115200。
根因是 CubeMX 默认的 `HSE_VALUE = 25 MHz` 与板上 8 MHz 晶振不符，整棵时钟树被缩放到 0.32 倍。

- `Core/Inc/stm32f4xx_hal_conf.h`：`HSE_VALUE` 25000000U → **8000000U**
- `Core/Src/main.c`：`PLL.PLLM` 25 → **8**（VCO 输入 1 MHz，SYSCLK 回到 168 MHz）
- `Core/Src/system_stm32f4xx.c`：默认 `HSE_VALUE` 同步改为 8000000（避免误导）
- `motor.ioc`：`RCC.HSE_VALUE=8000000`、`RCC.PLLM=8`

PLLN/PLLP/PLLQ 及所有分频都没动，因此 APB1/APB2/定时器时钟与之前完全一致，
PWM 回到 20 kHz、控制周期回到 10 ms、串口回到 115226 baud（误差 0.02%）。

### 第四轮：上电即运行 + 保护

问题：发数字/`TARGET` 只设了目标值，没使能闭环，电机不动。

- `app.c`：`TARGET` 与纯数字命令**自动使能驱动**（`App_EnableDrive()`），不再需要先发 `START`；
  `DUTY` 开环测试同样自动使能
- `app.h`：新增 `APP_DEFAULT_TARGET_RPM`（500 RPM）与 `APP_AUTO_START`，
  上电即以默认转速闭环运行，第一次上电就能看到转动和曲线
- `app.c`：新增两级保护 + 锁定
  - **FAULT NO FEEDBACK**：输出接近满量程但转速一直 < 50 RPM → 编码器没信号/电机没供电/堵转
  - **FAULT OVERSPEED**：`|实际| > 3000` 或 `|实际| > |目标|×2 + 200` → 编码器极性接反导致的正反馈
    （用相对判据，正常反向不会误触发）
- OLED 状态位与 `SAVE` 输出增加 `FLT` / `FAULT` 指示；保护原因同时写入 OLED 末行与串口文本
- README 第 7 节重写为"上电调试 → 保护说明 → PID 整定"三小节，附分层排查表
