/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          直流有刷电机 PID 速度控制 Demo
  *
  * ============================ 硬件接线 ============================
  *   PA0   TIM2_CH1  PWM   ->  TB6612 PWMA
  *   PA2   GPIO 输出        ->  TB6612 AIN1
  *   PA3   GPIO 输出        ->  TB6612 AIN2
  *   PA4   GPIO 输出        ->  TB6612 STBY (高电平使能)
  *   PE9   TIM1_CH1         ->  编码器 A 相
  *   PE11  TIM1_CH2         ->  编码器 B 相
  *   PB6   I2C1_SCL         ->  OLED SCL
  *   PB7   I2C1_SDA         ->  OLED SDA
  *   PA9   USART1_TX        ->  USB-TTL RX
  *   PA10  USART1_RX        ->  USB-TTL TX
  *
  * ============================ VOFA+ 设置 ============================
  *   数据引擎 = JustFloat    通道数 = 4    波特率 = 115200
  *   通道0 = 期望转速    通道1 = 实际转速
  *   通道2 = 占空比      通道3 = 误差
  *
  * ============================ 串口命令 ============================
  *   在 VOFA+ 发送框里输入, 带不带换行都行:
  *     500          目标转速 500 RPM (负数反转)
  *     KP 2.5       KI 10       KD 0.02
  *     START        STOP
  *     DUTY 800     开环给占空比 (测接线用)
  *
  * ============================ 代码结构 ============================
  *   main.c     调度 + 命令解析 + 显示    motor.c    电机输出
  *   encoder.c  测速                      pid.c      PID 算法
  *   comm.c     串口收发                  oled.c     屏幕
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "comm.h"
#include "oled.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ==================================================================
 *  【参数区】上电默认值
 * ================================================================== */
#define CTRL_MS          10       /* 控制周期 ms (速度环 100Hz)                */

#define DEFAULT_TARGET   200.0f   /* 上电目标转速 RPM                          */
#define DEFAULT_KP       2.0f     /* 比例                                      */
#define DEFAULT_KI       10.0f    /* 积分                                      */
#define DEFAULT_KD       0.02f    /* 微分                                      */

#define OVERSPEED_RPM    3000.0f  /* 实际转速超过它就停机 (防编码器接反飞车)   */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static PID_t   pid;                          /* PID 控制器            */
static float   target_rpm = DEFAULT_TARGET;  /* 目标转速              */
static uint8_t running    = 1;               /* 1=闭环运行 0=停机     */

static uint8_t open_loop  = 0;               /* 1=开环(DUTY 命令测试) */
static int16_t open_duty  = 0;

/* 下面几个是显示 / 上报用的 */
static float   speed_rpm  = 0.0f;            /* 实际转速              */
static int16_t duty       = 0;               /* 当前输出占空比        */
static float   err        = 0.0f;            /* 误差                  */
static char    last_cmd[20] = "NONE";        /* 最后收到的一条命令    */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static float StrToFloat(const char *s);
static void  HandleCommand(char *s);
static void  Control_Step(void);
static void  Display_Task(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ==================================================================
 *  字符串转浮点 (自己写, 免得为了收命令再引入 scanf)
 * ================================================================== */
static float StrToFloat(const char *s)
{
  float v = 0.0f;
  float f = 0.1f;
  int   neg = 0;

  while ((*s == ' ') || (*s == '=') || (*s == '\t')) s++;

  if (*s == '-') { neg = 1; s++; }
  else if (*s == '+') { s++; }

  while ((*s >= '0') && (*s <= '9')) { v = v * 10.0f + (float)(*s - '0'); s++; }

  if (*s == '.')
  {
    s++;
    while ((*s >= '0') && (*s <= '9')) { v += (float)(*s - '0') * f; f *= 0.1f; s++; }
  }

  return neg ? -v : v;
}

/* ==================================================================
 *  命令解析
 * ================================================================== */
static void HandleCommand(char *s)
{
  /* 记下来, 显示在 OLED 上, 方便确认命令到底收到没有 */
  strncpy(last_cmd, s, sizeof(last_cmd) - 1);
  last_cmd[sizeof(last_cmd) - 1] = '\0';

  if (strncmp(s, "KP", 2) == 0)
  {
    pid.Kp = StrToFloat(s + 2);
  }
  else if (strncmp(s, "KI", 2) == 0)
  {
    pid.Ki = StrToFloat(s + 2);
  }
  else if (strncmp(s, "KD", 2) == 0)
  {
    pid.Kd = StrToFloat(s + 2);
  }
  else if (strncmp(s, "START", 5) == 0)
  {
    open_loop = 0;
    PID_Reset(&pid);
    running = 1;
  }
  else if (strncmp(s, "STOP", 4) == 0)
  {
    running = 0;
    duty    = 0;
    Motor_Set(0);
  }
  else if (strncmp(s, "DUTY", 4) == 0)
  {
    open_duty = (int16_t)StrToFloat(s + 4);
    open_loop = 1;
    running   = 1;
  }
  else if ((s[0] == '-') || (s[0] == '+') || (s[0] == '.') ||
           ((s[0] >= '0') && (s[0] <= '9')))
  {
    /* 纯数字 = 设目标转速, 顺便确保在闭环运行 */
    target_rpm = StrToFloat(s);
    open_loop  = 0;
    PID_Reset(&pid);
    running    = 1;
  }
  else
  {
    /* 不认识的命令, 忽略 */
  }
}

/* ==================================================================
 *  控制: 每 CTRL_MS 跑一次 (由 SysTick 中断里的 Control_Tick 调)
 * ================================================================== */
static void Control_Step(void)
{
  speed_rpm = Encoder_ReadRPM((float)CTRL_MS);

  /* 过速保护: 编码器接反会变成正反馈, 转速一路飙升, 这里兜一下 */
  if ((speed_rpm > OVERSPEED_RPM) || (speed_rpm < -OVERSPEED_RPM))
  {
    running = 0;
    duty    = 0;
    Motor_Set(0);
    return;
  }

  if (running == 0)                       /* 停机 */
  {
    duty = 0;
    err  = 0.0f;
    Motor_Set(0);
    PID_Reset(&pid);
    pid.last_speed = speed_rpm;           /* 重新启动时微分不会突变 */
    return;
  }

  if (open_loop)                          /* 开环测试: 直接给占空比 */
  {
    err  = target_rpm - speed_rpm;
    duty = open_duty;
    Motor_Set(duty);
    return;
  }

  /* 闭环 */
  err  = target_rpm - speed_rpm;
  duty = (int16_t)PID_Calc(&pid, target_rpm, speed_rpm, (float)CTRL_MS / 1000.0f);
  Motor_Set(duty);
}

/* SysTick 每 1ms 进一次, 数到 CTRL_MS 再跑控制, 保证采样周期准确 */
void Control_Tick(void)
{
  static uint32_t cnt = 0;

  if (++cnt >= CTRL_MS)
  {
    cnt = 0;
    Control_Step();
  }
}

/* ==================================================================
 *  OLED 显示: 每次只刷一行(约 3ms), 8 行轮流刷, 一屏 160ms
 * ================================================================== */
static void Display_Task(void)
{
  static uint32_t tick = 0;
  static uint8_t  line = 0;
  char s[32];

  if ((HAL_GetTick() - tick) < 20)
  {
    return;
  }
  tick = HAL_GetTick();

  switch (line)
  {
    case 0: snprintf(s, sizeof(s), "TAR %6d rpm", (int)target_rpm);      break;
    case 1: snprintf(s, sizeof(s), "ACT %6d rpm", (int)speed_rpm);       break;
    case 2: snprintf(s, sizeof(s), "OUT %6d",     (int)duty);            break;
    case 3: snprintf(s, sizeof(s), "ERR %6d rpm", (int)err);             break;
    case 4: snprintf(s, sizeof(s), "P%.2f I%.1f D%.3f", pid.Kp, pid.Ki, pid.Kd); break;
    case 5: snprintf(s, sizeof(s), "%s %s",
                     running ? "RUN" : "STOP",
                     open_loop ? "OPEN" : "CLOSED");                     break;
    case 6: snprintf(s, sizeof(s), "CMD: %s", last_cmd);                 break;
    default: snprintf(s, sizeof(s), "PID MOTOR DEMO");                   break;
  }

  OLED_ShowLine(line, s);

  line = (uint8_t)((line + 1) % 8);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint32_t tick_send = 0;
  char     cmd[24];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  Motor_Init();                                        /* 电机 (PWM + STBY)  */
  Encoder_Init();                                      /* 编码器             */
  PID_Init(&pid, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, MOTOR_PWM_MAX);
  Comm_Init();                                         /* 串口 + 接收中断    */
  OLED_Init();                                         /* 屏幕(没接也不影响) */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 1. 每 10ms 给 VOFA+ 发一帧 */
    if ((HAL_GetTick() - tick_send) >= 10)
    {
      tick_send = HAL_GetTick();
      Comm_SendFrame(target_rpm, speed_rpm, (float)duty, err);
    }

    /* 2. 看看有没有收到串口命令 */
    if (Comm_GetCommand(cmd, sizeof(cmd)))
    {
      HandleCommand(cmd);
    }

    /* 3. 刷一行 OLED */
    Display_Task();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** 内核电压调节器
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** 晶振 + PLL:  HSE 8MHz / 8 * 336 / 2 = 168MHz
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** 总线分频: AHB=168M, APB1=42M, APB2=84M
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
