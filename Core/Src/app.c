/**
  ******************************************************************************
  * @file    app.c
  * @brief   应用层: 速度环调度、VOFA+ 上报、OLED 显示
  ******************************************************************************
  */

#include "app.h"
#include "main.h"
#include "pid.h"
#include "encoder.h"
#include "motor.h"
#include "vofa.h"
#include "oled.h"
#include "fmt.h"
#include "usart.h"
#include <string.h>

/* ============================ 运行状态 ============================ */

static PID_t  s_pid;

static volatile float   s_target_rpm    = 0.0f;   /* 目标转速 (RPM) */
static volatile uint8_t s_enabled       = 0u;     /* 闭环/开环总使能 */
static volatile uint8_t s_closed_loop   = 1u;     /* 1 = 闭环, 0 = 开环 */
static volatile int16_t s_open_duty     = 0;      /* 开环占空比 */
static volatile uint32_t s_safe_ms      = 0u;     /* 通信超时保护 (0 = 关闭) */

static volatile float   s_speed_rpm     = 0.0f;   /* 最近一次实际转速 */
static volatile float   s_err           = 0.0f;   /* 最近一次误差 */

static volatile uint32_t s_ms           = 0u;
static volatile uint8_t  s_tele_due     = 0u;
static volatile uint8_t  s_oled_due     = 0u;

static char s_msg[24] = "BOOT";

/* ============================ 小工具 ============================ */

static float App_Clampf(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static void App_SetMsg(const char *s)
{
  uint8_t i = 0u;

  if (s == 0) return;

  while ((s[i] != '\0') && (i < (uint8_t)(sizeof(s_msg) - 1u)))
  {
    s_msg[i] = s[i];
    i++;
  }
  s_msg[i] = '\0';
}

/* 安全追加 (不会越界, 不做隐式截断告警) */
static void App_Append(char *dst, uint16_t size, const char *s)
{
  uint16_t len = (uint16_t)strlen(dst);

  if (len >= (size - 1u))
  {
    return;
  }
  while ((*s != '\0') && (len < (size - 1u)))
  {
    dst[len++] = *s++;
  }
  dst[len] = '\0';
}

/* 手动解析浮点, 避免引入 scanf / printf 浮点库 */
static uint8_t App_ParseFloat(const char *s, float *out)
{
  float sign = 1.0f;
  float v    = 0.0f;
  float frac = 0.1f;
  uint8_t digits = 0u;

  if ((s == 0) || (out == 0)) return 0u;

  while ((*s == ' ') || (*s == '\t') || (*s == '=') || (*s == ':')) s++;

  if (*s == '-') { sign = -1.0f; s++; }
  else if (*s == '+') { s++; }

  while ((*s >= '0') && (*s <= '9'))
  {
    v = v * 10.0f + (float)(*s - '0');
    s++;
    digits = 1u;
  }

  if (*s == '.')
  {
    s++;
    while ((*s >= '0') && (*s <= '9'))
    {
      v += (float)(*s - '0') * frac;
      frac *= 0.1f;
      s++;
      digits = 1u;
    }
  }

  if (digits == 0u) return 0u;

  *out = sign * v;
  return 1u;
}

/* 命令字匹配 (不区分大小写), 成功后 rest 指向参数 */
static uint8_t App_MatchWord(const char *s, const char *word, const char **rest)
{
  while (*word != '\0')
  {
    char a = *s;
    char b = *word;

    if ((a >= 'a') && (a <= 'z')) a = (char)(a - 32);
    if ((b >= 'a') && (b <= 'z')) b = (char)(b - 32);
    if (a != b) return 0u;

    s++;
    word++;
  }

  if ((*s != '\0') && (*s != ' ') && (*s != '\t') && (*s != '=') && (*s != ':'))
  {
    return 0u;
  }

  while ((*s == ' ') || (*s == '\t') || (*s == '=') || (*s == ':')) s++;
  *rest = s;
  return 1u;
}

/* 打印当前全部参数 */
static void App_DumpParams(void)
{
  char buf[96];
  char n[20];

  /* ---- 整定参数 ---- */
  buf[0] = '\0';
  (void)FMT_Float(n, (int)sizeof(n), s_pid.Kp, 3);
  App_Append(buf, (uint16_t)sizeof(buf), "PID KP=");
  App_Append(buf, (uint16_t)sizeof(buf), n);
  (void)FMT_Float(n, (int)sizeof(n), s_pid.Ki, 3);
  App_Append(buf, (uint16_t)sizeof(buf), " KI=");
  App_Append(buf, (uint16_t)sizeof(buf), n);
  (void)FMT_Float(n, (int)sizeof(n), s_pid.Kd, 4);
  App_Append(buf, (uint16_t)sizeof(buf), " KD=");
  App_Append(buf, (uint16_t)sizeof(buf), n);
  VOFA_SendLine(buf);

  /* ---- 限幅 / 非线性 ---- */
  buf[0] = '\0';
  (void)FMT_Float(n, (int)sizeof(n), s_pid.output_limit, 0);
  App_Append(buf, (uint16_t)sizeof(buf), "LIMIT OUT=");
  App_Append(buf, (uint16_t)sizeof(buf), n);
  (void)FMT_Float(n, (int)sizeof(n), s_pid.integral_limit, 0);
  App_Append(buf, (uint16_t)sizeof(buf), " I=");
  App_Append(buf, (uint16_t)sizeof(buf), n);
  (void)FMT_Float(n, (int)sizeof(n), s_pid.sep_zone, 0);
  App_Append(buf, (uint16_t)sizeof(buf), " SEP=");
  App_Append(buf, (uint16_t)sizeof(buf), n);
  (void)FMT_Float(n, (int)sizeof(n), s_pid.d_lpf_alpha, 2);
  App_Append(buf, (uint16_t)sizeof(buf), " DFILT=");
  App_Append(buf, (uint16_t)sizeof(buf), n);
  VOFA_SendLine(buf);

  /* ---- 运行状态 ---- */
  buf[0] = '\0';
  (void)FMT_Float(n, (int)sizeof(n), s_target_rpm, 1);
  App_Append(buf, (uint16_t)sizeof(buf), "STATE TARGET=");
  App_Append(buf, (uint16_t)sizeof(buf), n);
  (void)FMT_Float(n, (int)sizeof(n), s_speed_rpm, 1);
  App_Append(buf, (uint16_t)sizeof(buf), " SPEED=");
  App_Append(buf, (uint16_t)sizeof(buf), n);
  App_Append(buf, (uint16_t)sizeof(buf), (s_enabled != 0u) ? " ENABLE=1" : " ENABLE=0");
  App_Append(buf, (uint16_t)sizeof(buf), (s_closed_loop != 0u) ? " MODE=CLOSED" : " MODE=OPEN");
  VOFA_SendLine(buf);
}

static void App_Help(void)
{
  VOFA_SendLine("---- VOFA+ PID Motor Speed Control ----");
  VOFA_SendLine("1200            set target rpm (same as TARGET 1200)");
  VOFA_SendLine("TARGET <rpm>    set target speed, negative = reverse");
  VOFA_SendLine("KP <v> KI <v> KD <v>   tune PID gains");
  VOFA_SendLine("LIMIT <v>       output limit (0..4199)");
  VOFA_SendLine("SEP <v>         integral separation zone (0=off)");
  VOFA_SendLine("DFILT <a>       derivative low-pass alpha (0..1)");
  VOFA_SendLine("ILIMIT <v>      integral term clamp");
  VOFA_SendLine("START / STOP    enable / disable the drive");
  VOFA_SendLine("RESET           clear PID internal state");
  VOFA_SendLine("OPEN / CLOSE    open-loop / closed-loop mode");
  VOFA_SendLine("DUTY <v>        open-loop duty (-4199..4199)");
  VOFA_SendLine("SAFE <ms>       stop if no command within ms (0=off)");
  VOFA_SendLine("SAVE            print current settings");
  VOFA_SendLine("HELP            print this help");
}

/* ============================ 命令处理 ============================ */

static void App_OnCommand(const char *cmd)
{
  const char *rest = cmd;
  float v = 0.0f;

  if ((cmd == 0) || (*cmd == '\0'))
  {
    return;
  }

  /* 纯数字: 直接当作目标转速, 在 VOFA+ 发送框里敲一个数字即可 */
  {
    const char *p = cmd;
    uint8_t numeric = 1u;
    uint8_t has_digit = 0u;

    while (*p != '\0')
    {
      if ((*p >= '0') && (*p <= '9')) { has_digit = 1u; }
      else if ((*p != '-') && (*p != '+') && (*p != '.') && (*p != ' '))
      {
        numeric = 0u;
        break;
      }
      p++;
    }

    if ((numeric != 0u) && (has_digit != 0u) && App_ParseFloat(cmd, &v))
    {
      s_target_rpm = App_Clampf(v, -APP_TARGET_LIMIT, APP_TARGET_LIMIT);
      App_SetMsg("TARGET SET");
      return;
    }
  }

  if (App_MatchWord(cmd, "TARGET", &rest) || App_MatchWord(cmd, "SPEED", &rest))
  {
    if (App_ParseFloat(rest, &v))
    {
      s_target_rpm = App_Clampf(v, -APP_TARGET_LIMIT, APP_TARGET_LIMIT);
      App_SetMsg("TARGET SET");
    }
  }
  else if (App_MatchWord(cmd, "KP", &rest))
  {
    if (App_ParseFloat(rest, &v)) { s_pid.Kp = v; App_SetMsg("KP SET"); }
  }
  else if (App_MatchWord(cmd, "KI", &rest))
  {
    if (App_ParseFloat(rest, &v)) { s_pid.Ki = v; App_SetMsg("KI SET"); }
  }
  else if (App_MatchWord(cmd, "KD", &rest))
  {
    if (App_ParseFloat(rest, &v)) { s_pid.Kd = v; App_SetMsg("KD SET"); }
  }
  else if (App_MatchWord(cmd, "ILIMIT", &rest))
  {
    if (App_ParseFloat(rest, &v) && (v >= 0.0f))
    {
      s_pid.integral_limit = v;
      App_SetMsg("ILIMIT SET");
    }
  }
  else if (App_MatchWord(cmd, "LIMIT", &rest))
  {
    if (App_ParseFloat(rest, &v))
    {
      v = App_Clampf(v, 1.0f, (float)MOTOR_PWM_MAX);
      s_pid.output_limit = v;
      App_SetMsg("LIMIT SET");
    }
  }
  else if (App_MatchWord(cmd, "SEP", &rest))
  {
    if (App_ParseFloat(rest, &v) && (v >= 0.0f))
    {
      s_pid.sep_zone = v;
      App_SetMsg("SEP SET");
    }
  }
  else if (App_MatchWord(cmd, "DFILT", &rest))
  {
    if (App_ParseFloat(rest, &v))
    {
      PID_SetDerivativeFilter(&s_pid, v);
      App_SetMsg("DFILT SET");
    }
  }
  else if (App_MatchWord(cmd, "SAFE", &rest))
  {
    if (App_ParseFloat(rest, &v) && (v >= 0.0f))
    {
      s_safe_ms = (uint32_t)v;
      App_SetMsg("SAFE SET");
    }
  }
  else if (App_MatchWord(cmd, "DUTY", &rest))
  {
    if (App_ParseFloat(rest, &v))
    {
      v = App_Clampf(v, -(float)MOTOR_PWM_MAX, (float)MOTOR_PWM_MAX);
      s_open_duty   = (int16_t)v;
      s_closed_loop = 0u;
      App_SetMsg("OPEN LOOP DUTY");
    }
  }
  else if (App_MatchWord(cmd, "START", &rest))
  {
    s_enabled = 1u;
    PID_Reset(&s_pid);
    App_SetMsg("START");
  }
  else if (App_MatchWord(cmd, "STOP", &rest))
  {
    s_enabled     = 0u;
    s_target_rpm  = 0.0f;
    s_open_duty   = 0;
    Motor_Stop();
    PID_Reset(&s_pid);
    App_SetMsg("STOP");
  }
  else if (App_MatchWord(cmd, "RESET", &rest))
  {
    PID_Reset(&s_pid);
    Encoder_Reset();
    App_SetMsg("PID RESET");
  }
  else if (App_MatchWord(cmd, "OPEN", &rest))
  {
    s_closed_loop = 0u;
    PID_Reset(&s_pid);
    App_SetMsg("MODE OPEN");
  }
  else if (App_MatchWord(cmd, "CLOSE", &rest) || App_MatchWord(cmd, "CLOSED", &rest))
  {
    s_closed_loop = 1u;
    PID_Reset(&s_pid);
    App_SetMsg("MODE CLOSED");
  }
  else if (App_MatchWord(cmd, "SAVE", &rest) || App_MatchWord(cmd, "DUMP", &rest))
  {
    App_DumpParams();
    App_SetMsg("PARAMS SAVED");
  }
  else if (App_MatchWord(cmd, "HELP", &rest) || App_MatchWord(cmd, "?", &rest))
  {
    App_Help();
    App_SetMsg("HELP SENT");
  }
  else
  {
    App_SetMsg("UNKNOWN CMD");
    VOFA_SendLine("ERR unknown command, send HELP");
  }
}

/* ============================ 控制步进 (10 ms, 中断内) ============================ */

static void App_ControlStep(void)
{
  float out;

  Encoder_Update();
  s_speed_rpm = Encoder_GetSpeedRPM();

  /* 通信超时保护 */
  if ((s_safe_ms > 0u) && (s_enabled != 0u))
  {
    if ((HAL_GetTick() - VOFA_GetLastRxTick()) > s_safe_ms)
    {
      s_enabled    = 0u;
      s_target_rpm = 0.0f;
      s_open_duty  = 0;
      Motor_Stop();
    }
  }

  if (s_enabled == 0u)
  {
    Motor_Stop();
    PID_Reset(&s_pid);
    s_err = 0.0f;
    return;
  }

  if (s_closed_loop != 0u)
  {
    out = PID_Compute(&s_pid, s_target_rpm, s_speed_rpm,
                      (float)APP_CONTROL_PERIOD_MS / 1000.0f);
    s_err = s_pid.err;
    Motor_SetDuty((int16_t)out);
  }
  else
  {
    s_err = s_target_rpm - s_speed_rpm;
    Motor_SetDuty(s_open_duty);
  }
}

/* ============================ 上报 (主循环) ============================ */

static void App_TelemetryTask(void)
{
  float frame[4];

  frame[0] = s_target_rpm;                       /* 期望值 */
  frame[1] = s_speed_rpm;                        /* 实际值 */
  frame[2] = (float)Motor_GetDuty();             /* 当前输出 (占空比) */
  frame[3] = s_err;                              /* 误差 */

  VOFA_SendSamples(frame, 4u);
}

/* ============================ 显示 (主循环) ============================ */

#if (OLED_HEIGHT >= 64u)
static void App_DisplayTask(void)
{
  char b1[16];
  char b2[16];
  char b3[16];

  if (OLED_IsPresent() == 0u)
  {
    return;
  }

  /* ---------- 设定值 (12x16 大字) ---------- */
  (void)FMT_Float(b1, (int)sizeof(b1), s_target_rpm, 0);
  OLED_DrawStringScaledPad(0u, 0u, b1, 2u, 6u);
  OLED_DrawString(76u, 4u, "SET rpm");

  /* ---------- 实际值 (12x16 大字) ---------- */
  (void)FMT_Float(b1, (int)sizeof(b1), s_speed_rpm, 0);
  OLED_DrawStringScaledPad(0u, 16u, b1, 2u, 6u);
  OLED_DrawString(76u, 20u, "ACT rpm");

  /* ---------- 误差与输出 ---------- */
  OLED_DrawString(0u, 32u, "ERR");
  (void)FMT_Int(b1, (int)sizeof(b1), (int32_t)s_err);
  OLED_DrawStringPad(18u, 32u, b1, 6u);

  OLED_DrawString(60u, 32u, "OUT");
  (void)FMT_Int(b1, (int)sizeof(b1), (int32_t)Motor_GetDuty());
  OLED_DrawStringPad(78u, 32u, b1, 5u);
  if (s_pid.saturated != 0u)
  {
    OLED_DrawString(110u, 32u, "SAT");
  }
  else
  {
    OLED_DrawStringPad(110u, 32u, "", 3u);
  }

  /* ---------- PID 参数 ---------- */
  (void)FMT_Float(b1, (int)sizeof(b1), s_pid.Kp, 2);
  (void)FMT_Float(b2, (int)sizeof(b2), s_pid.Ki, 3);
  (void)FMT_Float(b3, (int)sizeof(b3), s_pid.Kd, 4);
  OLED_DrawString(0u, 40u, "P");
  OLED_DrawStringPad(6u, 40u, b1, 6u);
  OLED_DrawString(42u, 40u, "I");
  OLED_DrawStringPad(48u, 40u, b2, 6u);
  OLED_DrawString(84u, 40u, "D");
  OLED_DrawStringPad(90u, 40u, b3, 6u);

  /* ---------- 状态 ---------- */
  OLED_DrawString(0u, 48u, s_closed_loop ? "CL" : "OP");
  OLED_DrawString(18u, 48u, s_enabled ? "EN" : "--");
  OLED_DrawString(36u, 48u, (HAL_GetTick() - VOFA_GetLastRxTick()) < 2000u ? "LNK" : "   ");
  OLED_DrawString(60u, 48u, "RPM");
  (void)FMT_Int(b1, (int)sizeof(b1), Encoder_GetDelta());
  OLED_DrawString(84u, 48u, "d");
  OLED_DrawStringPad(90u, 48u, b1, 6u);

  /* ---------- 最近一条命令 ---------- */
  OLED_DrawStringPad(0u, 56u, s_msg, 21u);

  OLED_Refresh();
}
#else  /* 128x32 精简布局 */
static void App_DisplayTask(void)
{
  char b1[16];
  char b2[16];
  char b3[16];

  if (OLED_IsPresent() == 0u)
  {
    return;
  }

  (void)FMT_Float(b1, (int)sizeof(b1), s_target_rpm, 0);
  OLED_DrawString(0u, 0u, "TAR");
  OLED_DrawStringPad(24u, 0u, b1, 6u);
  OLED_DrawString(66u, 0u, "rpm");

  (void)FMT_Float(b1, (int)sizeof(b1), s_speed_rpm, 0);
  OLED_DrawString(0u, 8u, "ACT");
  OLED_DrawStringPad(24u, 8u, b1, 6u);
  OLED_DrawString(66u, 8u, "rpm");

  (void)FMT_Int(b1, (int)sizeof(b1), (int32_t)s_err);
  (void)FMT_Int(b2, (int)sizeof(b2), (int32_t)Motor_GetDuty());
  OLED_DrawString(0u, 16u, "E");
  OLED_DrawStringPad(6u, 16u, b1, 5u);
  OLED_DrawString(42u, 16u, "O");
  OLED_DrawStringPad(48u, 16u, b2, 5u);
  OLED_DrawString(78u, 16u, s_enabled ? "EN" : "--");

  (void)FMT_Float(b3, (int)sizeof(b3), s_pid.Kp, 2);
  OLED_DrawString(0u, 24u, "P");
  OLED_DrawStringPad(6u, 24u, b3, 5u);
  OLED_DrawString(42u, 24u, s_closed_loop ? "CLOSE" : "OPEN");

  OLED_Refresh();
}
#endif

/* ============================ 对外接口 ============================ */

void App_Init(void)
{
  /* 1. 编码器 (TIM1 正交解码) */
  Encoder_Init();

  /* 2. 电机驱动 (TB6612) */
  Motor_Init();
  Motor_Stop();

  /* 3. PID 默认参数 */
  PID_Init(&s_pid, APP_DEFAULT_KP, APP_DEFAULT_KI, APP_DEFAULT_KD,
           APP_DEFAULT_I_LIMIT, (float)MOTOR_PWM_MAX, APP_DEFAULT_SEP_ZONE);
  PID_SetDerivativeFilter(&s_pid, APP_DEFAULT_D_ALPHA);

  /* 4. OLED (未接屏不影响控制) */
  (void)OLED_Init();

  /* 5. VOFA+ 串口 */
  VOFA_Init();

  VOFA_SendLine("");
  VOFA_SendLine("=== STM32F407 DC Motor Speed Control (PID) ===");
  App_Help();

  s_ms = 0u;
  App_SetMsg("READY");
}

void App_Tick(void)
{
  s_ms++;

  if ((s_ms % APP_CONTROL_PERIOD_MS) == 0u)
  {
    App_ControlStep();
  }
  if ((s_ms % APP_TELEMETRY_PERIOD_MS) == 0u)
  {
    s_tele_due = 1u;
  }
  if ((s_ms % APP_OLED_PERIOD_MS) == 0u)
  {
    s_oled_due = 1u;
  }
}

void App_Loop(void)
{
  /* 1. 处理 VOFA+ 下发命令 */
  VOFA_Poll(App_OnCommand);

  /* 2. FireWater 数据上报 */
  if (s_tele_due != 0u)
  {
    s_tele_due = 0u;
    App_TelemetryTask();
  }

  /* 3. OLED 显示 */
  if (s_oled_due != 0u)
  {
    s_oled_due = 0u;
    App_DisplayTask();
  }
}
