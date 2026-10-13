/**
  ******************************************************************************
  * @file    test_pid_fmt.c
  * @brief   PID / 数值格式化 / 测速换算 的主机端单元测试
  *
  *  这些模块不依赖 HAL, 可以直接用 PC 上的 gcc 编译运行:
  *
  *    cd motor
  *    gcc -std=c11 -Wall -Wextra -I Core/Inc \
  *        tests/test_pid_fmt.c Core/Src/pid.c Core/Src/fmt.c -lm \
  *        -o build/test_pid_fmt
  *    ./build/test_pid_fmt
  *
  *  测速换算部分来自 encoder.h 的宏定义, 用于校验
  *  "400 线 x 4 倍频, 10 ms 采样窗口" 的 RPM 换算系数。
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pid.h"
#include "fmt.h"
#include "encoder.h"
#include "app.h"
#include "vofa.h"

static int g_pass = 0;
static int g_fail = 0;

static void check_int(const char *name, int got, int want)
{
  if (got == want) { g_pass++; printf("  PASS  %-34s = %d\n", name, got); }
  else             { g_fail++; printf("  FAIL  %-34s got %d, want %d\n", name, got, want); }
}

static void check_str(const char *name, const char *got, const char *want)
{
  if (strcmp(got, want) == 0) { g_pass++; printf("  PASS  %-34s = \"%s\"\n", name, got); }
  else { g_fail++; printf("  FAIL  %-34s got \"%s\", want \"%s\"\n", name, got, want); }
}

/* 按长度比较: 帧构造是纯字节流, 不写结束符, 所以不能用 strcmp */
static void check_str_n(const char *name, const char *got, uint16_t len, const char *want)
{
  if ((len == (uint16_t)strlen(want)) && (memcmp(got, want, len) == 0))
  {
    g_pass++;
    printf("  PASS  %-34s = \"%s\" (%u bytes)\n", name, want, (unsigned)len);
  }
  else
  {
    g_fail++;
    printf("  FAIL  %-34s got %u bytes \"%.*s\", want %u bytes \"%s\"\n",
           name, (unsigned)len, (int)len, got, (unsigned)strlen(want), want);
  }
}

static void check_hex(const char *name, const uint8_t *got, const uint8_t *want, int n)
{
  int i;
  for (i = 0; i < n; i++)
  {
    if (got[i] != want[i])
    {
      g_fail++;
      printf("  FAIL  %-34s byte[%d] = 0x%02X, want 0x%02X\n", name, i, got[i], want[i]);
      return;
    }
  }
  g_pass++;
  printf("  PASS  %-34s %d bytes ok\n", name, n);
}

static void check_near(const char *name, double got, double want, double tol)
{
  if (fabs(got - want) <= tol)
  {
    g_pass++;
    printf("  PASS  %-34s = %.6f\n", name, got);
  }
  else
  {
    g_fail++;
    printf("  FAIL  %-34s got %.6f, want %.6f (tol %.6f)\n", name, got, want, tol);
  }
}

/* ------------------------------------------------------------------ */
/* 1. 数值格式化                                                       */
/* ------------------------------------------------------------------ */
static void test_fmt(void)
{
  char b[32];

  printf("\n[1] format helpers\n");

  FMT_Int(b, sizeof(b), 0);          check_str("FMT_Int(0)", b, "0");
  FMT_Int(b, sizeof(b), 42);         check_str("FMT_Int(42)", b, "42");
  FMT_Int(b, sizeof(b), -42);        check_str("FMT_Int(-42)", b, "-42");
  FMT_Int(b, sizeof(b), 2147483647); check_str("FMT_Int(INT32_MAX)", b, "2147483647");
  FMT_Int(b, sizeof(b), (int32_t)-2147483647 - 1); check_str("FMT_Int(INT32_MIN)", b, "-2147483648");

  FMT_Float(b, sizeof(b), 0.0f, 2);        check_str("FMT_Float(0, 2)", b, "0.00");
  FMT_Float(b, sizeof(b), 1200.0f, 2);     check_str("FMT_Float(1200, 2)", b, "1200.00");
  FMT_Float(b, sizeof(b), -12.66f, 2);     check_str("FMT_Float(-12.66, 2)", b, "-12.66");
  FMT_Float(b, sizeof(b), 3.14159f, 2);    check_str("FMT_Float(3.14159, 2)", b, "3.14");
  FMT_Float(b, sizeof(b), 3.14159f, 4);    check_str("FMT_Float(3.14159, 4)", b, "3.1416");
  FMT_Float(b, sizeof(b), -0.0001f, 2);    check_str("FMT_Float(-0.0001, 2)", b, "0.00");
  FMT_Float(b, sizeof(b), 0.005f, 2);      check_str("FMT_Float(0.005, 2)", b, "0.01");
  FMT_Float(b, sizeof(b), 999.999f, 2);    check_str("FMT_Float(999.999, 2)", b, "1000.00");
  FMT_Float(b, sizeof(b), 1.2f, 0);        check_str("FMT_Float(1.2, 0)", b, "1");
  FMT_Float(b, sizeof(b), -7.5f, 0);       check_str("FMT_Float(-7.5, 0)", b, "-8");
  FMT_Float(b, sizeof(b), 0.08f, 3);       check_str("FMT_Float(Ki 0.08, 3)", b, "0.080");
  FMT_Float(b, sizeof(b), 0.0015f, 4);     check_str("FMT_Float(Kd 0.0015, 4)", b, "0.0015");
}

/* ------------------------------------------------------------------ */
/* 2. 测速换算                                                        */
/* ------------------------------------------------------------------ */
static void test_speed_math(void)
{
  float rpm_per_count = 1.0f / ENCODER_COUNTS_PER_REV / (ENCODER_SAMPLE_MS / 60000.0f);

  printf("\n[2] encoder speed conversion (400 lines x4, 10 ms window)\n");

  check_near("counts per revolution", ENCODER_COUNTS_PER_REV, 1600.0, 1e-6);
  check_near("rpm per count", rpm_per_count, 3.75, 1e-4);
  check_near("1600 counts / 10 ms -> rpm", 1600.0f * rpm_per_count, 6000.0, 1e-2);
  check_near("1 count / 10 ms -> rpm", 1.0f * rpm_per_count, 3.75, 1e-4);
  /* 3000 RPM 对应的 10ms 计数 */
  check_near("3000 rpm -> counts", 3000.0f / rpm_per_count, 800.0, 1e-2);
}

/* ------------------------------------------------------------------ */
/* 3. PID: 纯比例                                                     */
/* ------------------------------------------------------------------ */
static void test_pid_proportional(void)
{
  PID_t pid;

  printf("\n[3] PID proportional only\n");

  PID_Init(&pid, 2.0f, 0.0f, 0.0f, 1000.0f, 4199.0f, 0.0f);
  check_near("out = Kp*err (err=100)", PID_Compute(&pid, 100.0f, 0.0f, 0.01f), 200.0, 1e-3);

  PID_Reset(&pid);
  check_near("out = Kp*err (err=-50)", PID_Compute(&pid, 0.0f, 50.0f, 0.01f), -100.0, 1e-3);

  /* 输出限幅 */
  PID_Reset(&pid);
  check_near("output clamp at +limit", PID_Compute(&pid, 10000.0f, 0.0f, 0.01f), 4199.0, 1e-3);
  PID_Reset(&pid);
  check_near("output clamp at -limit", PID_Compute(&pid, -10000.0f, 0.0f, 0.01f), -4199.0, 1e-3);
}

/* ------------------------------------------------------------------ */
/* 4. PID: 积分与抗饱和                                              */
/* ------------------------------------------------------------------ */
static void test_pid_integral(void)
{
  PID_t pid;
  float out = 0.0f;
  int   i;

  printf("\n[4] PID integral + anti-windup\n");

  /* Ki=1, err=100, dt=0.01 -> 每步积分增加 1 */
  PID_Init(&pid, 0.0f, 1.0f, 0.0f, 1000.0f, 4199.0f, 0.0f);
  PID_Compute(&pid, 100.0f, 0.0f, 0.01f);
  check_near("integral after 1 step", pid.integral, 1.0, 1e-4);
  for (i = 0; i < 9; i++) { PID_Compute(&pid, 100.0f, 0.0f, 0.01f); }
  check_near("integral after 10 steps", pid.integral, 10.0, 1e-3);

  /* 积分限幅: Ki=1, err=1000, dt=0.01 -> 每步 +10, 限幅 50 时必须停住 */
  PID_Init(&pid, 0.0f, 1.0f, 0.0f, 50.0f, 1e6f, 0.0f);
  for (i = 0; i < 100; i++) { out = PID_Compute(&pid, 1000.0f, 0.0f, 0.01f); }
  check_near("integral clamped to i_limit", pid.integral, 50.0, 1e-3);
  check_near("output = integral when Kp=0", out, 50.0, 1e-3);

  /* 条件积分抗饱和:
     Kp=0, Ki=1, err=1000, dt=0.01 -> 无抗饱和时积分会涨到 10000;
     有抗饱和时积分只能涨到输出限幅 100 就冻结。 */
  PID_Init(&pid, 0.0f, 1.0f, 0.0f, 1e6f, 100.0f, 0.0f);
  for (i = 0; i < 1000; i++) { out = PID_Compute(&pid, 1000.0f, 0.0f, 0.01f); }
  check_near("integral frozen at output limit", pid.integral, 100.0, 1e-3);
  check_near("output clamped", out, 100.0, 1e-3);
  check_int("saturated flag", (int)pid.saturated, 1);
  if (pid.integral < 200.0f) { g_pass++; printf("  PASS  no windup (raw would be 10000)\n"); }
  else                       { g_fail++; printf("  FAIL  windup: integral = %.1f\n", pid.integral); }

  /* 误差反向 (超调) 时积分必须能被拉回来: 100 - 1*1000*0.01 = 90 */
  out = PID_Compute(&pid, -1000.0f, 0.0f, 0.01f);
  check_near("integral released on reverse error", pid.integral, 90.0, 1e-3);
  check_near("output follows released integral", out, 90.0, 1e-3);
}

/* ------------------------------------------------------------------ */
/* 5. PID: 微分先行 + 微分滤波                                        */
/* ------------------------------------------------------------------ */
static void test_pid_derivative(void)
{
  PID_t pid;

  printf("\n[5] PID derivative-on-measurement\n");

  /* D 只对测量值求导: 目标值阶跃不产生微分冲击。
     测量值恒为 0 时, 无论目标怎么跳变, D 分量都应为 0。 */
  PID_Init(&pid, 1.0f, 0.0f, 100.0f, 0.0f, 1e6f, 0.0f);
  PID_Compute(&pid, 0.0f, 0.0f, 0.01f);
  (void)PID_Compute(&pid, 5000.0f, 0.0f, 0.01f);
  check_near("no derivative kick on target step", pid.d_term, 0.0, 1e-6);

  /* 测量值斜坡: dm/dt = 300 rpm/s, Kd=0.01 -> D 项 = 3.0 */
  PID_Reset(&pid);
  PID_SetTunings(&pid, 0.0f, 0.0f, 0.01f);
  PID_Compute(&pid, 0.0f, 0.0f, 0.01f);
  PID_Compute(&pid, 0.0f, 3.0f, 0.01f);   /* 3 rpm / 0.01 s = 300 rpm/s */
  check_near("D term on ramp", pid.d_term, 3.0, 1e-4);

  /* 微分低通: alpha=0.5 时阶跃响应只走一半 */
  PID_Reset(&pid);
  PID_SetTunings(&pid, 0.0f, 0.0f, 1.0f);
  PID_SetDerivativeFilter(&pid, 0.5f);
  PID_Compute(&pid, 0.0f, 0.0f, 0.01f);
  PID_Compute(&pid, 0.0f, 100.0f, 0.01f); /* 原始变化率 10000 rpm/s */
  check_near("derivative low-pass (alpha=0.5)", pid.d_term, 5000.0, 1e-1);
}

/* ------------------------------------------------------------------ */
/* 6. PID: 积分分离                                                   */
/* ------------------------------------------------------------------ */
static void test_pid_separation(void)
{
  PID_t pid;

  printf("\n[6] PID integral separation\n");

  /* sep_zone = 50: |err| = 100 时不积分 */
  PID_Init(&pid, 0.0f, 1.0f, 0.0f, 1000.0f, 4199.0f, 50.0f);
  PID_Compute(&pid, 100.0f, 0.0f, 0.01f);
  check_near("no integration outside zone", pid.integral, 0.0, 1e-6);

  /* |err| = 20 < 50: 正常积分 */
  PID_Compute(&pid, 20.0f, 0.0f, 0.01f);
  check_near("integration inside zone", pid.integral, 0.2, 1e-5);
}

/* ------------------------------------------------------------------ */
/* 7. 闭环仿真: 一阶电机模型, 验证默认整定参数能收敛且稳态无静差       */
/* ------------------------------------------------------------------ */
#define PLANT_K     (1200.0f / 4199.0f)   /* rpm / PWM 计数 (满占空比 1200 rpm) */
#define PLANT_TAU   0.08f                 /* 机电时间常数 (s) */
#define PLANT_DZ    120.0f                /* 占空比死区 (计数) */

static void sim_plant(float kp, float ki, float kd, float target,
                      int steps, float *final_rpm, float *overshoot_pct,
                      float *settle_s)
{
  PID_t pid;
  float v = 0.0f;
  float max_dev = 0.0f;    /* 沿阶跃方向上超出目标的最大量 */
  float sgn = (target >= 0.0f) ? 1.0f : -1.0f;
  int   i;
  int   settle_step = -1;
  float band = 0.02f * fabsf(target);

  PID_Init(&pid, kp, ki, kd, APP_DEFAULT_I_LIMIT, (float)4199, APP_DEFAULT_SEP_ZONE);
  PID_SetDerivativeFilter(&pid, APP_DEFAULT_D_ALPHA);

  for (i = 0; i < steps; i++)
  {
    float u  = PID_Compute(&pid, target, v, 0.01f);
    float ua = fabsf(u);

    if (ua < PLANT_DZ) { ua = 0.0f; } else { ua -= PLANT_DZ; }
    if (u < 0.0f) { ua = -ua; }

    v += 0.01f * ((PLANT_K * ua) - v) / PLANT_TAU;
    if (((v - target) * sgn) > max_dev) { max_dev = (v - target) * sgn; }

    if ((fabsf(v - target) <= band) && (settle_step < 0))
    {
      int   j;
      int   ok = 1;
      float vv = v;
      float ii = pid.integral;
      float lm = pid.last_measure;
      float df = pid.d_filtered;

      for (j = i; (j < (i + 100)) && (j < steps); j++)
      {
        float u2  = PID_Compute(&pid, target, vv, 0.01f);
        float ua2 = fabsf(u2);
        if (ua2 < PLANT_DZ) { ua2 = 0.0f; } else { ua2 -= PLANT_DZ; }
        if (u2 < 0.0f) { ua2 = -ua2; }
        vv += 0.01f * ((PLANT_K * ua2) - vv) / PLANT_TAU;
        if (fabsf(vv - target) > band) { ok = 0; break; }
      }
      pid.integral = ii;
      pid.last_measure = lm;
      pid.d_filtered = df;
      if (ok) { settle_step = i; }
    }
  }

  *final_rpm     = v;
  *overshoot_pct = max_dev / fabsf(target) * 100.0f;
  *settle_s      = (settle_step >= 0) ? ((float)settle_step * 0.01f) : -1.0f;
}

static void test_pid_closed_loop(void)
{
  float v, ov, st;

  printf("\n[7] closed-loop simulation with shipped defaults (%.2f/%.2f/%.4f)\n",
         (double)APP_DEFAULT_KP, (double)APP_DEFAULT_KI, (double)APP_DEFAULT_KD);
  printf("  INFO  plant: %.0f rpm @ full duty, tau = %.2f s, dead zone %.0f counts\n",
         1200.0, (double)PLANT_TAU, (double)PLANT_DZ);

  sim_plant(APP_DEFAULT_KP, APP_DEFAULT_KI, APP_DEFAULT_KD, 1000.0f, 3000, &v, &ov, &st);
  printf("  INFO  step  1000 rpm -> final %.2f rpm, overshoot %.1f%%, settle %.2f s\n", v, ov, st);
  if (fabs(v - 1000.0f) < 5.0f) { g_pass++; printf("  PASS  steady-state error < 5 rpm\n"); }
  else { g_fail++; printf("  FAIL  steady-state error = %.2f rpm\n", fabs(v - 1000.0f)); }
  if (ov < 10.0f) { g_pass++; printf("  PASS  overshoot < 10%%\n"); }
  else { g_fail++; printf("  FAIL  overshoot = %.1f%%\n", ov); }
  if ((st > 0.0f) && (st < 4.0f)) { g_pass++; printf("  PASS  settles within 4 s\n"); }
  else { g_fail++; printf("  FAIL  settle time = %.2f s\n", st); }

  sim_plant(APP_DEFAULT_KP, APP_DEFAULT_KI, APP_DEFAULT_KD, -800.0f, 3000, &v, &ov, &st);
  printf("  INFO  step  -800 rpm -> final %.2f rpm, overshoot %.1f%%, settle %.2f s\n", v, ov, st);
  if (fabs(v + 800.0f) < 5.0f) { g_pass++; printf("  PASS  reverse direction converges\n"); }
  else { g_fail++; printf("  FAIL  reverse steady-state error = %.2f rpm\n", fabs(v + 800.0f)); }

  /* 低速段: 受编码器 3.75 rpm 量化限制, 允许稍大的误差 */
  sim_plant(APP_DEFAULT_KP, APP_DEFAULT_KI, APP_DEFAULT_KD, 100.0f, 2000, &v, &ov, &st);
  printf("  INFO  step   100 rpm -> final %.2f rpm\n", v);
  if (fabs(v - 100.0f) < 6.0f) { g_pass++; printf("  PASS  low-speed accuracy < 6 rpm\n"); }
  else { g_fail++; printf("  FAIL  low-speed error = %.2f rpm\n", fabs(v - 100.0f)); }
}

/* ------------------------------------------------------------------ */
/* 8. JustFloat 二进制帧构造                                          */
/* ------------------------------------------------------------------ */
static void test_justfloat(void)
{
  uint8_t buf[VOFA_MAX_CHANNELS * 4u + 4u];
  uint8_t expect[VOFA_MAX_CHANNELS * 4u + 4u];
  const uint8_t tail[4] = {0x00u, 0x00u, 0x80u, 0x7Fu};
  float  v[VOFA_MAX_CHANNELS];
  uint16_t len;
  uint8_t  i, k;
  float    back;

  printf("\n[8] JustFloat framing\n");

  /* --- 单值: 1.0f 的 IEEE-754 位型 0x3F800000, 小端应为 00 00 80 3F --- */
  v[0] = 1.0f;
  len = VOFA_BuildJustFloat(buf, v, 1u);
  expect[0] = 0x00u; expect[1] = 0x00u; expect[2] = 0x80u; expect[3] = 0x3Fu;
  check_int("1 channel frame length", (int)len, 8);
  check_hex("1.0f little-endian", buf, expect, 4);
  check_hex("frame tail", &buf[4], tail, 4);

  /* --- 4 通道: 20 字节, 帧尾固定 --- */
  v[0] = 100.5f;    /* 0x42C90000 -> 00 00 C9 42 */
  v[1] = -2.5f;     /* 0xC0200000 -> 00 00 20 C0 */
  v[2] = 0.0f;      /* 0x00000000 -> 00 00 00 00 */
  v[3] = -0.125f;   /* 0xBE000000 -> 00 00 00 BE */
  len = VOFA_BuildJustFloat(buf, v, 4u);
  check_int("4 channels frame length", (int)len, 20);
  expect[0]  = 0x00u; expect[1]  = 0x00u; expect[2]  = 0xC9u; expect[3]  = 0x42u;
  expect[4]  = 0x00u; expect[5]  = 0x00u; expect[6]  = 0x20u; expect[7]  = 0xC0u;
  expect[8]  = 0x00u; expect[9]  = 0x00u; expect[10] = 0x00u; expect[11] = 0x00u;
  expect[12] = 0x00u; expect[13] = 0x00u; expect[14] = 0x00u; expect[15] = 0xBEu;
  check_hex("4 channel payload", buf, expect, 16);
  check_hex("4 channel tail", &buf[16], tail, 4);

  /* --- 8 通道: 36 字节 --- */
  for (i = 0u; i < 8u; i++) { v[i] = (float)i; }
  len = VOFA_BuildJustFloat(buf, v, 8u);
  check_int("8 channels frame length", (int)len, 36);
  check_hex("8 channel tail", &buf[32], tail, 4);
  {
    float b8;
    int   ok8 = 1;
    for (i = 0u; i < 8u; i++)
    {
      memcpy(&b8, &buf[(int)i * 4], 4);
      if (b8 != v[i]) { ok8 = 0; }
    }
    if (ok8 != 0) { g_pass++; printf("  PASS  8 channel values round-trip\n"); }
    else { g_fail++; printf("  FAIL  8 channel values differ\n"); }
  }

  /* --- 往返: 按上位机的方式找帧尾、切帧、还原浮点 --- */
  {
    uint8_t stream[64];
    uint16_t pos = 0u;
    uint8_t  frames = 3u;
    uint8_t  f;

    /* 连续 3 帧, 模拟真实数据流 */
    for (f = 0u; f < frames; f++)
    {
      float src[3];
      src[0] = 100.0f + (float)f;
      src[1] = -50.0f - (float)f;
      src[2] = 0.25f * (float)f;
      pos = (uint16_t)(pos + VOFA_BuildJustFloat(&stream[pos], src, 3u));
    }

    /* 上位机: 扫描 00 00 80 7F, 前面就是 3 个 float */
    {
      int      start = 0;
      uint8_t  nframe = 0u;
      int      j;
      for (j = 0; (j + 3) < (int)pos; j++)
      {
        if ((stream[j] == 0x00u) && (stream[j + 1] == 0x00u) &&
            (stream[j + 2] == 0x80u) && (stream[j + 3] == 0x7Fu))
        {
          int payload = j - start;      /* 帧尾前的字节都是本帧负载 */
          if (payload == 12)
          {
            float got[3];
            for (k = 0u; k < 3u; k++)
            {
              memcpy(&got[k], &stream[start + (int)k * 4], 4);
            }
            if (nframe == 0u)
            {
              check_near("round-trip ch0", got[0], 100.0, 1e-6);
              check_near("round-trip ch1", got[1], -50.0, 1e-6);
              check_near("round-trip ch2", got[2], 0.0, 1e-6);
            }
            nframe++;
          }
          start = j + 4;
          j += 3;
        }
      }
      check_int("round-trip frames decoded", (int)nframe, (int)frames);
    }
  }

  /* --- 解析回读: 每一路都要和写入值完全一致 --- */
  v[0] = 1234.5f; v[1] = -0.001f; v[2] = 4095.75f; v[3] = -30000.0f;
  len = VOFA_BuildJustFloat(buf, v, 4u);
  check_int("4ch length again", (int)len, 20);
  for (i = 0u; i < 4u; i++)
  {
    memcpy(&back, &buf[(int)i * 4], 4);
    if (back == v[i]) { g_pass++; }
    else { g_fail++; printf("  FAIL  value[%d] round-trip %.6f != %.6f\n", i, (double)back, (double)v[i]); }
  }
  if (g_fail == 0) { printf("  PASS  all 4 values bit-exact round-trip\n"); }

  /* --- 帧尾常量必须是 +Inf 的小端表示 --- */
  {
    float inf;
    uint8_t t2[4] = {VOFA_JUSTFLOAT_TAIL_0, VOFA_JUSTFLOAT_TAIL_1,
                     VOFA_JUSTFLOAT_TAIL_2, VOFA_JUSTFLOAT_TAIL_3};
    memcpy(&inf, t2, 4);
    if ((inf > 3.0e38f) && (inf != inf * 0.0f)) { g_pass++; printf("  PASS  tail decodes to +Inf (0x7F800000)\n"); }
    else { g_fail++; printf("  FAIL  tail does not decode to +Inf\n"); }
  }
}

/* ------------------------------------------------------------------ */
/* 9. FireWater ASCII 帧构造 (切回文本协议时使用)                     */
/* ------------------------------------------------------------------ */
static void test_firewater(void)
{
  char     buf[VOFA_FRAME_BUF_SIZE];
  float    v[4];
  uint16_t len;

  printf("\n[9] FireWater framing\n");

  v[0] = 1.5f;
  v[1] = -2.25f;
  len = VOFA_BuildFireWater(buf, (uint16_t)sizeof(buf), v, 2u);
  check_int("2 channels length", (int)len, 11);
  check_str_n("csv + newline", buf, len, "1.50,-2.25\n");

  v[0] = 1500.0f; v[1] = 1489.34f; v[2] = 3821.0f; v[3] = 10.66f;
  len = VOFA_BuildFireWater(buf, (uint16_t)sizeof(buf), v, 4u);
  check_str_n("4 channels csv", buf, len, "1500.00,1489.34,3821.00,10.66\n");
  check_int("ends with newline", (int)buf[len - 1u], '\n');

  /* 缓冲区不足时必须整体失败, 不能发出半帧 */
  len = VOFA_BuildFireWater(buf, 8u, v, 4u);
  check_int("tiny buffer rejected", (int)len, 0);
}

int main(void)
{
  printf("==== motor PID / format unit tests ====\n");

  test_fmt();
  test_speed_math();
  test_pid_proportional();
  test_pid_integral();
  test_pid_derivative();
  test_pid_separation();
  test_pid_closed_loop();
  test_justfloat();
  test_firewater();

  printf("\n==== result: %d passed, %d failed ====\n", g_pass, g_fail);
  return (g_fail == 0) ? 0 : 1;
}
