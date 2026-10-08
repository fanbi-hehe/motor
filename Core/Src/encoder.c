/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   400 线 AB 相霍尔编码器测速 (TIM1 正交编码器接口)
  *
  *   TIM1 时钟 = 168 MHz (APB2 定时器时钟), 16 位计数器, ARR = 65535。
  *   在 10 ms 窗口内即使 65535 计数全部跑满也对应 ~245000 RPM, 不会溢出。
  ******************************************************************************
  */

#include "encoder.h"
#include "main.h"
#include "tim.h"

/* 每个采样周期对应的分钟数: (ENCODER_SAMPLE_MS/1000)/60 */
#define ENCODER_MIN_PER_TICK   (ENCODER_SAMPLE_MS / 60000.0f)

/* delta -> RPM 的换算系数 */
#define ENCODER_RPM_PER_COUNT  (1.0f / ENCODER_COUNTS_PER_REV / ENCODER_MIN_PER_TICK)

static volatile int32_t s_last_cnt = 0;
static volatile int32_t s_delta    = 0;
static volatile float   s_rpm      = 0.0f;

void Encoder_Init(void)
{
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

  __HAL_TIM_SET_COUNTER(&htim1, 0);
  s_last_cnt = 0;
  s_delta    = 0;
  s_rpm      = 0.0f;
}

int32_t Encoder_GetCount(void)
{
  return (int32_t)__HAL_TIM_GET_COUNTER(&htim1);
}

int32_t Encoder_GetDelta(void)
{
  return s_delta;
}

void Encoder_Update(void)
{
  int32_t cur;
  int32_t delta;
  float   rpm;

  cur   = (int32_t)__HAL_TIM_GET_COUNTER(&htim1);
  /* 16 位计数器: 用 int16_t 语义做差, 自动处理 0 <-> 65535 回绕 */
  delta = (int32_t)(int16_t)((uint16_t)cur - (uint16_t)s_last_cnt);
  s_last_cnt = cur;

#if (ENCODER_DIR_INVERT != 0)
  delta = -delta;
#endif

  s_delta = delta;

  /* M 法测速 */
  rpm = (float)delta * ENCODER_RPM_PER_COUNT;

  /* 一阶低通滤波: 抑制量化噪声与齿槽脉动对微分项的放大。
     ENCODER_FILTER_ALPHA 为宏常量, 下面的判断在编译期即被折叠。 */
  if ((ENCODER_FILTER_ALPHA >= 1.0f) || (ENCODER_FILTER_ALPHA <= 0.0f))
  {
    s_rpm = rpm;
  }
  else
  {
    s_rpm += ENCODER_FILTER_ALPHA * (rpm - s_rpm);
  }
}

float Encoder_GetSpeedRPM(void)
{
  return s_rpm;
}

float Encoder_GetSpeedRPS(void)
{
  return s_rpm / 60.0f;
}

void Encoder_Reset(void)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  s_last_cnt = 0;
  s_delta    = 0;
  s_rpm      = 0.0f;
}
