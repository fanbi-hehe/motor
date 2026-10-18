/**
 * encoder.c —— 编码器测速 (M 法: 固定时间数一次脉冲)
 *
 *   转速(RPM) = 这段时间内的计数增量 / 每转计数 / 时间(min)
 */
#include "encoder.h"
#include "main.h"
#include "tim.h"

#define COUNTS_PER_REV   (ENCODER_LINES * ENCODER_X4 * GEAR_RATIO)

static int32_t last_cnt = 0;      /* 上次读到的计数值 */

void Encoder_Init(void)
{
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  last_cnt = (int32_t)__HAL_TIM_GET_COUNTER(&htim1);
}

float Encoder_ReadRPM(float dt_ms)
{
  static float filt = 0.0f;       /* 滤波后的转速 */
  int32_t now   = (int32_t)__HAL_TIM_GET_COUNTER(&htim1);
  /* 用 int16 做差, 16 位计数器从 65535 回绕到 0 时也能算对 */
  int16_t delta = (int16_t)(now - last_cnt);
  float   rpm;

  last_cnt = now;

  /* delta / 1600 / (10ms/60000) = delta * 3.75 */
  rpm = (float)delta * 60000.0f / (COUNTS_PER_REV * dt_ms) * ENCODER_DIR;

  /* 一阶低通, 让转速平滑一点。想更跟手就把 0.4 调大 (1.0 = 不滤波) */
  filt += 0.4f * (rpm - filt);

  return filt;
}
