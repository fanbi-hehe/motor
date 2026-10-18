/**
 * pid.c —— 位置式 PID
 *
 *   输出 = Kp*误差 + 积分累加 - Kd*加速度
 *
 * 两个必要的小处理:
 *   1) 微分只对"实际转速"求导, 不对误差求导。
 *      否则一改目标值, 误差瞬间跳变, 微分项会突然打满(微分冲击)。
 *   2) 输出已经打满时不再继续积分(抗饱和)。
 *      否则输出被限幅期间积分还在猛涨, 等转速追上来时会严重超调。
 */
#include "pid.h"

void PID_Init(PID_t *p, float kp, float ki, float kd, float limit)
{
  p->Kp = kp;
  p->Ki = ki;
  p->Kd = kd;
  p->limit = limit;

  PID_Reset(p);
}

void PID_Reset(PID_t *p)
{
  p->integral   = 0.0f;
  p->last_speed = 0.0f;
  p->d_filt     = 0.0f;
  p->err        = 0.0f;
}

float PID_Calc(PID_t *p, float target, float actual, float dt)
{
  float d;
  float out;

  p->err = target - actual;

  /* ---- 微分: 对实际转速求导, 再做个低通抑制编码器噪声 ---- */
  d = (actual - p->last_speed) / dt;
  p->last_speed  = actual;
  p->d_filt     += 0.3f * (d - p->d_filt);

  /* ---- 积分累加 + 限幅 ---- */
  p->integral += p->Ki * p->err * dt;
  if (p->integral >  p->limit) p->integral =  p->limit;
  if (p->integral < -p->limit) p->integral = -p->limit;

  /* ---- 三项相加 ---- */
  out = p->Kp * p->err + p->integral - p->Kd * p->d_filt;

  /* ---- 抗饱和: 已经打满, 误差还在往同方向推 -> 撤销这次积分 ---- */
  if (((out >  p->limit) && (p->err > 0.0f)) ||
      ((out < -p->limit) && (p->err < 0.0f)))
  {
    p->integral -= p->Ki * p->err * dt;
    out = p->Kp * p->err + p->integral - p->Kd * p->d_filt;
  }

  /* ---- 输出限幅 ---- */
  if (out >  p->limit) out =  p->limit;
  if (out < -p->limit) out = -p->limit;

  return out;
}
