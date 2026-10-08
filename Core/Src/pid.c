/**
  ******************************************************************************
  * @file    pid.c
  * @brief   直流电机速度环 PID 控制器 (位置式)
  *
  *   输出 = Kp*e + ∫(Ki*e)dt - Kd*d(measure)/dt
  *          \___/   \________/    \_______________/
  *            P          I              D(微分先行)
  ******************************************************************************
  */

#include "pid.h"

static float PID_Abs(float x)
{
  return (x < 0.0f) ? -x : x;
}

static float PID_Clamp(float x, float limit)
{
  if (x >  limit) return  limit;
  if (x < -limit) return -limit;
  return x;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float i_limit, float o_limit, float sep_zone)
{
  if (pid == 0)
  {
    return;
  }

  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;

  pid->integral_limit = (i_limit > 0.0f) ? i_limit : 0.0f;
  pid->output_limit   = (o_limit > 0.0f) ? o_limit : 0.0f;
  pid->sep_zone       = sep_zone;
  pid->d_lpf_alpha    = 1.0f;      /* 默认不滤波 */

  PID_Reset(pid);
}

void PID_SetTunings(PID_t *pid, float kp, float ki, float kd)
{
  if (pid == 0) return;
  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;
}

void PID_SetDerivativeFilter(PID_t *pid, float alpha)
{
  if (pid == 0) return;
  if (alpha <= 0.0f) alpha = 1.0f;
  if (alpha >  1.0f) alpha = 1.0f;
  pid->d_lpf_alpha = alpha;
}

float PID_Compute(PID_t *pid, float target, float measure, float dt)
{
  float err;
  float d_raw;
  float i_cand;
  float out;
  float p_term;
  float d_term;
  int   sat_hi;
  int   sat_lo;

  if ((pid == 0) || (dt <= 0.0f))
  {
    return 0.0f;
  }

  err = target - measure;

  /* ---------- 微分先行: 对测量值求导, 再做一阶低通 ---------- */
  d_raw = (measure - pid->last_measure) / dt;
  pid->last_measure = measure;
  pid->d_filtered += pid->d_lpf_alpha * (d_raw - pid->d_filtered);

  /* ---------- 积分: 积分分离 + 限幅 ---------- */
  i_cand = pid->integral;
  if ((pid->sep_zone <= 0.0f) || (PID_Abs(err) <= pid->sep_zone))
  {
    i_cand += pid->Ki * err * dt;
    i_cand  = PID_Clamp(i_cand, pid->integral_limit);
  }

  /* ---------- 三个分量 ---------- */
  p_term = pid->Kp * err;
  d_term = pid->Kd * pid->d_filtered;
  out    = p_term + i_cand - d_term;

  /* ---------- 条件积分抗饱和 ----------
     输出已饱和且误差仍把输出往饱和方向推时, 冻结积分 (不接受新的 i_cand),
     否则积分会持续累积, 造成退饱和时的严重超调。 */
  sat_hi = (out >  pid->output_limit) ? 1 : 0;
  sat_lo = (out < -pid->output_limit) ? 1 : 0;

  if (((sat_hi != 0) && (err > 0.0f)) || ((sat_lo != 0) && (err < 0.0f)))
  {
    i_cand = pid->integral;
    out    = p_term + i_cand - d_term;
  }

  pid->integral = i_cand;

  /* ---------- 输出限幅 ---------- */
  if (out > pid->output_limit)
  {
    out = pid->output_limit;
  }
  else if (out < -pid->output_limit)
  {
    out = -pid->output_limit;
  }

  /* ---------- 记录观测量 ---------- */
  pid->err       = err;
  pid->p_term    = p_term;
  pid->i_term    = i_cand;
  pid->d_term    = d_term;
  pid->out       = out;
  pid->saturated = (uint8_t)((out >= pid->output_limit) || (out <= -pid->output_limit));

  return out;
}

void PID_Reset(PID_t *pid)
{
  if (pid == 0) return;

  pid->integral     = 0.0f;
  pid->last_measure = 0.0f;
  pid->d_filtered   = 0.0f;
  pid->err          = 0.0f;
  pid->p_term       = 0.0f;
  pid->i_term       = 0.0f;
  pid->d_term       = 0.0f;
  pid->out          = 0.0f;
  pid->saturated    = 0u;
}
