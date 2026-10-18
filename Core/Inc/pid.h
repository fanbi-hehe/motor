/**
 * pid.h —— 位置式 PID
 */
#ifndef __PID_H
#define __PID_H

typedef struct
{
  float Kp, Ki, Kd;      /* 三个参数, 可以随时改 */
  float limit;           /* 输出限幅 (占空比满量程) */

  float integral;        /* 积分累加值 */
  float last_speed;      /* 上次转速, 用来算微分 */
  float d_filt;          /* 滤波后的加速度 */

  float err;             /* 最近一次误差 (显示用) */
} PID_t;

/** 初始化 (limit 传满占空比, 例如 MOTOR_PWM_MAX) */
void PID_Init(PID_t *p, float kp, float ki, float kd, float limit);

/** 清掉积分和微分状态 (停机 / 换目标值时用) */
void PID_Reset(PID_t *p);

/**
 * 算一次输出
 * @param dt 控制周期, 单位秒 (例如 10ms 传 0.01f)
 * @return 限幅后的输出
 */
float PID_Calc(PID_t *p, float target, float actual, float dt);

#endif
