/**
  ******************************************************************************
  * @file    pid.h
  * @brief   直流电机速度环 PID 控制器 (位置式)
  *
  * 特性:
  *   - 微分先行 (Derivative on Measurement): 目标值阶跃时不会产生微分冲击
  *   - 微分低通滤波: 抑制编码器量化噪声被 Kd 放大
  *   - 积分分离: |误差| 超过阈值时暂停积分, 兼顾快速性与超调
  *   - 条件积分抗饱和 (Conditional Integration): 输出饱和且误差同向时冻结积分
  *   - 输出限幅: 直接映射到 PWM 占空比范围
  ******************************************************************************
  */
#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
  /* ---- 整定参数 ---- */
  float Kp;               /* 比例增益 */
  float Ki;               /* 积分增益 */
  float Kd;               /* 微分增益 */

  /* ---- 限幅与非线性 ---- */
  float integral_limit;   /* 积分项限幅 (绝对值) */
  float output_limit;     /* 输出限幅 (绝对值) */
  float sep_zone;         /* 积分分离阈值, <=0 表示不启用 */
  float d_lpf_alpha;      /* 微分低通系数 0<alpha<=1, 1 表示不滤波 */

  /* ---- 内部状态 ---- */
  float integral;         /* 积分累加量 (已经是 Ki*∫e dt) */
  float last_measure;     /* 上一次测量值 (微分先行) */
  float d_filtered;       /* 滤波后的测量值变化率 (单位/s) */

  /* ---- 观测量 ---- */
  float err;              /* 最近一次误差 */
  float p_term;           /* 最近一次 P 分量 */
  float i_term;           /* 最近一次 I 分量 */
  float d_term;           /* 最近一次 D 分量 */
  float out;              /* 最近一次输出 */
  uint8_t saturated;      /* 最近一次输出是否限幅 */
} PID_t;

/**
  * @brief  初始化 PID 控制器
  * @param  pid       控制器对象
  * @param  kp,ki,kd  整定参数
  * @param  i_limit   积分项限幅
  * @param  o_limit   输出限幅 (例如 PWM 满量程)
  * @param  sep_zone  积分分离阈值, 0 表示不启用
  */
void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float i_limit, float o_limit, float sep_zone);

/**
  * @brief  在线修改整定参数 (不清除积分状态)
  */
void PID_SetTunings(PID_t *pid, float kp, float ki, float kd);

/**
  * @brief  设置微分低通系数 (0<alpha<=1)
  */
void PID_SetDerivativeFilter(PID_t *pid, float alpha);

/**
  * @brief  计算一次 PID 输出
  * @param  pid      控制器对象
  * @param  target   目标值 (RPM)
  * @param  measure  实际值 (RPM)
  * @param  dt       控制周期 (s)
  * @retval 限幅后的输出
  */
float PID_Compute(PID_t *pid, float target, float measure, float dt);

/**
  * @brief  复位积分与微分状态 (停机/模式切换时调用)
  */
void PID_Reset(PID_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
