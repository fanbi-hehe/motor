/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   400 线 AB 相霍尔编码器测速 (TIM1 正交编码器接口)
  *
  * 接线:
  *   PE9  = TIM1_CH1 -> 编码器 A 相
  *   PE11 = TIM1_CH2 -> 编码器 B 相
  *
  * 测速方法: M 法 (固定时间窗口计脉冲数)
  *   每 ENCODER_SAMPLE_MS 读取一次 TIM1->CNT, 求增量 delta,
  *   RPM = delta / COUNTS_PER_REV * (60000 / ENCODER_SAMPLE_MS)
  ******************************************************************************
  */
#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================ 参数配置 ============================ */

/** 编码器线数 (铭牌值): 每转 A 相输出 400 个脉冲 */
#define ENCODER_LINES             400.0f

/** 倍频系数: TIM1 配置为 EncoderMode TI12 (AB 双方沿计数) = 4 倍频 */
#define ENCODER_MULTIPLE          4.0f

/** 减速比:
 *    编码器装在电机轴上 -> 填减速比 (例: 30.0f), 显示输出轴转速
 *    编码器装在输出轴上 -> 填 1.0f
 *    只想看电机轴转速   -> 填 1.0f
 */
#define ENCODER_GEAR_RATIO        1.0f

/** 每转计数值 = 线数 x 倍频 x 减速比 */
#define ENCODER_COUNTS_PER_REV    (ENCODER_LINES * ENCODER_MULTIPLE * ENCODER_GEAR_RATIO)

/** 测速采样周期 (ms), 必须与 app.c 中的 CONTROL_PERIOD_MS 一致 */
#define ENCODER_SAMPLE_MS         10.0f

/** 一阶低通滤波系数 (0~1): 越小越平滑, 越大越跟手; 1 表示不滤波 */
#define ENCODER_FILTER_ALPHA      0.40f

/** 编码器方向取反: 若电机正转时读数为负, 改成 1 */
#define ENCODER_DIR_INVERT        0

/* ================================================================== */

/** 初始化: 启动 TIM1 编码器接口, 清零计数 */
void Encoder_Init(void);

/** 读取当前计数 (有符号, 16 位回绕安全) */
int32_t Encoder_GetCount(void);

/** 读取最近一个采样窗口内的计数增量 */
int32_t Encoder_GetDelta(void);

/** 测速更新: 必须每 ENCODER_SAMPLE_MS 调用一次 (在控制中断中) */
void Encoder_Update(void);

/** 最近一次滤波后的转速 (RPM) */
float Encoder_GetSpeedRPM(void);

/** 最近一次滤波后的转速 (转/秒) */
float Encoder_GetSpeedRPS(void);

/** 清零计数与滤波状态 */
void Encoder_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H */
