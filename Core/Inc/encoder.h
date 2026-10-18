/**
 * encoder.h —— 400 线 AB 相编码器测速
 *
 *   PE9  TIM1_CH1 -> A 相
 *   PE11 TIM1_CH2 -> B 相
 */
#ifndef __ENCODER_H
#define __ENCODER_H

/**
 * ============ 编码器参数: 换电机/编码器改这里 ============
 * 每转计数 = 线数 x 倍频 x 减速比 = 400 x 4 x 1 = 1600
 * 10ms 窗口下, 1 个计数 = 3.75 RPM
 */
#define ENCODER_LINES   400.0f   /* 编码器线数 (铭牌值)                          */
#define ENCODER_X4      4.0f     /* 4 倍频 (TIM1 用 TI12 模式, AB 双方沿都计数)  */
#define GEAR_RATIO      1.0f     /* 减速比: 编码器在电机轴填减速比, 在输出轴填 1 */
#define ENCODER_DIR     1.0f     /* 转向反了就改成 -1.0f                         */

/** 初始化: 启动 TIM1 正交解码, 记录当前计数 */
void Encoder_Init(void);

/**
 * 读一次转速
 * @param dt_ms 距离上次调用的时间(ms), 例如控制周期 10
 * @return 转速 RPM (已做一阶低通)
 */
float Encoder_ReadRPM(float dt_ms);

#endif
