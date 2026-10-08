/**
  ******************************************************************************
  * @file    app.h
  * @brief   应用层: 速度环调度、VOFA+ 上报、OLED 显示
  *
  *   SysTick (1 ms) -> App_Tick()
  *       每 10 ms : 编码器测速 + PID + 电机输出   (中断内, 采样周期确定)
  *       每 50 ms : 置上报标志
  *       每 100ms : 置显示标志
  *
  *   main while(1)  -> App_Loop()
  *       解析 VOFA+ 下发的 ASCII 命令
  *       FireWater 帧上报 (阻塞发送放主循环, 不占用中断)
  *       OLED 刷新 (只推送脏页)
  ******************************************************************************
  */
#ifndef __APP_H
#define __APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** 控制周期 (ms), 必须与 ENCODER_SAMPLE_MS 一致 */
#define APP_CONTROL_PERIOD_MS    10u
/** VOFA+ 上报周期 (ms) */
#define APP_TELEMETRY_PERIOD_MS  20u
/** OLED 刷新周期 (ms) */
#define APP_OLED_PERIOD_MS       100u

/** 上电默认整定参数 (可被 VOFA+ 在线修改)
 *  单位: Kp/Ki/Kd 的输出量纲是 PWM 计数 (满量程 4199), 输入量纲是 RPM。
 *  这组值来自一阶电机模型 (满占空比 ~1200 rpm, 时间常数 80 ms, 死区 ~3%)
 *  的仿真: 1000 rpm 阶跃约 1.7 s 稳定, 基本无超调, 属于保守的起步值。
 *  实际电机请按 README 的整定步骤微调。 */
#define APP_DEFAULT_KP           2.00f
#define APP_DEFAULT_KI           10.0f
#define APP_DEFAULT_KD           0.02f
/** 积分项限幅 (输出量纲): 取满量程, 允许积分单独撑起满占空比 */
#define APP_DEFAULT_I_LIMIT      4199.0f
/** 积分分离阈值 (RPM), 0 = 不启用 */
#define APP_DEFAULT_SEP_ZONE     0.0f
/** 微分低通系数 (抑制编码器量化噪声被 Kd 放大) */
#define APP_DEFAULT_D_ALPHA      0.30f
/** 目标转速绝对值上限 (RPM), 防止误输入 */
#define APP_TARGET_LIMIT         30000.0f

/** 初始化: 各驱动 + PID 默认参数 */
void App_Init(void);

/** 1 ms 周期调用 (SysTick 中断) */
void App_Tick(void);

/** 主循环调用 (永不返回) */
void App_Loop(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_H */
