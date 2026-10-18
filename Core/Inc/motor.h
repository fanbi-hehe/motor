/**
 * motor.h —— TB6612 电机驱动
 */
#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

#define MOTOR_PWM_MAX   4199      /* TIM2 满占空比 (PWM 20 kHz) */

/** 初始化: 启动 PWM, STBY 拉高使能驱动器 */
void Motor_Init(void);

/** 设置输出: -4199 ~ +4199, 正转/反转/停 */
void Motor_Set(int16_t duty);

#endif
