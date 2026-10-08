/**
  ******************************************************************************
  * @file    motor.h
  * @brief   TB6612FNG 直流有刷电机驱动 (单路 A 通道)
  *
  * 接线:
  *   PA0  = TIM2_CH1 -> TB6612 PWMA   (PWM, 20 kHz)
  *   PA2             -> TB6612 AIN1   (方向)
  *   PA3             -> TB6612 AIN2   (方向)
  *   PA4             -> TB6612 STBY   (使能, 高有效)
  *   GND             -> TB6612 GND / VM 电源共地
  *
  * TB6612 真值表 (STBY = 1):
  *   AIN1=1, AIN2=0, PWM 有效 -> 正转
  *   AIN1=0, AIN2=1, PWM 有效 -> 反转
  *   AIN1=0, AIN2=0           -> 停止 (输出高阻, 自由滑行)
  *   AIN1=1, AIN2=1           -> 刹车 (输出短接, 快速制动)
  ******************************************************************************
  */
#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* TIM2: 84 MHz / (0+1) / (4199+1) = 20 kHz, 占空比满量程 */
#define MOTOR_PWM_MAX      4199

/* 方向引脚 */
#define MOTOR_AIN1_PORT    GPIOA
#define MOTOR_AIN1_PIN     GPIO_PIN_2
#define MOTOR_AIN2_PORT    GPIOA
#define MOTOR_AIN2_PIN     GPIO_PIN_3
#define MOTOR_STBY_PORT    GPIOA
#define MOTOR_STBY_PIN     GPIO_PIN_4

/* PWM 通道: TIM2_CH1 -> PA0 */
#define MOTOR_PWM_CHANNEL  TIM_CHANNEL_1

/** 初始化: 配置方向/使能 GPIO, 启动 PWM, 初始输出 0, 并拉高 STBY */
void Motor_Init(void);

/**
  * @brief  设置输出占空比 (带符号)
  * @param  duty 取值范围 [-MOTOR_PWM_MAX, +MOTOR_PWM_MAX]
  *              正 -> 正转, 负 -> 反转, 0 -> 自由停止
  */
void Motor_SetDuty(int16_t duty);

/** 滑行停止: PWM=0 且 AIN1=AIN2=0 */
void Motor_Stop(void);

/** 刹车: AIN1=AIN2=1 且 PWM=0, 电机绕组短接制动 */
void Motor_Brake(void);

/** 驱动器使能 (STBY 引脚) */
void Motor_SetEnable(uint8_t enable);

/** 读取最近一次设定的占空比 */
int16_t Motor_GetDuty(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
