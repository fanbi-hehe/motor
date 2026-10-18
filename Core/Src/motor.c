/**
 * motor.c —— TB6612 电机驱动
 *
 *   PA0  TIM2_CH1 PWM  -> PWMA
 *   PA2                -> AIN1
 *   PA3                -> AIN2
 *   PA4                -> STBY (高电平使能)
 *
 *   AIN1/AIN2 决定方向, PWMA 决定转速:
 *     1 0 -> 正转     0 1 -> 反转     0 0 -> 停(自由滑行)
 */
#include "motor.h"
#include "main.h"
#include "tim.h"

#define AIN1_PORT   GPIOA
#define AIN1_PIN    GPIO_PIN_2
#define AIN2_PORT   GPIOA
#define AIN2_PIN    GPIO_PIN_3
#define STBY_PORT   GPIOA
#define STBY_PIN    GPIO_PIN_4

void Motor_Init(void)
{
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  Motor_Set(0);
  HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_SET);   /* 使能驱动器 */
}

void Motor_Set(int16_t duty)
{
  uint16_t pwm;

  if (duty >  MOTOR_PWM_MAX) duty =  MOTOR_PWM_MAX;
  if (duty < -MOTOR_PWM_MAX) duty = -MOTOR_PWM_MAX;

  if (duty > 0)                    /* 正转 */
  {
    HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
    pwm = (uint16_t)duty;
  }
  else if (duty < 0)               /* 反转 */
  {
    HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET);
    pwm = (uint16_t)(-duty);
  }
  else                             /* 停 */
  {
    HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
    pwm = 0;
  }

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);
}
