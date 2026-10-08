/**
  ******************************************************************************
  * @file    motor.c
  * @brief   TB6612FNG 直流有刷电机驱动 (单路 A 通道)
  ******************************************************************************
  */

#include "motor.h"
#include "main.h"
#include "tim.h"

static volatile int16_t s_duty = 0;

void Motor_Init(void)
{
  /* 方向/使能引脚 (PA2 = AIN1, PA3 = AIN2, PA4 = STBY) 的方向配置
     由 CubeMX 生成的 MX_GPIO_Init() 完成, 见 gpio.c 与 motor.ioc。
     这里只负责给出安全初始电平并启动 PWM。 */

  HAL_GPIO_WritePin(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_RESET);

  /* 启动 PWM 通道, 输出 0 */
  HAL_TIM_PWM_Start(&htim2, MOTOR_PWM_CHANNEL);
  __HAL_TIM_SET_COMPARE(&htim2, MOTOR_PWM_CHANNEL, 0);
  s_duty = 0;

  /* TB6612 上电需要一点建立时间后再使能 */
  HAL_Delay(1);
  HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_SET);
}

void Motor_SetDuty(int16_t duty)
{
  uint16_t pwm;

  if (duty > MOTOR_PWM_MAX)
  {
    duty = MOTOR_PWM_MAX;
  }
  else if (duty < -MOTOR_PWM_MAX)
  {
    duty = -MOTOR_PWM_MAX;
  }

  if (duty > 0)
  {
    HAL_GPIO_WritePin(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, GPIO_PIN_RESET);
    pwm = (uint16_t)duty;
  }
  else if (duty < 0)
  {
    HAL_GPIO_WritePin(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, GPIO_PIN_SET);
    pwm = (uint16_t)(-duty);
  }
  else
  {
    HAL_GPIO_WritePin(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, GPIO_PIN_RESET);
    pwm = 0;
  }

  __HAL_TIM_SET_COMPARE(&htim2, MOTOR_PWM_CHANNEL, pwm);
  s_duty = duty;
}

void Motor_Stop(void)
{
  Motor_SetDuty(0);
}

void Motor_Brake(void)
{
  __HAL_TIM_SET_COMPARE(&htim2, MOTOR_PWM_CHANNEL, 0);
  HAL_GPIO_WritePin(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, GPIO_PIN_SET);
  s_duty = 0;
}

void Motor_SetEnable(uint8_t enable)
{
  HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN,
                    (enable != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

int16_t Motor_GetDuty(void)
{
  return s_duty;
}
