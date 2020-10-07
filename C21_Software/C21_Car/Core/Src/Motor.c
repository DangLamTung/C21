/*
 * Motor.c
 *
 */

#include "Motor.h"
#include "tim.h"

void motor_init(void)
{
	  HAL_TIM_PWM_Start(&MOTOR_TIM, MOTOR0_CH);
	  HAL_TIM_PWM_Start(&MOTOR_TIM, MOTOR1_CH);
}

void go_forward(float pulse_width)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(&MOTOR_TIM, MOTOR0_CH, 999 - pulse_width);
	__HAL_TIM_SET_COMPARE(&MOTOR_TIM, MOTOR1_CH, 999 - pulse_width);
}

void turn_right(float pulse_width)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&MOTOR_TIM, MOTOR1_CH, 999 - pulse_width);
}

void turn_left(float pulse_width)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&MOTOR_TIM, MOTOR0_CH, 999 - pulse_width);
}

void go_backward(float pulse_width)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&MOTOR_TIM, MOTOR0_CH, pulse_width);
	__HAL_TIM_SET_COMPARE(&MOTOR_TIM, MOTOR1_CH, pulse_width);
}

void stop(void)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&MOTOR_TIM, MOTOR0_CH, 0);
	__HAL_TIM_SET_COMPARE(&MOTOR_TIM, MOTOR1_CH, 0);
}


