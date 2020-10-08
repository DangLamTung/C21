/*
 * Motor.h
 *
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>

#define MOTOR_TIM htim3
#define MOTOR0_CH TIM_CHANNEL_3
#define MOTOR1_CH TIM_CHANNEL_4


// Function Prototypes
void motor_init(void);
void go_forward(float pulse_width);
void turn_right(float pulse_width);
void turn_left(float pulse_width);
void go_backward(float pulse_width);
void stop(void);

#endif /* CONTROL_MOTOR_H_ */
