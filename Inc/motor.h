#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void motor_init(void);
bool motor_control(int16_t m1speed, int16_t m2speed);
void get_motor_speed(int16_t* m1speed, int16_t* m2speed);

// Private
extern TIM_HandleTypeDef htim4;
#define MOTOR_TIM (&htim4)
void motor_pwm_pulse(HAL_TIM_ActiveChannel channel);

#ifdef __cplusplus
}
#endif

#endif
