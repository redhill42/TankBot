#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f1xx_hal.h>

extern TIM_HandleTypeDef htim4;
#define MOTOR_TIM (&htim4)

void motor_init(void);
void motor_control(int16_t m1speed, int16_t m2speed);

#ifdef __cplusplus
}
#endif

#endif
