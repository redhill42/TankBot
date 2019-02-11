#ifndef __BEEP_H
#define __BEEP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

int beep_start(uint32_t frequency);
void beep_stop(void);

// Private
extern TIM_HandleTypeDef htim3;
#define BEEP_TIM (&htim3)
#define BEEP_TIM_CHANNEL TIM_CHANNEL_1

void beep_pwm_pulse(void);

#ifdef __cplusplus
}
#endif

#endif
