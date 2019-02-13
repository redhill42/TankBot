#ifndef __BEEP_H
#define __BEEP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void tone(uint32_t frequency);
void no_tone(void);

// Private
extern TIM_HandleTypeDef htim3;
#define BEEP_TIM (&htim3)
#define BEEP_TIM_CHANNEL TIM_CHANNEL_1

void beep_pwm_pulse(void);

#ifdef __cplusplus
}
#endif

#endif
