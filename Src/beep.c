#include "main.h"
#include "cmsis_os.h"
#include "beep.h"
#include "servo.h"

void tone(uint32_t frequency) {
  uint32_t period = 1000000/frequency;
  
  __HAL_TIM_SET_COUNTER(BEEP_TIM, 0);
  __HAL_TIM_SET_AUTORELOAD(BEEP_TIM, period-1);
  __HAL_TIM_SET_COMPARE(BEEP_TIM, BEEP_TIM_CHANNEL, period*8/10);
  
  HAL_TIM_Base_Start_IT(BEEP_TIM);
  HAL_TIM_OC_Start_IT(BEEP_TIM, BEEP_TIM_CHANNEL);
}

void no_tone(void) {
  HAL_TIM_Base_Stop_IT(BEEP_TIM);
  HAL_TIM_OC_Stop_IT(BEEP_TIM, BEEP_TIM_CHANNEL);
}

void beep_pwm_pulse(void) {
  HAL_GPIO_TogglePin(BEEP_GPIO_Port, BEEP_Pin);
}
