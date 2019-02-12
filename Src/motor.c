#include <stdlib.h>
#include "main.h"
#include "motor.h"
#include "cmsis_os.h"

static int16_t  m1spd, m2spd;
static uint32_t m1pwm, m2pwm;
static uint32_t pwmup;

extern DMA_HandleTypeDef hdma_tim4_up;

#define MOTOR_PORT M1P_GPIO_Port

void motor_init(void) {
  // Use DMA to generate PWM waveform
  HAL_DMA_Start_IT(&hdma_tim4_up, (uint32_t)&pwmup, (uint32_t)&MOTOR_PORT->BSRR, 1);
  __HAL_TIM_ENABLE_DMA(MOTOR_TIM, TIM_DMA_UPDATE);
  __HAL_TIM_ENABLE(MOTOR_TIM);
  
  HAL_TIM_OC_Start_IT(MOTOR_TIM, TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(MOTOR_TIM, TIM_CHANNEL_2);
}

uint8_t motor_control(int16_t m1s, int16_t m2s) {
  uint32_t ctl = 0, m1p = 0, m2p = 0;
  
  if (m1s == m1spd && m2s == m2spd) {
    return 0;
  }
  
  // set raising edge for motor 1 PWM
  if (m1s > 0) {
    ctl |= M1P_Pin | (M1N_Pin<<16);
    m1p = m1s;
  } else if (m1s < 0) {
    ctl |= (M1P_Pin<<16) | M1N_Pin;
    m1p = -m1s;
  } else {
    ctl |= (M1P_Pin|M1N_Pin) << 16;
    m1p = 0;
  }
  
  // set raising edge for motor 2 PWM
  if (m2s > 0) {
    ctl |= M2P_Pin | (M2N_Pin<<16);
    m2p = m2s;
  } else if (m2s < 0) {
    ctl |= (M2P_Pin<<16) | M2N_Pin;
    m2p = -m2s;
  } else {
    ctl |= (M2P_Pin|M2N_Pin) << 16;
    m2p = 0;
  }

  // update PWM data
  portENTER_CRITICAL();
  m1spd = m1s;
  m2spd = m2s;
  pwmup = ctl;
  m1pwm = m1p;
  m2pwm = m2p;
  __HAL_TIM_SET_COMPARE(MOTOR_TIM, TIM_CHANNEL_1, m1p);
  __HAL_TIM_SET_COMPARE(MOTOR_TIM, TIM_CHANNEL_2, m2p);
  portEXIT_CRITICAL();
  
  return 1;
}

void motor_pwm_pulse(HAL_TIM_ActiveChannel channel) {
  switch (channel) {
    case HAL_TIM_ACTIVE_CHANNEL_1:
      if (m1pwm>0 && m1pwm<1000)
        MOTOR_PORT->BRR = M1P_Pin | M1N_Pin;
      break;

    case HAL_TIM_ACTIVE_CHANNEL_2:
      if (m2pwm>0 && m2pwm<1000)
        MOTOR_PORT->BRR = M2P_Pin | M2N_Pin;
      break;
      
    default:
      break;
  }
}
