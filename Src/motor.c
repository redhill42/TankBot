#include <stdlib.h>
#include "main.h"
#include "motor.h"

static int16_t m1speed, m2speed;

static __IO uint32_t up_bsrr;
static __IO uint32_t cc1_bsrr;
static __IO uint32_t cc3_bsrr;

extern DMA_HandleTypeDef hdma_tim2_up;
extern DMA_HandleTypeDef hdma_tim2_ch1;
extern DMA_HandleTypeDef hdma_tim2_ch3;

#define MOTOR_PORT M1P_GPIO_Port

void motor_init(void) {
  // Use DMA to generate PWM waveform
  HAL_DMA_Start_IT(&hdma_tim2_up, (uint32_t)&up_bsrr, (uint32_t)&MOTOR_PORT->BSRR, 1);
  __HAL_TIM_ENABLE_DMA(MOTOR_TIM, TIM_DMA_UPDATE);
  
  HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)&cc1_bsrr, (uint32_t)&MOTOR_PORT->BSRR, 1);
  __HAL_TIM_ENABLE_DMA(MOTOR_TIM, TIM_DMA_CC1);
  TIM_CCxChannelCmd(MOTOR_TIM->Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
  
  HAL_DMA_Start_IT(&hdma_tim2_ch3, (uint32_t)&cc3_bsrr, (uint32_t)&MOTOR_PORT->BSRR, 1);
  __HAL_TIM_ENABLE_DMA(MOTOR_TIM, TIM_DMA_CC3);
  TIM_CCxChannelCmd(MOTOR_TIM->Instance, TIM_CHANNEL_3, TIM_CCx_ENABLE);
  
  __HAL_TIM_ENABLE(MOTOR_TIM);
}

void motor_control(int16_t m1s, int16_t m2s) {
  if (m1s == m1speed && m2s == m2speed)
    return;
  m1speed = m1s;
  m2speed = m2s;
  
  uint32_t tmp = 0;
  
  // set raising edge for motor 1 PWM
  if (m1s > 0) {
    tmp |= M1P_Pin | (M1N_Pin<<16);
  } else if (m1s < 0) {
    tmp |= (M1P_Pin<<16) | M1N_Pin;
  } else {
    tmp |= (M1P_Pin|M1N_Pin) << 16;
  }
  
  // set raising edge for motor 2 PWM
  if (m2s > 0) {
    tmp |= M2P_Pin | (M2N_Pin<<16);
  } else if (m2s < 0) {
    tmp |= (M2P_Pin<<16) | M2N_Pin;
  } else {
    tmp |= (M2P_Pin|M2N_Pin) << 16;
  }
  
  up_bsrr = tmp;
  
  // set falling edge for motor 1 PWM
  if (m1s < 1000 && m1s > -1000) {
    cc1_bsrr = (M1P_Pin|M1N_Pin) << 16;
  } else {
    cc1_bsrr = 0;
  }
  
  // set falling edge for motor 2 PWM
  if (m2s < 1000 && m2s > -1000) {
    cc3_bsrr = (M2P_Pin|M2N_Pin) << 16;
  } else {
    cc3_bsrr = 0;
  }
  
  // set duty cycles for motor PWM
  __HAL_TIM_SET_COMPARE(MOTOR_TIM, TIM_CHANNEL_1, abs(m1speed));
  __HAL_TIM_SET_COMPARE(MOTOR_TIM, TIM_CHANNEL_3, abs(m2speed));
}
