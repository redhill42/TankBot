#include "main.h"
#include "rangefinder.h"
#include "delay.h"

extern TIM_HandleTypeDef htim1;

#define TRIGGER_H   1
#define TRIGGER_L   2
#define MEASURE     3

static __IO uint16_t distance = INVALID_DISTANCE;
static __IO uint8_t state = TRIGGER_H;
static __IO bool measure = false;

void rangefinder_init(void) {
  HAL_TIM_Base_Start_IT(&htim1);
}

uint16_t get_distance(void) {
  return distance;
}

void TimeElapsedCallback(TIM_HandleTypeDef* htim) {
  switch (state) {
  case TRIGGER_H:
    state = TRIGGER_L;
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
    TIM1->CNT = TIM1->ARR - 20;
    break;
  
  case TRIGGER_L:
    state = MEASURE;
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
    break;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
  if (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin)) {
    // raising edge, start measuring
    TIM1->CNT = 0;
  } else {
    // falling edge, measure the pulse width
    uint32_t time = TIM1->CNT;
    TIM1->CNT = 0;
    
    // compute distance from time multiply sound speed
    if (state == MEASURE) {
      distance = (uint16_t)(time*17/100);
      state = TRIGGER_H;
    }
  }
}
