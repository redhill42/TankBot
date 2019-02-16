#include "main.h"
#include "delay.h"

#define CHECK_TIME 10000

extern ADC_HandleTypeDef hadc1;
static uint32_t voltage = UINT32_MAX;
static stopwatch_t check_time;

void battery_check_init(void) {
  HAL_ADC_Start(&hadc1);
}

static void read_adc(void) {
  if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC)) {
    uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
    
    // Clear regular group conversion flag and overrun flag
    // (To ensure of no unknown state from potential previous ADC operations)
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);
    
    // Start ADC conversion on regular group with SW start
    ADC1->CR2 |= ADC_CR2_SWSTART | ADC_CR2_EXTTRIG;
    
    // Battery voltage = ADC_Value * 3300 / 4096 * 3
    // The voltage is divided by 3 1K resistors, so the actual voltage must be multiply by 3
    voltage = adc_value * 3*3300/4096;
  }
}

uint32_t get_battery_voltage(void) {
  if (SW_Elapsed(&check_time, CHECK_TIME)) {
    SW_Reset(&check_time);
    read_adc();
  }
  return voltage;
}

bool battery_low(void) {
  return get_battery_voltage() < 6400;
}
