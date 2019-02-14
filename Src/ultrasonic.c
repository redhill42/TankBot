#include "main.h"
#include "cmsis_os.h"
#include "rangefinder.h"
#include "delay.h"

#define IDLE      0
#define TRIGGER   1
#define MEASURE   2

static __IO uint16_t distance = INVALID_DISTANCE;
static __IO uint8_t state = IDLE;
static uint32_t start_count, start_tick;
extern osTimerId ultrasonic_timerHandle;

void rangefinder_init(void) {
  osTimerStart(ultrasonic_timerHandle, 50);
}

uint16_t get_distance(void) {
  return distance;
}

void ultrasonic_task(void const* args) {
  static uint8_t timeout;
  
  if (state!=IDLE && ++timeout>20) {
    state = IDLE;
  }
  
  if (state == IDLE) {
    state = TRIGGER;
    timeout = 0;
    
    // trigger the pulse
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
    HAL_Delay_us(10);
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin) {
  GPIO_PinState pin = HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin);
  
  if (state==TRIGGER && pin==GPIO_PIN_SET) {
    // raising edge, start measuring
    start_tick  = HAL_GetTick();
    start_count = SysTick->VAL;
    state       = MEASURE;
  } else if (state==MEASURE && pin==GPIO_PIN_RESET) {
    // falling edge, measure the pulse width
    uint32_t stop_tick  = HAL_GetTick();
    uint32_t stop_count = SysTick->VAL;
    uint32_t period     = SysTick->LOAD;
    uint32_t elapsed_time;
    
    // handle counter overflow
    if (start_tick == stop_tick) {
      elapsed_time = start_count - stop_count;
    } else {
      elapsed_time = start_count + (stop_tick-start_tick-1)*period + (period-stop_count);
    }
    
    // convert system clock counter to microseconds
    elapsed_time /= period/1000;
    
    // compute distance from time multiply by sound speed
    distance = (uint16_t)(elapsed_time*17/100);
    state = IDLE;
  }
}
