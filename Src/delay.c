#include "stm32f1xx_hal.h"
#include "delay.h"

/**
 * @brief This function provides minimum delay in microseconds.
 * @param time specifies the delay time length, in microseconds
 * @retval None
 */
void HAL_Delay_us(uint32_t time) {
  uint32_t tickstart = SysTick->VAL;
  uint32_t ticks = time * (SystemCoreClock/1000000);
  uint32_t tickstop;
  
  if (tickstart < ticks) {
    tickstop = SysTick->LOAD - ticks + tickstart;
    while (SysTick->VAL <= tickstart);
  } else {
    tickstop = tickstart - ticks;
  }
  
  while (SysTick->VAL > tickstop);
}
