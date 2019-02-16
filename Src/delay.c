#include "main.h"
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

/**
 * @brief Start the stop watch.
 */
void SW_Start(stopwatch_t* sw) {
  *sw = SysTick->VAL;
}

/**
 * @brief Stop the stop watch and return the time elapsed in microseconds.
 */
uint32_t SW_Stop(stopwatch_t* sw) {
  uint32_t tickstart = *sw;
  uint32_t tickstop  = SysTick->VAL;
  uint32_t ticks;
  
  if (tickstart > tickstop) {
    ticks = tickstart - tickstop;
  } else {
    ticks = SysTick->LOAD - tickstop + tickstart;
  }
  
  *sw = tickstop;
  return ticks / (SystemCoreClock/1000000);
}

/**
 * @brief Reset the stop watch.
 */
void SW_Reset(stopwatch_t* sw) {
  *sw = HAL_GetTick();
}

/**
 * @brief Determine if the stop watch was elapsed the give time.
 */
bool SW_Elapsed(stopwatch_t* sw, uint32_t time) {
  uint32_t tickstart = *sw;
  uint32_t tickstop = HAL_GetTick();
  
  if (tickstop >= tickstart) {
    return tickstop - tickstart >= time;
  } else {
    return UINT32_MAX - tickstart + tickstop >= time;
  }
}
