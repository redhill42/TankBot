#include "main.h"
#include "cmsis_os.h"
#include "beep.h"

static __IO uint8_t beep_started;
extern osTimerId beep_timerHandle;

void beep_start(void) {
  if (!beep_started) {
    beep_started = 1;
    osTimerStart(beep_timerHandle, 1);
  }
}

void beep_stop(void) {
  if (beep_started) {
    beep_started = 0;
    osTimerStop(beep_timerHandle);
  }
}

void beep_task(void const* args) {
  HAL_GPIO_TogglePin(BEEP_GPIO_Port, BEEP_Pin);
}
