#ifndef __BEEP_H
#define __BEEP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

void beep_start(void);
void beep_stop(void);

#ifdef __cplusplus
}
#endif

#endif
