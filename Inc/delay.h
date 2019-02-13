#ifndef __DELAY_H
#define __DELAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef uint32_t stopwatch_t;

void     HAL_Delay_us(uint32_t time);
void     SW_Start(stopwatch_t* sw);
uint32_t SW_Stop(stopwatch_t* sw);
void     SW_Reset(stopwatch_t* sw);
bool     SW_Elapsed(stopwatch_t* sw, uint32_t time);

#ifdef __cplusplus
}
#endif

#endif
