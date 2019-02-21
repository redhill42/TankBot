#ifndef __BEEP_H
#define __BEEP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void tone(uint32_t frequency);
void no_tone(void);

bool beep_start(const char* notes, bool flash);
void beep_stop(void);

#ifdef __cplusplus
}
#endif

#endif
