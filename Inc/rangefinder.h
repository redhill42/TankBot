#ifndef __RANGEFINDER_H
#define __RANGEFINDER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define INVALID_DISTANCE UINT16_MAX

void rangefinder_init(void);
uint16_t get_distance(void);

#ifdef __cplusplus
}
#endif

#endif
