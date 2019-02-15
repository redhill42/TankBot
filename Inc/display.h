#ifndef __DISPLAY_H
#define __DISPLAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void display_init(void);
void display_message(const char* message, uint32_t color);

#ifdef __cplusplus
}
#endif

#endif
