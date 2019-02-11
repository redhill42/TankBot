#ifndef __ADXL345_H
#define __ADXL345_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void adxl345_init(void);
uint8_t adxl345_id(void);
void adxl345_read(short* x, short* y, short* z);

#ifdef __cplusplus
}
#endif

#endif
