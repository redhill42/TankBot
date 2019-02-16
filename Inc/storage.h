#ifndef __STORAGE_H
#define __STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

HAL_StatusTypeDef read_store(uint16_t address, uint8_t* buff, uint16_t size);
HAL_StatusTypeDef write_store(uint16_t address, uint8_t* buff, uint16_t size, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif
