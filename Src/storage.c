#include "main.h"
#include "storage.h"
#include "i2c.h"
#include "delay.h"

#define EE_ADDRESS  0xA0
#define PAGE_SIZE   32

HAL_StatusTypeDef read_store(uint16_t address, uint8_t* buff, uint16_t size) {
  if (I2C_ReadMem(EE_ADDRESS, address, 2, buff, size))
    return HAL_OK;
  return HAL_ERROR;
}

static HAL_StatusTypeDef write_page(uint16_t address, uint8_t* buff, uint16_t size, stopwatch_t* sw, uint32_t timeout) {
  if (!I2C_WriteMem(EE_ADDRESS, address, 2, buff, size))
    return HAL_ERROR;
  
  // wait for write complete
  while (!I2C_Transmit(EE_ADDRESS, NULL, 0)) {
    if (SW_Elapsed(sw, timeout)) {
      return HAL_TIMEOUT;
    }
  }
  
  return HAL_OK;
}

HAL_StatusTypeDef write_store(uint16_t address, uint8_t* buff, uint16_t count, uint32_t timeout) {
  HAL_StatusTypeDef status;
  stopwatch_t sw;
  uint32_t rem;
  
  SW_Reset(&sw);
  
  rem = PAGE_SIZE - (address % PAGE_SIZE);
  if (rem > count)
    rem = count;
  if (rem != 0) {
    status = write_page(address, buff, rem, &sw, timeout);
    if (status != HAL_OK)
      return status;
    address += rem;
    buff    += rem;
    count   -= rem;
  }
  
  while (count >= PAGE_SIZE) {
    status = write_page(address, buff, PAGE_SIZE, &sw, timeout);
    if (status != HAL_OK)
      return status;
    address += PAGE_SIZE;
    buff    += PAGE_SIZE;
    count   -= PAGE_SIZE;
  }
  
  if (count > 0) {
    status = write_page(address, buff, count, &sw, timeout);
    if (status != HAL_OK)
      return status;
  }
  
  return HAL_OK;
}
