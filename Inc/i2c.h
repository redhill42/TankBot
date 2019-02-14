#ifndef __I2C_H
#define __I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void    I2C_Start(void);
void    I2C_Stop(void);
void    I2C_Ack(void);
void    I2C_NoAck(void);
bool    I2C_WaitAck(void);
void    I2C_SendByte(uint8_t data);
uint8_t I2C_ReadByte(uint8_t ack);

bool    I2C_Transmit(uint8_t dev_addr, uint8_t* data, uint16_t size);
bool    I2C_Receive(uint8_t dev_addr, uint8_t* data, uint16_t size);
bool    I2C_WriteMem(uint8_t dev_addr, uint16_t mem_addr, uint16_t mem_size, uint8_t* data, uint16_t size);
bool    I2C_ReadMem(uint8_t dev_addr, uint16_t mem_addr, uint16_t mem_size, uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
