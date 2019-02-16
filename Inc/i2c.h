#ifndef __I2C_H
#define __I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

///////////////////////////////////////////////////////////////////////////////
// Low level interface

/**
 * @brief Writes a start condition on I2C-Bus.
 * @note: Timing (delay) may have to be changed for different microcontroller.
 */
void I2C_Start(void);

/**
 * @brief Writes a stop condition on I2C-Bus.
 * @note: Timing (delay) may have to be channged for different microcontroller.
 */
void I2C_Stop(void);

/**
 * @brief Writes a byte to I2C-Bus and checks acknowledge.
 * @param data: data to transmit
 * @retval true:  no acknowledgement from slave
 *         false: no error
 */
bool I2C_SendByte(uint8_t data);

/**
 * @brief Reads a byte from I2C-Bus.
 */
bool I2C_ReadByte(uint8_t* data, bool ack, uint8_t timeout);

///////////////////////////////////////////////////////////////////////////////
// High level interface

bool I2C_Transmit(uint8_t dev_addr, uint8_t* data, uint16_t size);
bool I2C_Receive(uint8_t dev_addr, uint8_t* data, uint16_t size);
bool I2C_WriteMem(uint8_t dev_addr, uint16_t mem_addr, uint16_t mem_size, uint8_t* data, uint16_t size);
bool I2C_ReadMem(uint8_t dev_addr, uint16_t mem_addr, uint16_t mem_size, uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
