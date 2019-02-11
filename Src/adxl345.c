#include "main.h"
#include "adxl345.h"
#include "delay.h"

///////////////////////////////////////////////////////////////////////////////
// Software I2C protocol
///////////////////////////////////////////////////////////////////////////////

#define SDA_H   I2C_SDA_GPIO_Port->BSRR=I2C_SDA_Pin
#define SDA_L   I2C_SDA_GPIO_Port->BRR=I2C_SDA_Pin

#define DATA    ((I2C_SDA_GPIO_Port->IDR&I2C_SDA_Pin)!=(uint32_t)GPIO_PIN_RESET)

#define SCL_H   I2C_SCL_GPIO_Port->BSRR=I2C_SCL_Pin
#define SCL_L   I2C_SCL_GPIO_Port->BRR=I2C_SCL_Pin

#define DELAY   HAL_Delay_us(5)

/**
 * Config I2C SDA pin as output mode.
 */
static void SDA_Out(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  GPIO_InitStruct.Pin = I2C_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(I2C_SDA_GPIO_Port, &GPIO_InitStruct);
}

/**
 * Config I2C SDA pin as input mode.
 */
static void SDA_In(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  GPIO_InitStruct.Pin = I2C_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(I2C_SDA_GPIO_Port, &GPIO_InitStruct);
}

/**
 * Send start signal.
 */
static void I2C_Start(void) {
  SDA_Out();
  SDA_H;
  SCL_H;
  DELAY;
  SDA_L;
  DELAY;
  SCL_L;
}

/**
 * Send stop signal.
 */
static void I2C_Stop(void) {
  SDA_Out();
  SCL_L;
  SDA_L;
  DELAY;
  SCL_H;
  SDA_H;
  DELAY;
}

/**
 * Send ACK signal to slave.
 */
static void I2C_Ack(void) {
  SCL_L;
  SDA_Out();
  SDA_L;
  DELAY;
  SCL_H;
  DELAY;
  SCL_L;
}

/**
 * Send NACK signal to slave.
 */
static void I2C_NoAck(void) {
  SCL_L;
  SDA_Out();
  SDA_H;
  DELAY;
  SCL_H;
  DELAY;
  SCL_L;
}

/**
 * Wait ACK signal from slave.
 */
static uint8_t I2C_WaitAck(void) {
  uint8_t time = 0;
  
  SDA_In();
  SDA_H;
  DELAY;
  SCL_H;
  DELAY;
  
  while (DATA) {
    time++;
    if (time > 250) {
      I2C_Stop();
      return 0;
    }
  }
  
  SCL_L;
  return 1;
}

static void I2C_SendByte(uint8_t data) {
  SDA_Out();
  SCL_L;
  
  for (int i=0; i<8; i++) {
    if (data&0x80) {
      SDA_H;
    } else {
      SDA_L;
    }
    data <<= 1;
    
    SCL_H;
    DELAY;
    SCL_L;
    DELAY;
  }
}

static uint8_t I2C_ReadByte(uint8_t ack) {
  uint8_t data = 0;
  
  SDA_In();
  for (int i=0; i<8; i++) {
    SCL_L;
    DELAY;
    SCL_H;
    data <<= 1;
    if (DATA)
      data |= 1;
    DELAY;
  }
  
  if (ack) {
    I2C_Ack();
  } else {
    I2C_NoAck();
  }
  
  return data;
}

///////////////////////////////////////////////////////////////////////////////
// ADXL345 driver
///////////////////////////////////////////////////////////////////////////////

#define DEV_ADDR 0xA6

static void adxl345_WriteReg(uint8_t addr, uint8_t val) {
  I2C_Start();
  I2C_SendByte(DEV_ADDR);
  I2C_WaitAck();
  I2C_SendByte(addr);
  I2C_WaitAck();
  I2C_SendByte(val);
  I2C_WaitAck();
  I2C_Stop();
}

static uint8_t adxl345_ReadReg(uint8_t addr) {
  uint8_t val = 0;
  
  I2C_Start();
  I2C_SendByte(DEV_ADDR);
  I2C_WaitAck();
  I2C_SendByte(addr);
  I2C_WaitAck();
  
  I2C_Start();
  I2C_SendByte(DEV_ADDR|1);
  I2C_WaitAck();
  val = I2C_ReadByte(0);
  I2C_Stop();
  
  return val;
}

void adxl345_init(void) {
  adxl345_WriteReg(0x31, 0x09);
  adxl345_WriteReg(0x2C, 0x0A);
  adxl345_WriteReg(0x2D, 0x08);
  adxl345_WriteReg(0x2E, 0x80);
  adxl345_WriteReg(0x1E, 0x00);
  adxl345_WriteReg(0x1F, 0x00);
  adxl345_WriteReg(0x20, 0x05);
}

uint8_t adxl345_id(void) {
  return adxl345_ReadReg(0x00);
}

void adxl345_read(short* x, short* y, short* z) {
  uint8_t buf[6];
  
  I2C_Start();
  I2C_SendByte(DEV_ADDR);
  I2C_WaitAck();
  I2C_SendByte(0x32);
  I2C_WaitAck();
  
  I2C_Start();
  I2C_SendByte(DEV_ADDR|1);
  I2C_WaitAck();
  for (int i=0; i<6; i++)
    buf[i] = I2C_ReadByte(i!=5);
  I2C_Stop();
  
  *x = (short)(((uint16_t)buf[1]<<8)|buf[0]);
  *y = (short)(((uint16_t)buf[3]<<8)|buf[2]);
  *z = (short)(((uint16_t)buf[5]<<8)|buf[4]);
}
