#include "i2c.h"
#include "delay.h"

#define SDA_H     I2C_SDA_GPIO_Port->BSRR=I2C_SDA_Pin
#define SDA_L     I2C_SDA_GPIO_Port->BRR=I2C_SDA_Pin

#define SDA_READ  ((I2C_SDA_GPIO_Port->IDR&I2C_SDA_Pin)!=0)

#define SCL_H     I2C_SCL_GPIO_Port->BSRR=I2C_SCL_Pin
#define SCL_L     I2C_SCL_GPIO_Port->BRR=I2C_SCL_Pin

#define DELAY(t)  HAL_Delay_us(t)

///////////////////////////////////////////////////////////////////////////////
// Low level I2C wire protocol
///////////////////////////////////////////////////////////////////////////////

// SDA and SCL pins must be configured as Output Open Drain

/**
 * Send start signal.
 */
void I2C_Start(void) {
  SDA_H;
  SCL_H;
  DELAY(1);
  SDA_L;
  DELAY(1);
  SCL_L;
  DELAY(2);
}

/**
 * Send stop signal.
 */
void I2C_Stop(void) {
  SCL_L;
  SDA_L;
  DELAY(1);
  SCL_H;
  DELAY(1);
  SDA_H;
  DELAY(2);
}

void I2C_SendByte(uint8_t data) {
  SCL_L;            // hold on SCL-line
  for (uint8_t bit=0x80; bit>0; bit>>=1) {
    if (data & bit) // write bit to SDA-line
      SDA_H;
    else
      SDA_L;
    
    SCL_H;          // generate clock pulse on SCL
    DELAY(1);
    SCL_L;
    DELAY(2);
  }
}

/**
 * Wait ACK signal from slave.
 */
bool I2C_WaitAck(void) {
  uint8_t time = 0;
  
  SDA_H;          // release SDA-line
  SCL_H;          // generate clock pulse on SCL
  DELAY(1);
  
  // check ACK from I2C slave
  while (SDA_READ) {
    if (++time > 250) {
      I2C_Stop();
      return false;
    }
  }
  
  SCL_L;
  DELAY(2);
  return true;
}

uint8_t I2C_ReadByte(uint8_t ack) {
  uint8_t data = 0;
  
  SDA_H;            // release SDA-line for read
  
  for (uint8_t bit=0x80; bit>0; bit>>=1) {
    SCL_H;          // generate SCL clock pulse
    DELAY(1);
    if (SDA_READ)   // read bit
      data |= bit;
    SCL_L;
    DELAY(2);
  }
  
  if (ack) SDA_L;   // send acknowledge if necessary
  else     SDA_H;

  SCL_H;            // generate SCL clock pulse
  DELAY(1);
  SCL_L;
  DELAY(2);
  
  return data;
}

///////////////////////////////////////////////////////////////////////////////
// High level I2C protocol
///////////////////////////////////////////////////////////////////////////////

extern osMutexId i2c_mutexHandle;

#define WAIT_ACK()  do {\
                      if (!I2C_WaitAck()) {\
                        osMutexRelease(i2c_mutexHandle);\
                        return false;\
                      }\
                    } while(0)

#define BEGIN(addr) do {\
                      if (osMutexWait(i2c_mutexHandle, 500) != osOK)\
                        return false;\
                      I2C_Start();\
                      I2C_SendByte(addr);\
                      WAIT_ACK();\
                    } while(0)

#define END()       do {\
                      I2C_Stop();\
                      osMutexRelease(i2c_mutexHandle);\
                    } while(0)

bool I2C_Transmit(uint8_t dev_addr, uint8_t* data, uint16_t size) {
  BEGIN(dev_addr);
  for (int i=0; i<size; i++) {
    I2C_SendByte(data[i]);
    WAIT_ACK();
  }
  END();
  return true;
}

bool I2C_Receive(uint8_t dev_addr, uint8_t* data, uint16_t size) {
  BEGIN(dev_addr);
  for (int i=0; i<size; i++) {
    data[i] = I2C_ReadByte(i != size-1);
  }
  END();
  return true;
}

bool I2C_WriteMem(uint8_t dev_addr, uint16_t mem_addr, uint16_t mem_size, uint8_t* data, uint16_t size) {
  BEGIN(dev_addr);
  
  if (mem_size == 2) {
    I2C_SendByte((mem_addr>>8)&0xFF);
    WAIT_ACK();
  }
  
  I2C_SendByte(mem_addr&0xFF);
  WAIT_ACK();
  
  for (int i=0; i<size; i++) {
    I2C_SendByte(data[i]);
    WAIT_ACK();
  }
  
  END();
  return true;
}

bool I2C_ReadMem(uint8_t dev_addr, uint16_t mem_addr, uint16_t mem_size, uint8_t* data, uint16_t size) {
  BEGIN(dev_addr);
  
  if (mem_size == 2) {
    I2C_SendByte((mem_addr>>8)&0xFF);
    WAIT_ACK();
  }
  
  I2C_SendByte(mem_addr&0xFF);
  WAIT_ACK();
  
  I2C_Start();
  I2C_SendByte(dev_addr|1);
  WAIT_ACK();
  
  for (int i=0; i<size; i++) {
    data[i] = I2C_ReadByte(i != size-1);
  }

  END();
  return true;
}
