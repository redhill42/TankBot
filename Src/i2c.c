#include "i2c.h"
#include "delay.h"

#define SDA_LOW()   (I2C_SDA_GPIO_Port->ODR &= ~I2C_SDA_Pin) // set SDA to low
#define SDA_OPEN()  (I2C_SDA_GPIO_Port->ODR |= I2C_SDA_Pin)  // set SDA to open-drain
#define SDA_READ()  (I2C_SDA_GPIO_Port->IDR &  I2C_SDA_Pin)  // read SDA

#define SCL_LOW()   (I2C_SCL_GPIO_Port->ODR &= ~I2C_SCL_Pin) // set SCL to low
#define SCL_OPEN()  (I2C_SCL_GPIO_Port->ODR |= I2C_SCL_Pin)  // set SCL to open-drain
#define SCL_READ()  (I2C_SCL_GPIO_Port->IDR &  I2C_SCL_Pin)  // read SCL

#define DELAY(t)  HAL_Delay_us(t)

///////////////////////////////////////////////////////////////////////////////
// Low level I2C wire protocol
///////////////////////////////////////////////////////////////////////////////

// SDA and SCL pins must be configured as Output Open Drain

/**
 * Send start signal.
 */
void I2C_Start(void) {
  SDA_OPEN();
  DELAY(1);
  SCL_OPEN();
  DELAY(1);
  SDA_LOW();
  DELAY(10);  // hold time start condition (t_HD;STA)
  SCL_LOW();
  DELAY(10);
}

/**
 * Send stop signal.
 */
void I2C_Stop(void) {
  SCL_LOW();
  DELAY(1);
  SDA_LOW();
  DELAY(1);
  SCL_OPEN();
  DELAY(10); // set-up time stop condition (t_SU;STO)
  SDA_OPEN();
  DELAY(10);
}

bool I2C_SendByte(uint8_t data) {
  bool retval = true;

  for (uint8_t bit=0x80; bit>0; bit>>=1) { // shift bit for masking (8 times)
    if (data & bit) SDA_OPEN(); else SDA_LOW();
    DELAY(1);       // data set-up time (t_SU;DAT)
    SCL_OPEN();     // generate clock pulse on SCL
    DELAY(5);       // SCL high time (t_HIGH)
    SCL_LOW();
    DELAY(1);       // data hold time (t_HD;DAT)
  }
  
  SDA_OPEN();       // release SDA-line
  SCL_OPEN();       // clk #9 for ack
  DELAY(1);         // data set-up time (t_SU;DAT)
  if (SDA_READ())   // check ack from i2c slave
    retval = false;
  SCL_LOW();
  DELAY(20);        // wait to see byte package on scope
  
  return retval;
}

static bool WaitWhileClockStreching(uint8_t timeout) {
  while (SCL_READ() == 0) {
    if (--timeout == 0)
      return false;
    DELAY(1000);
  }
  return true;
}

bool I2C_ReadByte(uint8_t* data, bool ack, uint8_t timeout) {
  bool retval = true;
  
  *data = 0;
  SDA_OPEN();       // release SDA-line
  
  for (uint8_t bit=0x80; bit>0; bit>>=1) { // shift bit for masking (8 times)
    SCL_OPEN();     // start clock on SCL-line
    DELAY(1);       // clock set-up time (t_SU;CLK)
    retval = WaitWhileClockStreching(timeout);
    DELAY(3);       // SCL high time (t_HIGH)
    if (SDA_READ()) // read bit
      *data |= bit;
    SCL_LOW();
    DELAY(1);       // data hold time (t_HD;DAT)
  }
  
  if (ack) SDA_LOW(); // send acknowledge if necessary
  else     SDA_OPEN();  
  DELAY(1);         // data set-up time (t_SU;DAT)
  SCL_OPEN();       // clk #9 for ack
  DELAY(5);         // SCL high time (t_HIGH)
  SCL_LOW();
  SDA_OPEN();       // release SDA-line
  DELAY(20);        // wait to see byte package on scope
  
  return retval;
}

///////////////////////////////////////////////////////////////////////////////
// High level I2C protocol
///////////////////////////////////////////////////////////////////////////////

extern osMutexId i2c_mutexHandle;

#define SEND(data)  do {\
                      if (!I2C_SendByte(data)) {\
                        osMutexRelease(i2c_mutexHandle);\
                        return false;\
                      }\
                    } while(0)

#define RECEIVE(data,ack) \
                    do {\
                      if (!I2C_ReadByte(&data, ack, 250)) {\
                        osMutexRelease(i2c_mutexHandle);\
                        return false;\
                      }\
                    } while(0)
                    
#define BEGIN(addr) do {\
                      if (osMutexWait(i2c_mutexHandle, 500) != osOK)\
                        return false;\
                      I2C_Start();\
                      SEND(addr);\
                    } while(0)

#define END()       do {\
                      I2C_Stop();\
                      osMutexRelease(i2c_mutexHandle);\
                    } while(0)

bool I2C_Transmit(uint8_t dev_addr, uint8_t* data, uint16_t size) {
  BEGIN(dev_addr);
  for (int i=0; i<size; i++)
    SEND(data[i]);
  END();
  return true;
}

bool I2C_Receive(uint8_t dev_addr, uint8_t* data, uint16_t size) {
  BEGIN(dev_addr);
  for (int i=0; i<size; i++)
    RECEIVE(data[i], i != size-1);
  END();
  return true;
}

bool I2C_WriteMem(uint8_t dev_addr, uint16_t mem_addr, uint16_t mem_size, uint8_t* data, uint16_t size) {
  BEGIN(dev_addr);
  if (mem_size == 2)
    SEND((mem_addr>>8) & 0xFF);
  SEND(mem_addr & 0xFF);
  for (int i=0; i<size; i++)
    SEND(data[i]);
  END();
  return true;
}

bool I2C_ReadMem(uint8_t dev_addr, uint16_t mem_addr, uint16_t mem_size, uint8_t* data, uint16_t size) {
  BEGIN(dev_addr);
  if (mem_size == 2)
    SEND((mem_addr>>8) & 0xFF);
  SEND(mem_addr & 0xFF);
  I2C_Start();
  SEND(dev_addr|1);
  for (int i=0; i<size; i++)
    RECEIVE(data[i], i != size-1);
  END();
  return true;
}
