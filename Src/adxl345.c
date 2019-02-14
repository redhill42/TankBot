#include "main.h"
#include "adxl345.h"
#include "i2c.h"

#define DEV_ADDR 0xA6

static void adxl345_WriteReg(uint8_t addr, uint8_t val) {
  I2C_WriteMem(DEV_ADDR, addr, 1, &val, 1);
}

static uint8_t adxl345_ReadReg(uint8_t addr) {
  uint8_t val = 0;
  I2C_ReadMem(DEV_ADDR, addr, 1, &val, 1);
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
  if (adxl345_id() == 0345) {
    uint8_t buf[6];
    I2C_ReadMem(DEV_ADDR, 0x32, 1, buf, 6);
    *x = (short)(((uint16_t)buf[1]<<8)|buf[0]);
    *y = (short)(((uint16_t)buf[3]<<8)|buf[2]);
    *z = (short)(((uint16_t)buf[5]<<8)|buf[4]);
  } else {
    *x = *y = *z = 0;
  }
}
