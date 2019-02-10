#ifndef __PS2CONTROLLER_H
#define __PS2CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// The button constants
#define PSB_SELECT      0x0001
#define PSB_L3          0x0002
#define PSB_R3          0x0004
#define PSB_START       0x0008
#define PSB_PAD_UP      0x0010
#define PSB_PAD_RIGHT   0x0020
#define PSB_PAD_DOWN    0x0040
#define PSB_PAD_LEFT    0x0080
#define PSB_L2          0x0100
#define PSB_R2          0x0200
#define PSB_L1          0x0400
#define PSB_R1          0x0800
#define PSB_TRIANGLE    0x1000
#define PSB_CIRCLE      0x2000
#define PSB_CROSS       0x4000
#define PSB_SQUARE      0x8000

// The stick values
#define PSS_RX  5
#define PSS_RY  6
#define PSS_LX  7
#define PSS_LY  8

void ps2_init(void);
uint16_t ps2_get_key(uint16_t keep);
uint8_t ps2_get_stick(uint8_t stick);

#ifdef __cplusplus
}
#endif

#endif
