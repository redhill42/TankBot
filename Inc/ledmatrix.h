#ifndef __LEDMATRIX_H
#define __LEDMATRIX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define LED_CFG_LEDS_CNT 64    /* Number of leds in a strip row  */

#define RGB(r,g,b) ((((r)&0xFF)<<16)|(((g)&0xFF)<<8)|((b)&0xFF))

void led_set_brightness(uint8_t brightness);
bool led_set_color(int index, uint32_t color);
bool led_fill(uint32_t color);
bool led_update(bool block);

// private

extern TIM_HandleTypeDef  htim4;
extern DMA_HandleTypeDef  hdma_tim4_ch3;

#define LED_TIM           (&htim4)
#define LED_TIM_CHANNEL   TIM_CHANNEL_3
#define LED_DMA           (&hdma_tim4_ch3)

void led_pwm_pulse(void);

#ifdef __cplusplus
}
#endif

#endif // __LEDMATRIX_H
