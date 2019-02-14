#include <string.h>
#include "main.h"
#include "cmsis_os.h"
#include "ledmatrix.h"

#define LED_CFG_BYTES_PER_LED       3
#define LED_CFG_RAW_BYTES_PER_LED   (LED_CFG_BYTES_PER_LED*8)

typedef struct {
  uint8_t R, G, B;
} PIXEL;

/**
 * @brief Array of 3x (or 4x) number of leds (R, G, B[, W] colors)
 */
static PIXEL led_pixels[LED_CFG_LEDS_CNT];

/**
 * @brief The brightness of LEDs
 */
static uint8_t brightness = 255;

/**
 * @brief  Temporary array for dual LED with extracted PWM duty cycles
 *
 * We need LED_CFG_RAW_BYTES_PER_LED bytes for PWM setup to send all bits.
 * Before we can send data for first led, we have to send reset pulse,
 * which must be 50us long.
 *
 * PWM frequency is 800kHz, to archieve 50us, we need to send 40 pulses with
 * 0 duty cycle.
 */
static uint32_t led_pwm_data[2*LED_CFG_RAW_BYTES_PER_LED];

static __IO bool is_updating;     /* Is updating in progress? */
static uint8_t   is_reset_pulse;  /* Status if we are sending reset pulse */
static uint32_t  current_led;     /* Current LED number we are sending */

static void led_start_reset_pulse(uint8_t num);
static bool led_fill_led_pwm_data(int index, uint32_t* ptr);
static void led_half_cplt_callback(DMA_HandleTypeDef* hdma);

/**
 * @brief  Set R,G,B color for specific LED
 * @param  index: LED index in array, starting from '0'
 * @param  r,g,b: Red, Green, Blue values
 * @return 1 on success, 0 otherwise
 */
bool led_set_color(int index, uint32_t color) {
  if (index < LED_CFG_LEDS_CNT) {
    uint8_t r = (color>>16) & 0xFF;
    uint8_t g = (color>> 8) & 0xFF;
    uint8_t b =  color      & 0xFF;
    
    led_pixels[index].R = (r*brightness)>>8;
    led_pixels[index].G = (g*brightness)>>8;
    led_pixels[index].B = (b*brightness)>>8;
    return true;
  }
  return false;
}

bool led_fill(uint32_t color) {
  uint8_t r = (color>>16) & 0xFF;
  uint8_t g = (color>> 8) & 0xFF;
  uint8_t b =  color      & 0xFF;
  
  for (int index=0; index<LED_CFG_LEDS_CNT; index++) {
    led_pixels[index].R = (r*brightness)>>8;
    led_pixels[index].G = (g*brightness)>>8;
    led_pixels[index].B = (b*brightness)>>8;
  }
  return true;
}

/**
 * Adjust output brightness; 0=darkest (0ff), 255=brightst. This function must be
 * called before setting pixel colors to take effect.
 */
void led_set_brightness(uint8_t b) {
  brightness = b;
}

/**
 * @brief  Start LEDs update procedure
 * @param  block: Set to 1 to block for update process until finished
 * @return 1 if update started, 0 otherwise
 */
bool led_update(bool block) {
  if (is_updating) {
    return false;
  }
  
  is_updating = true;
  led_start_reset_pulse(1);
  
  if (block) {
    while (is_updating);
  }
  
  return true;
}

/**
 * @brief  Start reset pulse sequence
 * @param  num: Number indicating pulse is for beginning (1) or end (2) of PWM data stream
 */
static void led_start_reset_pulse(uint8_t num) {
  is_reset_pulse = num;
  
  // Set all bytes to 0 to achieve 50us pulse
  memset(led_pwm_data, 0, sizeof(led_pwm_data));
  
  if (num == 1) {
    led_pwm_data[0] = __HAL_TIM_GET_AUTORELOAD(LED_TIM) / 2;
  }
  
  // Set DMA to normal mode
  LED_DMA->Instance->CCR &= ~DMA_CIRCULAR;
  LED_DMA->XferHalfCpltCallback = NULL;
  HAL_TIM_PWM_Start_DMA(LED_TIM, LED_TIM_CHANNEL, (uint32_t*)led_pwm_data, 40);
}

/**
 * @brief  Update sequence function, called on each DMA transfer complete
 *         or half-transfer complete events
 * @param  tc: Transfer complete flag. Set to 1 on TC event, or 0 on HT event
 * @note   TC = Transfer-Complete event, called at the end of  block
 * @note   HT = Half-Transfer-Complete event, called in the middle of elements
 *         transfered by DMA. 
 *         If block is 48 elements (our case),
 *            HT is called when first LED_CFG_RAW_BYTES_PER_LED elements are transfered
 *            TC is called when second LED_CFG_RAW_BYTES_PER_LED elements are transfered
 * @note   This function must be called from DMA interrupt
 */
static void led_update_sequence(bool tc) {
  // Check for reset pulse at the end of PWM stream
  if (is_reset_pulse == 2) {
    HAL_TIM_PWM_Stop_DMA(LED_TIM, LED_TIM_CHANNEL); // manually stop DMA
    is_updating = false;
    return;
  }
  
  // Check for reset pulse on beginning of PWM stream
  if (is_reset_pulse == 1) {
    // When reset pulse is active, we have to wait full DMA response,
    // before we can start modifying array which is shared with DMA and PWM
    if (!tc) {
      return;
    }
    
    // Disable timer output and disable DMA stream
    HAL_TIM_PWM_Stop_DMA(LED_TIM, LED_TIM_CHANNEL);
    
    is_reset_pulse = 0;
    current_led = 0;
  } else {
    // When we are not in reset mode, go to next led and process data for it
    current_led++;
  }
  
  // This part is used to prepare data for next led, for which update will
  // start once current transfer stops in circular mode
  if (current_led < LED_CFG_LEDS_CNT) {
    // If we are preparing data for first time (current_led == 0)
    // or if there was no TC event (it was HT):
    //
    //  - Prepare first part of array, because either there is no transfer
    //    or second part (from HT to TC) is now in process for PWM transfer
    if (current_led == 0 || !tc) {
      led_fill_led_pwm_data(current_led, &led_pwm_data[0]);
    } else {
      led_fill_led_pwm_data(current_led, &led_pwm_data[LED_CFG_RAW_BYTES_PER_LED]);
    }
    
    // If we are preparing first led (current_led = 0), then:
    //
    //  - We setup first part of array for first led,
    //  - We have to prepare second part for second led to have one led prepared
    //  - Set DMA to circular mode and start the transfer + PWM output
    if (current_led == 0) {
      current_led++;
      led_fill_led_pwm_data(current_led, &led_pwm_data[LED_CFG_RAW_BYTES_PER_LED]);
      
      LED_DMA->Instance->CCR |= DMA_CIRCULAR;
      LED_DMA->XferHalfCpltCallback = led_half_cplt_callback;
      HAL_TIM_PWM_Start_DMA(LED_TIM, LED_TIM_CHANNEL, (uint32_t*)led_pwm_data, 2*LED_CFG_RAW_BYTES_PER_LED);
    }
    
  // When we reached all leds, we have to wait to transmit data for all leds
  // before we can disable DMA and PWM:
  //
  //  - If TC event is enabled and we have EVEN number of LEDS (2,4,6,...)
  //  - If HT event is enabled and we have ODD number of LEDS (1,3,5,...)
  } else if ((!tc & (LED_CFG_LEDS_CNT & 0x01)) || (tc && !(LED_CFG_LEDS_CNT & 0x01))) {
    HAL_TIM_PWM_Stop_DMA(LED_TIM, LED_TIM_CHANNEL);
    
    // It is time to send final reset pulse, 50us at least
    led_start_reset_pulse(2);
  }
}

/**
 * @brief  Prepares data from memory for PWM output for timer
 * @note   Memory is in format R,G,B, while PWM must be configured in G,R,B[,W]
 * @param  index: LED index to set the color
 * @param  ptr: Output array with at least LED_CFG_RAW_BYTES_PER_LED-words of memory
 */
static bool led_fill_led_pwm_data(int index, uint32_t* ptr) {
  uint32_t duty_0 = (LED_TIM->Instance->ARR+1) / 3;
  uint32_t duty_1 = 2 * (LED_TIM->Instance->ARR+1) / 3;
  int i;
  
  if (index < LED_CFG_LEDS_CNT) {
    for (i=0; i<8; i++) {
      ptr[i   ] = (led_pixels[index].G & (1<<(7-i))) ? duty_1 : duty_0;
      ptr[i+ 8] = (led_pixels[index].R & (1<<(7-i))) ? duty_1 : duty_0;
      ptr[i+16] = (led_pixels[index].B & (1<<(7-i))) ? duty_1 : duty_0;
    }
    return true;
  }
  return false;
}

static void led_half_cplt_callback(DMA_HandleTypeDef* hdma) {
  led_update_sequence(false);
}

void led_pwm_pulse(void) {
  led_update_sequence(true);
}
