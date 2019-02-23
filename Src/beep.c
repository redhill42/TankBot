#include "main.h"
#include "cmsis_os.h"
#include "beep.h"

#define CLOCK_FREQ   72000000

extern TIM_HandleTypeDef  htim3;
#define BEEP_TIM          (&htim3)
#define BEEP_TIM_CHANNEL  TIM_CHANNEL_2

#define SPWM 0

#if SPWM

extern DMA_HandleTypeDef  hdma_tim3_ch4_up;
#define BEEP_DMA          (&hdma_tim3_ch4_up)

#define WAVE_LENGTH 256

static const uint8_t sine[WAVE_LENGTH] = {
  128, 131, 134, 137, 140, 144, 147, 150, 153, 156, 159, 162, 165, 168, 171, 174, 
  177, 179, 182, 185, 188, 191, 193, 196, 199, 201, 204, 206, 209, 211, 213, 216, 
  218, 220, 222, 224, 226, 228, 230, 232, 234, 235, 237, 239, 240, 241, 243, 244, 
  245, 246, 248, 249, 250, 250, 251, 252, 253, 253, 254, 254, 254, 255, 255, 255, 
  255, 255, 255, 255, 254, 254, 254, 253, 253, 252, 251, 250, 250, 249, 248, 246, 
  245, 244, 243, 241, 240, 239, 237, 235, 234, 232, 230, 228, 226, 224, 222, 220, 
  218, 216, 213, 211, 209, 206, 204, 201, 199, 196, 193, 191, 188, 185, 182, 179, 
  177, 174, 171, 168, 165, 162, 159, 156, 153, 150, 147, 144, 140, 137, 134, 131, 
  128, 125, 122, 119, 116, 112, 109, 106, 103, 100,  97,  94,  91,  88,  85,  82,
   79,  77,  74,  71,  68,  65,  63,  60,  57,  55,  52,  50,  47,  45,  43,  40,
   38,  36,  34,  32,  30,  28,  26,  24,  22,  21,  19,  17,  16,  15,  13,  12,
   11,  10,   8,   7,   6,   6,   5,   4,   3,   3,   2,   2,   2,   1,   1,   1,
    1,   1,   1,   1,   2,   2,   2,   3,   3,   4,   5,   6,   6,   7,   8,  10,
   11,  12,  13,  15,  16,  17,  19,  21,  22,  24,  26,  28,  30,  32,  34,  36,
   38,  40,  43,  45,  47,  50,  52,  55,  57,  60,  63,  65,  68,  71,  74,  77,
   79,  82,  85,  88,  91,  94,  97, 100, 103, 106, 109, 112, 116, 119, 122, 125
};

static uint32_t wave[WAVE_LENGTH];

void tone(uint32_t frequency) {
  uint32_t period = CLOCK_FREQ / (frequency * WAVE_LENGTH);
  for (int i=0; i<WAVE_LENGTH; i++) {
    wave[i] = (uint32_t)sine[i] * period / 256;
  }
  
  __HAL_TIM_SET_AUTORELOAD(BEEP_TIM, period-1);
  __HAL_TIM_SET_COUNTER(BEEP_TIM, 0);
  
  HAL_DMA_Start(BEEP_DMA, (uint32_t)wave, (uint32_t)&BEEP_TIM->Instance->CCR2, WAVE_LENGTH);
  __HAL_TIM_ENABLE_DMA(BEEP_TIM, TIM_DMA_UPDATE);
  __HAL_TIM_ENABLE(BEEP_TIM);
  HAL_TIM_PWM_Start(BEEP_TIM, BEEP_TIM_CHANNEL);
}

void no_tone(void) {
  __HAL_TIM_DISABLE_DMA(BEEP_TIM, TIM_DMA_UPDATE);
  __HAL_TIM_DISABLE(BEEP_TIM);
  HAL_TIM_PWM_Stop(BEEP_TIM, BEEP_TIM_CHANNEL);
}

#else

void tone(uint32_t frequency) {
  uint32_t period = 1000000/frequency;
  
  __HAL_TIM_SET_COUNTER(BEEP_TIM, 0);
  __HAL_TIM_SET_AUTORELOAD(BEEP_TIM, period-1);
  __HAL_TIM_SET_COMPARE(BEEP_TIM, BEEP_TIM_CHANNEL, period/2);
  
  HAL_TIM_PWM_Start(BEEP_TIM, BEEP_TIM_CHANNEL);
}

void no_tone(void) {
  HAL_TIM_PWM_Stop(BEEP_TIM, BEEP_TIM_CHANNEL);
}

#endif

///////////////////////////////////////////////////////////////////////////////

static const char* tone_notes;
static const char* tone_note_ptr;
static bool flash_led;

static uint8_t  base_octave = 4;  // note octave (initialized at octave 4)
static uint8_t  base_meter = 4;   // note meter (initialized at a quarter)
static uint16_t base_duration = 1500; // note whole duration

// Array of frequencies of music notes at the 7th octave
// (frequency of 0 added for rests)
static const uint16_t frequencies[] = {3520, 3951, 2093, 2349, 2637, 2794, 3136, 0};

// Divisors used to produce music note frequencies at octaves 1 to 7
static const int divisors[] = {64, 32, 16, 8, 4, 2, 1};

extern osThreadId beep_taskHandle;
extern osMutexId beep_mutexHandle;

static bool decode(const char** pp, uint16_t* frequency, uint32_t* duration);

bool beep_start(const char* notes, bool flash) {
  if (osMutexWait(beep_mutexHandle, 500) != osOK) {
    return false;
  }
  
  tone_notes = notes;
  tone_note_ptr = notes;
  flash_led = flash;
  base_octave = 4;
  base_meter = 4;
  base_duration = 1500;
  
  osMutexRelease(beep_mutexHandle);
  osSignalSet(beep_taskHandle, 0x01);
  return true;
}

void beep_stop(void) {
  if (osMutexWait(beep_mutexHandle, 500) == osOK) {
    tone_notes = NULL;
    tone_note_ptr = NULL;
    osMutexRelease(beep_mutexHandle);
    osSignalSet(beep_taskHandle, 0x01);
  }
}

static void led_on(void) {
  if (flash_led) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  }
}

static void led_off(void) {
  // always turn off led
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void beep_daemon(void const* args) {
  uint32_t delay = osWaitForever;
  uint16_t freq;
  
  for (;;) {
    // Wait for external event. If no event happened, then wait for given time period.
    osSignalWait(0xFFFF, delay);
    delay = osWaitForever;
    
    // Turn off buzzer and LED
    no_tone();
    led_off();

    // Attempt to decode current note
    if (osMutexWait(beep_mutexHandle, 50) == osOK) {
      if (tone_note_ptr != NULL) {
        if (decode(&tone_note_ptr, &freq, &delay)) {
          if (freq != 0) {
            tone(freq);
            led_on();
          }
          if (delay == 0) {
            delay = osWaitForever;
          }
        }
      }
      osMutexRelease(beep_mutexHandle);
    }
  }
}

static bool decode(const char** pp, uint16_t* frequency, uint32_t* duration) {
  const char* p = *pp;
  
  while (*p) {
    uint8_t octave = base_octave;
    uint8_t note, sharp = 0;
    uint8_t meter = base_meter;
    int tmp = 0;
    
    // ignore whitespaces
    if (*p == ' ') {
      p++;
      continue;
    }
    
    // repeat current note
    if (*p == '@') {
      p = tone_notes;
      continue;
    }
    
    // set base octave
    if (*p == '=') {
      p++;
      if (*p>='1' && *p<='7') {
        base_octave = *p++ - '0';
      }
      continue;
    }

    // increment base octave
    if (*p == '+') {
      p++;
      if (base_octave < 7)
        base_octave++;
      continue;
    }
  
    // decrement base octave
    if (*p == '-') {
      p++;
      if (base_octave > 1)
        base_octave--;
      continue;
    }
  
    // set base duration
    if (*p == ':') {
      p++;
      while (*p>='0' && *p<='9') {
        tmp = tmp*10 + (*p++ - '0');
      }
      if (tmp>0 && tmp<=UINT16_MAX) {
        base_duration = tmp;
      }
      continue;
    }
  
    // set base meter
    if (*p == '/') {
      p++;
      while (*p>='0' && *p<='9') {
        tmp = tmp*10 + (*p++ - '0');
      }
      if (tmp>0 && tmp<64) {
        base_meter = tmp;
      }
      continue;
    }
    
    if (*p>='a' && *p<='g') {
      note = *p++ - 'a';
    } else if (*p>='A' && *p<='G') {
      octave++;
      note = *p++ - 'A';
    } else if (*p=='$') {
      p++;
      note = 7;
    } else {
      p++;
      continue;
    }
  
    while (*p=='+' || *p=='-') {
      if (*p=='+' && octave<7)
        octave++;
      if (*p=='-' && octave>1)
        octave--;
      p++;
    }
    if (*p == '#') {
      p++;
      sharp = 1;
    }

    if (*p>='0' && *p<='9') {
      meter = 0;
      while (*p>='0' && *p<='9') {
        meter = meter*10 + (*p++ - '0');
      }
    }
    
    *frequency = frequencies[note] / divisors[octave-1];
    if (sharp) {
      *frequency = *frequency * 1059 / 1000;
    }
    
    if (meter == 0) {
      *duration = 0;
    } else {
      *duration = base_duration / meter;
      if (*p == '.') {
        p++;
        *duration = *duration * 3/2;
      }
    }
    
    *pp = p;
    return true;
  }
  
  *pp = p;
  return false;
}
