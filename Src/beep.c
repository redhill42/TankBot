#include "main.h"
#include "cmsis_os.h"
#include "beep.h"

extern TIM_HandleTypeDef htim3;

#define BEEP_TIM (&htim3)
#define BEEP_TIM_CHANNEL TIM_CHANNEL_2

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
