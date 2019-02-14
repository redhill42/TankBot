#include "main.h"
#include "cmsis_os.h"
#include "ledmatrix.h"
#include "motor.h"

extern osTimerId displayHandle;

void display_init(void) {
  led_set_brightness(10);
  osTimerStart(displayHandle, 100);
}

static const uint8_t up_arrow[] = {
  0x08, 0x1C, 0x3E, 0x7F, 0x1C, 0x1C, 0x1C, 0x00
};

static const uint8_t down_arrow[] = {
  0x1C, 0x1C, 0x1C, 0x7F, 0x3E, 0x1C, 0x08, 0x00
};

static void display_matrix(const uint8_t* matrix, uint32_t color, uint8_t frame) {
  for (int i=0; i<8; i++) {
    uint8_t line = matrix[(frame+i)%8];
    for (int j=0; j<8; j++) {
      if (line & 0x80) {
        led_set_color(i*8+j, color);
      } else {
        led_set_color(i*8+j, 0);
      }
      line <<= 1;
    }
  }
  led_update(1);
}

typedef enum {
  STOP,
  FORWARD,
  BACKWARD,
} BotState;

static BotState get_current_state(void) {
  int16_t m1spd, m2spd;
  
  get_motor_state(&m1spd, &m2spd);
  if (m1spd>0 && m2spd>0)
    return FORWARD;
  if (m1spd<0 && m2spd<0)
    return BACKWARD;
  return STOP;
}

static BotState current_state = STOP;
static const uint8_t* matrix;
static uint32_t color;
static int8_t frame, dir;

void display_task(void const* args) {
  BotState state = get_current_state();
  
  if (state != current_state) {
    current_state = state;
    
    switch (state) {
    case FORWARD:
      matrix = up_arrow;
      color  = 0x00FF00;
      frame  = 0;
      dir    = 1;
      break;
    
    case BACKWARD:
      matrix = down_arrow;
      color  = 0xFF0000;
      frame  = 0;
      dir    = -1;
      break;
    
    default:
      matrix = NULL;
      led_fill(0);
      led_update(1);
      break;
    }
  }
    
  if (matrix != NULL) {
    display_matrix(matrix, color, frame);
    frame = (frame+dir+8)%8;
  }
}
