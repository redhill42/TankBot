#include <string.h>
#include "main.h"
#include "cmsis_os.h"
#include "ledmatrix.h"
#include "motor.h"

#include "font.inc"

#define DISP_WIDTH    8
#define DISP_HEIGHT   8
#define FONT_WIDTH    8
#define FONT_HEIGHT   8
#define CHAR_SPACING  2
#define LINE_SPACING  2
#define CHAR_WIDTH    (FONT_WIDTH+CHAR_SPACING)
#define CHAR_HEIGHT   (FONT_HEIGHT+LINE_SPACING)

typedef uint32_t (*COLOR_FN)(int id, uint32_t color);

/** 
 * Simple text display routine. Handles text scrolling and spacing.
 */
void display_string_at(const char* str, int xpos, int ypos, uint32_t color, COLOR_FN color_fn) {
  int  len   = strlen(str);
  int  chpos = xpos / CHAR_WIDTH;
  char ch1   = str[chpos%len];
  char ch2   = str[(chpos+1)%len];
  int  xoff  = xpos % CHAR_WIDTH;
  
  for (int y=0; y<DISP_HEIGHT; y++) {
    int yoff = (y+ypos) % CHAR_HEIGHT;
    uint8_t line = 0;

    if (yoff < FONT_HEIGHT) {
      line = (font[(ch1-' ')*FONT_HEIGHT + yoff] << xoff)
           | (font[(ch2-' ')*FONT_HEIGHT + yoff] >> (CHAR_WIDTH-xoff));
    }
    
    for (int x=0; x<DISP_WIDTH; x++, line<<=1) {
      int id = y*DISP_WIDTH+x;
      if (line & 0x80) {
        led_set_color(id, color_fn(id, color));
      } else {
        led_set_color(id, 0);
      }
    }
  }
  
  led_update(true);
}

/* Input a value 0 to 255 to get a color value. The colours are
 * a transition r - g - b - back to r.
 */
static uint32_t wheel(uint8_t wheelPos) {
  wheelPos = 255 - wheelPos;
  if (wheelPos < 85) {
    return RGB(255-wheelPos*3, 0, wheelPos*3);
  } else if (wheelPos < 170) {
    wheelPos -= 85;
    return RGB(0, wheelPos*3, 255-wheelPos*3);
  } else {
    wheelPos -= 170;
    return RGB(wheelPos*3, 255-wheelPos*3, 0);
  }
}

static uint32_t normal(int id, uint32_t color) {
  return color;
}

static uint32_t rainbow(int id, uint32_t color) {
  return wheel((id*256/LED_CFG_LEDS_CNT+color)%256);
}

///////////////////////////////////////////////////////////////////////////////

typedef enum {
  STOP,
  FORWARD,
  BACKWARD,
  TURN_LEFT,
  TURN_RIGHT,
} BotState;

static BotState current_state = STOP;
static uint16_t delay = 1;

static const char* text;
static int16_t xpos, ypos;
static int8_t  xdir, ydir;
static uint32_t color;
static COLOR_FN color_fn = normal;

extern osTimerId displayHandle;

void display_init(void) {
  led_set_brightness(10);
  osTimerStart(displayHandle, 100);
}

static BotState get_current_state(void) {
  int16_t m1spd, m2spd;
  
  get_motor_state(&m1spd, &m2spd);
  if (m1spd>0 && m2spd>0)
    return FORWARD;
  if (m1spd<0 && m2spd<0)
    return BACKWARD;
  if (m1spd<0 && m2spd>0)
    return TURN_LEFT;
  if (m1spd>0 && m2spd<0)
    return TURN_RIGHT;
  return STOP;
}

void display_task(void const* args) {
  BotState state = get_current_state();
  
  if (state != current_state) {
    current_state = state;
    xpos = ypos = 0;
    xdir = ydir = 0;
    color_fn = normal;
    
    switch (state) {
    case FORWARD:
      text  = "\x80";  // up arrow
      color = 0x00FF00;
      ydir  = 1;
      break;
    
    case BACKWARD:
      text  = "\x81";  // down arrow
      color = 0xFF0000;
      ydir  = -1;
      break;
    
    case TURN_LEFT:
      text  = "\x82";  // left arrow
      color = 0xFFFF00;
      xdir  = 1;
      break;
    
    case TURN_RIGHT:
      text  = "\x83";  // right arrow
      color = 0xFFFF00;
      xdir  = -1;
      break;
    
    default:
      text = NULL;
      delay = 600;
      led_fill(0);
      led_update(true);
      break;
    }
  }

  // display logo when idle
  if (text==NULL && --delay==0) {
    text  = "TankBot ";
    xdir  = 1;
    color = 0;
    color_fn = rainbow;
  }
  
  if (text != NULL) {
    int width  = CHAR_WIDTH * strlen(text);
    int height = CHAR_HEIGHT;
    
    display_string_at(text, xpos, ypos, color, color_fn);
    
    xpos = (xpos+xdir+width ) % width;
    ypos = (ypos+ydir+height) % height;
    
    if (color_fn != normal) 
      color++;
  }
}
