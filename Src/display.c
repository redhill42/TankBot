#include <string.h>
#include "main.h"
#include "cmsis_os.h"
#include "display.h"
#include "ledmatrix.h"
#include "motor.h"
#include "servo.h"

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
static void display_string_at(const char* str, int xpos, int ypos, uint32_t color, COLOR_FN color_fn) {
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
  UNKNOWN,
  IDLE,
  NEW_MESSAGE,
  MESSAGE,
  FORWARD,
  BACKWARD,
  TURN_LEFT,
  TURN_RIGHT,
  BATTERY_LOW,
} BotState;

static BotState current_state = IDLE;
static uint16_t idle_counter = 1;
static uint16_t display_counter = 0;

static const Message_t* post_message;
static const Message_t* current_message;
static uint8_t next_message_id;

/* Special address to indicate a clear_message command was issued */
#define CLEAR_MESSAGE ((const Message_t*)0xFFFFFFFF)

static const char* text;
static int16_t xpos, ypos;
static int8_t  xdir, ydir;
static uint32_t color;
static COLOR_FN color_fn = normal;
static bool blink = false;
static bool blink_state = false;
static uint8_t blink_counter = 0;

extern osTimerId display_timerHandle;
extern osMutexId display_mutexHandle;

extern void battery_check_init(void);
extern bool battery_low(void);

void display_init(void) {
  led_set_brightness(10);
  osTimerStart(display_timerHandle, 100);
  battery_check_init();
}

void message_init(Message_t* message, MessageLevel_t level, const char* text) {
  osMutexWait(display_mutexHandle, osWaitForever);
  message->id = ++next_message_id;
  osMutexRelease(display_mutexHandle);
  
  message->level = level;
  message->text = text;
  message->keep_time = -1;
  message->blink = false;
  
  switch (level) {
  case ALERT:
    message->color = 0xFF0000;
    message->keep_time = 0;
    break;
  case WARNING:
    message->color = 0xFFFF00;
    break;
  default:
    message->color = 0x0000FF;
    break;
  }
}

static uint32_t get_message_keep_time(const Message_t* message) {
  if (message->keep_time >= 0)
    return message->keep_time;
  else
    return (-message->keep_time)*CHAR_WIDTH*strlen(message->text) - CHAR_WIDTH;
}

bool display_message(const Message_t* message) {
  bool retval = false;
  if (osMutexWait(display_mutexHandle, 500) == osOK) {
    // the more important message have a lower message level value
    if (current_message==NULL || message->level<=current_message->level) {
      post_message = message;
      retval = true;
    }
    osMutexRelease(display_mutexHandle);
  }
  return retval;
}

bool clear_message(uint8_t id) {
  bool retval = false;
  if (osMutexWait(display_mutexHandle, 500) == osOK) {
    if (current_message!=NULL && current_message->id==id) {
      post_message = CLEAR_MESSAGE;
      retval = true;
    }
    osMutexRelease(display_mutexHandle);
  }
  return retval;
}

static void update_servo_recording_state() {
  uint8_t cnt = servo_record_count();
  
  led_fill(0);
  for (int i=0; i<LED_CFG_LEDS_CNT; i++) {
    if (i < cnt) {
      led_set_color(i, 0xFF0000);
    } else if (i < SERVO_MAX_RECORDS) {
      led_set_color(i, 0x0000FF);
    } else {
      led_set_color(i, 0);
    }
  }
  led_update(1);
}

static BotState get_motor_state(void) {
  int16_t m1spd, m2spd;
  
  get_motor_speed(&m1spd, &m2spd);
  if (m1spd>0 && m2spd>0)
    return FORWARD;
  if (m1spd<0 && m2spd<0)
    return BACKWARD;
  if (m1spd<0 && m2spd>0)
    return TURN_LEFT;
  if (m1spd>0 && m2spd<0)
    return TURN_RIGHT;
  return IDLE;
}

void display_task(void const* args) {
  BotState state = UNKNOWN;
  const Message_t* message = NULL;
  
  // stop display when display time elapsed
  if (text!=NULL && display_counter!=0 && --display_counter==0) {
    current_state = UNKNOWN;
  }

  // get message atomically
  if (osMutexWait(display_mutexHandle, 0) == osOK) {
    message = post_message;
    post_message = NULL;
    if (message == CLEAR_MESSAGE) {
      message = NULL;
      current_message = NULL;
      current_state = UNKNOWN;
    }
    osMutexRelease(display_mutexHandle);
  }
  
  // check current state
  if (message != NULL) {
    state = NEW_MESSAGE;
  } else if (current_state == MESSAGE) {
    state = MESSAGE;
  } else if (servo_is_recording()) {
    update_servo_recording_state();
    current_state = UNKNOWN;
    return;
  } else {
    state = get_motor_state();
  }
  
  if (state==IDLE && battery_low()) {
    state = BATTERY_LOW;
  }
  
  // update if state changed
  if (state != current_state) {
    current_state = state;
    text = NULL;
    xpos = ypos = 0;
    xdir = ydir = 0;
    color_fn = normal;
    display_counter = 0;
    blink = false;
    blink_state = false;
    blink_counter = 0;

    switch (state) {
    case NEW_MESSAGE:
      current_message = message;
      current_state = MESSAGE;
      text = message->text;
      color = message->color;
      blink = message->blink;
      xdir = strlen(text)>1 ? 1 : 0;
      display_counter = get_message_keep_time(message);
      break;
    
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
    
    case BATTERY_LOW:
      text  = "\x84"; // battery
      color = 0x0000FF;
      blink = true;
      break;
    
    default:
      current_state = IDLE;
      idle_counter = 600;
      led_fill(0);
      led_update(true);
      break;
    }
  }
  
  // display logo when idle
  if (current_state==IDLE && text==NULL && --idle_counter==0) {
    text  = "TankBot ";
    xdir  = 1;
    color = 0;
    color_fn = rainbow;
  }
  
  // display the current text
  if (text != NULL) {
    int width  = CHAR_WIDTH * strlen(text);
    int height = CHAR_HEIGHT;

    if (blink && blink_counter++ == 5) {
      blink_state = !blink_state;
      blink_counter = 0;
    }
    
    if (blink && !blink_state) {
      led_fill(0);
      led_update(true);
    } else {
      display_string_at(text, xpos, ypos, color, color_fn);
    }
    
    xpos = (xpos+xdir+width ) % width;
    ypos = (ypos+ydir+height) % height;
    
    if (color_fn != normal) 
      color++;
  }
}
