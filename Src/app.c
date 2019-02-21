#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "servo.h"
#include "motor.h"
#include "ps2controller.h"
#include "adxl345.h"
#include "rangefinder.h"
#include "display.h"
#include "beep.h"
#include "delay.h"

static Message_t stop_message;

static int map(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min;
}

struct KeyPress {
  stopwatch_t time;
  bool pressed;
};

enum KeyPressState {
  KEY_PRESS_NONE,
  KEY_PRESS_LONG,
  KEY_PRESS_SHORT
};

static enum KeyPressState check_key_press(uint16_t key, uint16_t mask, struct KeyPress* kp) {
  if (key & mask) {
    if (!kp->pressed) {
      // key pressed, start timer
      kp->pressed = 1;
      SW_Reset(&kp->time);
    } else {
      // key repeately pressed, increment timer and check for timeout
      if (SW_Elapsed(&kp->time, 500)) {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        return KEY_PRESS_NONE;
      }
    }
  } else {
    if (kp->pressed) {
      // key released, check timer for long or short press
      kp->pressed = 0;
      if (SW_Elapsed(&kp->time, 500)) {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        return KEY_PRESS_LONG;
      } else {
        return KEY_PRESS_SHORT;
      }
    }
  }
  return KEY_PRESS_NONE;
}

static const int16_t grab[][6] = {
/*   1     2     3     4     5     6   */
 { 900,  900, 1450,    0, 1440,  850}, // standby
 { 200,   -1,  200,  200,    0,   -1}, // arm down
 {1200,   -1,  450,  350,   -1,   -1}, // grab
 {  -1,   -1, 1400,    0, 1400,   -1}, // lift up
 {  -1,   -1,   -1,   -1,   -1,    0}, // rotate
 {  -1,   -1,  450,  350,    0,   -1}, // arm down
 {  -1,   -1,   -1,   -1,   -1,   -1}, // delay
 {   0,   -1,   -1,   -1,   -1,   -1}, // drop
 {  -1,   -1, 1450,  500,  900,   -1}, // lift up
 { 900,  900, 1450,    0, 1440,  850}  // reset all
};

static const int16_t grab_left[][6] = {
/*    1    2    3    4     5     6   */
  { 900, 900,   0,   0, 1440,  850}, // 1: standby
  {1200,  -1, 900, 900,  900,  850}, // 2: stretch
  {  -1,  -1,  -1,  -1,   -1, 1800}, // 3: rotate shoulder
  {  -1,  -1,  -1,  -1,    0,   -1}, // 4: hold up with shoulder
  {  -1,  -1,  -1, 200,   -1,   -1}, // 5: hold up with elbow
  {  -1,  -1,  -1, 450,  360,   -1}, // 6: lift up shoulder
  {  -1,  -1,  -1,  -1,   -1,  850}, // 7: rotate shoulder forward
  { 900, 900, 500,   0,  600,  850}  // 8: reset all
};

static const int16_t grab_right[][6] = {
/*    1    2    3    4     5    6   */
  { 900, 900,   0,   0, 1440, 850}, // 1: standby
  {1200,  -1, 900, 900,  900, 850}, // 2: stretch
  {  -1,  -1,  -1,  -1,   -1,   0}, // 3: rotate shoulder
  {  -1,  -1,  -1,  -1,    0,  -1}, // 4: hold up with shoulder
  {  -1,  -1,  -1, 200,   -1,  -1}, // 5: hold up with elbow
  {  -1,  -1,  -1, 450,  360,  -1}, // 6: lift up shoulder
  {  -1,  -1,  -1,  -1,   -1, 850}, // 7: rotate shoulder forward
  { 900, 900, 500,   0,  600, 850}  // 8: reset all
};

// Tilt states:
//    Unknow:   The tilt state is unknown, read accelerometer to get the state
//    Detected: Tilt detected, delay one more second to check again
//    Recover:  Recovering from tilt state
#define TILT_UNKNOW   0
#define TILT_DETECTED 1
#define TILT_RECOVER  2

#define TILT_LEFT     1
#define TILT_RIGHT    2

#define TILT_CHECKTIME    1000
#define TILT_CONFIRMTIME  2000

static bool tilt_detect(void) {
  static uint8_t     state = TILT_UNKNOW;
  static uint8_t     mode;
  static stopwatch_t checktime;

  int16_t m1speed, m2speed;
  short x, y, z;
  uint8_t t;

  // Don't detect if motor is moving
  get_motor_speed(&m1speed, &m2speed);
  if (!(m1speed==0 && m2speed==0)) {
    return false;
  }
  
  if (state == TILT_UNKNOW && SW_Elapsed(&checktime, TILT_CHECKTIME)) {
    // Read tilt state from accelerometer
    adxl345_read(&x, &y, &z);
    mode = (x<-200) ? TILT_LEFT : (x>200) ? TILT_RIGHT : 0;
    if (mode != 0)
      state = TILT_DETECTED;
    SW_Reset(&checktime);
    return false;
  }
  
  if (state == TILT_DETECTED && SW_Elapsed(&checktime, TILT_CONFIRMTIME)) {
    // Read accelerometer again to make sure tilt state not change
    adxl345_read(&x, &y, &z);
    t = (x<-200) ? TILT_LEFT : (x>200) ? TILT_RIGHT : 0;
    
    // if tilt mode changed, then restart from beginning
    if (t != mode) {
      state = TILT_UNKNOW;
      SW_Reset(&checktime);
      return false;
    }
    
    // tilt really happend, recover from tilt
    state = TILT_RECOVER;
    if (mode == TILT_LEFT) {
      servo_play_sequence(grab_left, sizeof(grab_left)/sizeof(*grab_left), 1000);
    } else {
      servo_play_sequence(grab_right, sizeof(grab_right)/sizeof(*grab_right), 1000);
    }
    return true;
  }
  
  // Wait until recovering finished
  if (state == TILT_RECOVER) {
    if (servo_sequence_finished()) {
      state = TILT_UNKNOW;
      SW_Reset(&checktime);
      return false;
    } else {
      return true;
    }
  }
  
  return false;
}

#define SERVO_DELTA 50

static void do_servo_control(uint16_t key) {
  if (key == 0) {
    return;
  }
  
  servo_stop_replay();

  if (key & PSB_PAD_UP)
    servo_add(SERVO_SHOULDER_UD, -SERVO_DELTA);
  if (key & PSB_PAD_DOWN)
    servo_add(SERVO_SHOULDER_UD, SERVO_DELTA);
  if (key & PSB_PAD_LEFT)
    servo_add(SERVO_SHOULDER_ROT, SERVO_DELTA);
  if (key & PSB_PAD_RIGHT)
    servo_add(SERVO_SHOULDER_ROT, -SERVO_DELTA);
  if (key & PSB_TRIANGLE)
    servo_add(SERVO_ELBOW_UD, -SERVO_DELTA);
  if (key & PSB_CROSS)
    servo_add(SERVO_ELBOW_UD, SERVO_DELTA);
  if (key & PSB_SQUARE)
    servo_add(SERVO_WRIST_UD, -SERVO_DELTA);
  if (key & PSB_CIRCLE)
    servo_add(SERVO_WRIST_UD, SERVO_DELTA);
  if (key & PSB_L1)
    servo_add(SERVO_WRIST_ROT, -SERVO_DELTA);
  if (key & PSB_R1)
    servo_add(SERVO_WRIST_ROT, SERVO_DELTA);
  if (key & PSB_L2)
    servo_add(SERVO_PAW, -SERVO_DELTA);
  if (key & PSB_R2)
    servo_add(SERVO_PAW, SERVO_DELTA);
}

/* 使用抛物线函数逼近正弦函数，最大误差0.1% */
static int fast_sin(int l, int x) {
  int y;
  
  // always wrap input angle to -180..180
  while (x < -1800)
    x += 3600;
  while (x > 1800)
    x -= 3600;
  
  if (x < 0) {
    y = l*x*(1800+x)/810000;
    return (78*y - 22*y*y/l)/100;
  } else {
    y = l*x*(1800-x)/810000;
    return (78*y + 22*y*y/l)/100;
  }
}

static int fast_cos(int l, int x) {
  return fast_sin(l, x+900);
}

#define BASE_LENGTH       70
#define SAFE_DISTANCE     100
#define MIN_DISTANCE      150

#define UPPERARM_LENGTH   110
#define FOREARM_LENGTH    90
#define PAW_LENGTH        180
#define MIN_PAW_LENGTH    45

static int compute_arm_extent(void) {
  int upperarm_degree = (int)servo_get(SERVO_SHOULDER_UD);
  int forearm_degree  = upperarm_degree - 90 + (int)servo_get(SERVO_ELBOW_UD);
  int paw_degree      = forearm_degree + 90 - (int)servo_get(SERVO_WRIST_UD);
  
  int upperarm_extent = fast_cos(UPPERARM_LENGTH, upperarm_degree);
  int forearm_extent  = fast_cos(FOREARM_LENGTH, forearm_degree);
  int paw_extent      = fast_cos(PAW_LENGTH, paw_degree);

  if (paw_extent < MIN_PAW_LENGTH)
    paw_extent = MIN_PAW_LENGTH;
  
  int extent = upperarm_extent + forearm_extent + paw_extent - BASE_LENGTH;
  if (extent > 0)
    extent += SAFE_DISTANCE;
  else
    extent = MIN_DISTANCE;
  return extent;
}

static bool check_collision(bool forward) {
  static bool impact = false;
  
  if (forward) {
    uint32_t distance = get_distance();
    int extent = compute_arm_extent();

    if (distance == INVALID_DISTANCE) {
      return impact; // ignore invalid distance value
    }
    
    if (distance<extent || (impact && distance<extent+20)) {
      if (!impact) {
        impact = true;
        display_message(&stop_message);
        beep_start("=6:50/1c$@", true);
      }
      return true;
    } // else fallthrough
  }
  
  if (impact) {
    impact = false;
    clear_message(stop_message.id);
    beep_stop();
  }
  
  return false;
}

static void do_motor_control(uint8_t lx, uint8_t ly) {
  int16_t speed;
  int16_t m1spd, m2spd;
  
  if (ly <= 110) {
    if (lx < 55) {
      // turn left
      m1spd = 600;
      m2spd = 1000;
    } else if (lx > 200) {
      // turn right
      m1spd = 1000;
      m2spd = 600;
    } else {
      // forward, map 110..0 to 600..1000
      speed = map(ly, 110, 0, 6, 11) * 100;
      if (speed > 1000)
        speed = 1000;
      m1spd = m2spd = speed;
    }
  } else if (ly >= 145) {
    if (lx < 55) {
      // turn back left
      m1spd = -600;
      m2spd = -1000;
    } else if (lx > 200) {
      // turn back right
      m1spd = -1000;
      m2spd = -600;
    } else {
      // backward, map 145..255 to -600..-1000
      speed = map(ly, 145, 255, 6, 11) * 100;
      if (speed > 1000)
        speed = 1000;
      m1spd = m2spd = -speed;
    }
  } else if (lx == 0) { 
    // rotate left
    m1spd = -1000;
    m2spd = 1000;
  } else if (lx == 255) {
    // rotate right
    m1spd = 1000;
    m2spd = -1000;
  } else {
    // stop
    m1spd = m2spd = 0;
  }

  // stop if impact
  if (check_collision(m1spd>0 && m2spd>0)) {
    m1spd = m2spd = 0;
  }

  motor_control(m1spd, m2spd);
}

static void do_beep_control(bool on) {
  static bool beep = 0;
  
  if (on) {
    if (!beep) {
      beep = 1;
      beep_start("=4a0", false);
    }
  } else {
    if (beep) {
      beep = 0;
      beep_stop();
    }
  }
}

static void special_action(void);

void app_main(const void* args) {
  uint16_t key;
  uint8_t  lx, ly;

  struct KeyPress select = {0};
  struct KeyPress start = {0};
  struct KeyPress r3 = {0};

  I2C_Reset();
  servo_init();
  motor_init();
  ps2_init();
  adxl345_init();
  rangefinder_init();
  display_init();
  message_init(&stop_message, ALERT, "\x85");
  osDelay(200);
  
  for (;;osDelay(1)) {
    if (tilt_detect()) {
      continue;
    }
    
    key = ps2_get_key(PSB_SELECT|PSB_START|PSB_L3|PSB_R3);
    lx  = ps2_get_stick(PSS_LX);
    ly  = ps2_get_stick(PSS_LY);
    
    switch (check_key_press(key, PSB_SELECT, &select)) {
      case KEY_PRESS_LONG:
        beep_start("=5:60/1a", false);
        servo_toggle_recording();
        break;
      case KEY_PRESS_SHORT:
        if (servo_is_recording()) {
          beep_start("=5:60/1a", false);
          servo_record();
        }
        break;
      default:
        break;
    }
    
    switch (check_key_press(key, PSB_START, &start)) {
      case KEY_PRESS_LONG:
        servo_start_replay();
        break;
      case KEY_PRESS_SHORT:
        servo_reset();
        break;
      default:
        break;
    }
    
    switch (check_key_press(key, PSB_R3, &r3)) {
    case KEY_PRESS_LONG:
      servo_play_sequence(grab, sizeof(grab)/sizeof(*grab), 1000);
      break;
    case KEY_PRESS_SHORT:
      special_action();
      break;
    default:
      break;
    }

    do_servo_control(key);
    do_motor_control(lx, ly);
    do_beep_control((key&PSB_L3) != 0);
  }
}

static void special_action(void) {
  static const char* const musics[] = {
    /* imperial march */
    "=4:1500/3"
    "aaaf6.C12.af6.C12.a2.$12"
    "EEEF6.C12g#f6.C12.a2.$12"
    "Aa6.a12AG#6G6F#12F12F#6$6"
    "a#6D#D6C#6C12b12C6$6"
    "f12g#f6.a12Ca6.C12E2.$12"
    "Aa6.a12AG#6G6F#12F12F#6$6"
    "a#6D#D6C#6C12b12C6$6"
    "f6g#f6.C12af6.C12a2.",

    /* pirates of caribbean */
    "=4:500/4"
    "ega2a$abC2C$CDb2b$aga2.$"
    "ega2a$abC2C$CDb2b$aga2.$"
    "ega2a$aCD2D$DEF2F$EDEa2$"
    "abC2C$DEa2$aCb2b$Cab2.$2."
    "a2a"
    "abC2C$CDb2b$aga2.$"
    "ega2a$abC2C$CDb2b$aga2.$"
    "ega2a$aCD2D$DEF2F$EDEa2$"
    "abC2C$DEa2$aCb2b$Cab2.$2."
    "E2$$2.F2$$2.EE$GED$$2.D2$$2.C2$$2.bC$b$a1"
    "E2$$2.F2$$2.EE$G$ED$$2.D2$$2.C2$$2.bC$b$a1",
    
    /* tetris */
    "=4:444/2"
    "E1bCDE4D4Cba1aCE1DCb1b4b4CD1E1C1a1a1a1D1DFA1GF"
    "E1ECE1DCb1b4b4CD1E1C1a1a1a1E1bCDE4D4Cba1aCE1DC"
    "b1bCD1E1C1a1a1D1DFAGFE1ECE1DCb1bCD1E1C1a1a1a1",
  };
  
  static int idx = 0;
  
  beep_start(musics[idx], true);
  idx = (idx+1)%(sizeof(musics)/sizeof(*musics));
}
