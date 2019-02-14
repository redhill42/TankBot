#include "main.h"
#include "cmsis_os.h"
#include "servo.h"
#include "motor.h"
#include "ps2controller.h"
#include "adxl345.h"
#include "rangefinder.h"
#include "ledmatrix.h"
#include "beep.h"
#include "delay.h"

extern void display_init(void);

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

static const int16_t grab_left[][6] = {
/*   1   2   3   4    5    6   */
  { 90, 90,  0,  0, 144,  90}, // 1: standby
  {120, -1, 90, 90,  90,  90}, // 2: stretch
  { -1, -1, -1, -1,  -1, 180}, // 3: rotate shoulder
  { -1, -1, -1, -1,   0,  -1}, // 4: hold up with shoulder
  { -1, -1, -1, 10,  -1,  -1}, // 5: hold up with elbow
  { -1, -1, -1, 45,  36,  -1}, // 6: lift up shoulder
  { -1, -1, -1, -1,  -1,  90}, // 7: rotate shoulder forward
  { 90, 90, 50,  0,  60,  96}  // 8: reset all
};

static const int16_t grab_right[][6] = {
/*   1   2   3   4    5   6   */
  { 90, 90,  0,  0, 144, 90}, // 1: standby
  {120, -1, 90, 90,  90, 90}, // 2: stretch
  { -1, -1, -1, -1,  -1,  0}, // 3: rotate shoulder
  { -1, -1, -1, -1,   0, -1}, // 4: hold up with shoulder
  { -1, -1, -1, 10,  -1, -1}, // 5: hold up with elbow
  { -1, -1, -1, 45,  36, -1}, // 6: lift up shoulder
  { -1, -1, -1, -1,  -1, 90}, // 7: rotate shoulder forward
  { 90, 90, 50,  0,  60, 96}  // 8: reset all
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
  get_motor_state(&m1speed, &m2speed);
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

#define SERVO_DELTA 5

static void control_servo(uint16_t key) {
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

static bool check_collision(bool forward) {
  static bool impact = false;
  
  uint32_t distance = get_distance();
  if (distance == INVALID_DISTANCE) {
    return false;
  }
  
  if (forward && (distance<300 || (impact && distance<310))) {
    if (!impact) {
      impact = true;
      beep_start("=6:50/1CR@", true);
    }
  } else {
    if (impact) {
      impact = false;
      beep_stop();
    }
  }
  
  return impact;
}

static void control_motor(uint8_t lx, uint8_t ly) {
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

static void control_beep(bool on) {
  static bool beep = 0;
  
  if (on) {
    if (!beep) {
      beep = 1;
      beep_start("=6C0", false);
    }
  } else {
    if (beep) {
      beep = 0;
      beep_stop();
    }
  }
}

void app_main(const void* args) {
  uint16_t key;
  uint8_t  lx, ly;
  
  struct KeyPress select = {0};
  struct KeyPress start = {0};

  servo_init();
  motor_init();
  ps2_init();
  adxl345_init();
  rangefinder_init();
  display_init();
  osDelay(200);
  
  for (;;osDelay(1)) {
    if (tilt_detect()) {
      continue;
    }
    
    key = ps2_get_key(PSB_SELECT|PSB_START|PSB_L3);
    lx  = ps2_get_stick(PSS_LX);
    ly  = ps2_get_stick(PSS_LY);
    
    switch (check_key_press(key, PSB_SELECT, &select)) {
      case KEY_PRESS_LONG:
        beep_start("=5:60/1A", false);
        servo_start_record();
        break;
      case KEY_PRESS_SHORT:
        beep_start("=5:60/1A", false);
        servo_record();
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

    control_servo(key);
    control_motor(lx, ly);
    control_beep((key&PSB_L3) != 0);
  }
}
