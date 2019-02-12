#include "main.h"
#include "cmsis_os.h"
#include "servo.h"
#include "motor.h"
#include "ps2controller.h"
#include "adxl345.h"
#include "beep.h"

#define DELTA 2

struct KeyPress {
  uint8_t pressed;
  uint32_t time;
};

enum KeyPressState {
  KEY_PRESS_NONE,
  KEY_PRESS_TIMEOUT,
  KEY_PRESS_LONG,
  KEY_PRESS_SHORT
};

static enum KeyPressState check_key_press(uint16_t key, uint16_t mask, struct KeyPress* kp) {
  if (key & mask) {
    if (!kp->pressed) {
      // key pressed, start timer
      kp->pressed = 1;
      kp->time = 0;
    } else {
      // key repeately pressed, increment timer and check for timeout
      kp->time++;
      if (kp->time > 500) {
        return KEY_PRESS_TIMEOUT;
      }
    }
  } else {
    if (kp->pressed) {
      // key released, check timer for long or short press
      kp->pressed = 0;
      if (kp->time > 500) {
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

#define TILT_LEFT     1
#define TILT_RIGHT    2
#define TILT_TIMEOUT  2000

static uint8_t tilt_detect(void) {
  static uint8_t  tilt;
  static uint32_t tilt_time;

  short x, y, z;
  uint8_t t;

  if (!servo_sequence_finished() ) {
    return 1;
  }
  
  adxl345_read(&x, &y, &z);
  t = (x<-200) ? TILT_LEFT : (x>200) ? TILT_RIGHT : 0;
  
  if (t != tilt) {
    // Tilt detected, start timer
    tilt = t;
    tilt_time = 0;
  } else if (t != 0 && ++tilt_time > TILT_TIMEOUT) {
    // Tilt timeout, recover from tilt
    if (tilt == TILT_LEFT) {
      servo_play_sequence(grab_left, sizeof(grab_left)/sizeof(*grab_left), 1000);
    } else {
      servo_play_sequence(grab_right, sizeof(grab_right)/sizeof(*grab_right), 1000);
    }
    tilt = 0;
    return 1;
  }
  
  return 0;
}

static void control_servo(uint16_t key) {
  if (key == 0) {
    return;
  }
  
  servo_stop_replay();

  if (key & PSB_PAD_UP)
    servo_add(SERVO_SHOULDER_UD, -DELTA);
  if (key & PSB_PAD_DOWN)
    servo_add(SERVO_SHOULDER_UD, DELTA);
  if (key & PSB_PAD_LEFT)
    servo_add(SERVO_SHOULDER_ROT, DELTA);
  if (key & PSB_PAD_RIGHT)
    servo_add(SERVO_SHOULDER_ROT, -DELTA);
  if (key & PSB_TRIANGLE)
    servo_add(SERVO_ELBOW_UD, -DELTA);
  if (key & PSB_CROSS)
    servo_add(SERVO_ELBOW_UD, DELTA);
  if (key & PSB_SQUARE)
    servo_add(SERVO_WRIST_UD, -DELTA);
  if (key & PSB_CIRCLE)
    servo_add(SERVO_WRIST_UD, DELTA);
  if (key & PSB_L1)
    servo_add(SERVO_WRIST_ROT, -DELTA);
  if (key & PSB_R1)
    servo_add(SERVO_WRIST_ROT, DELTA);
  if (key & PSB_L2)
    servo_add(SERVO_PAW, -DELTA);
  if (key & PSB_R2)
    servo_add(SERVO_PAW, DELTA);
}

static int map(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min;
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

  motor_control(m1spd, m2spd);
}

static void control_beep(uint8_t on) {
  static uint8_t beep = 0;
  
  if (on) {
    if (!beep) {
      beep = 1;
      beep_start(440);
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
  osDelay(200);
  
  for (;;osDelay(1)) {
    if (tilt_detect()) {
      continue;
    }
    
    key = ps2_get_key(PSB_SELECT|PSB_START|PSB_L3);
    lx  = ps2_get_stick(PSS_LX);
    ly  = ps2_get_stick(PSS_LY);
    
    switch (check_key_press(key, PSB_SELECT, &select)) {
      case KEY_PRESS_TIMEOUT:
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        break;
      case KEY_PRESS_LONG:
        servo_start_record();
        break;
      case KEY_PRESS_SHORT:
        servo_record();
        break;
      default:
        break;
    }
    
    switch (check_key_press(key, PSB_START, &start)) {
      case KEY_PRESS_TIMEOUT:
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        break;
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
