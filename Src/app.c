#include "main.h"
#include "cmsis_os.h"
#include "servo.h"
#include "motor.h"
#include "ps2controller.h"
#include "beep.h"

#define DELTA 2

struct KeyPress {
  uint8_t pressed;
  uint32_t time;
};

enum KeyPressState {
  NO_KEY_PRESS,
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
  return NO_KEY_PRESS;
}

void app_main(const void* args) {
  uint16_t key;
  uint8_t lx, ly;
  struct KeyPress select_key = {0};
  struct KeyPress start_key = {0};
  
  servo_init();
  motor_init();
  ps2_init();
  
  while (1) {
    key = ps2_get_key(PSB_SELECT|PSB_START|PSB_L3);
    lx  = ps2_get_stick(PSS_LX);
    ly  = ps2_get_stick(PSS_LY);
    
    switch (check_key_press(key, PSB_SELECT, &select_key)) {
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
    
    switch (check_key_press(key, PSB_START, &start_key)) {
      case KEY_PRESS_TIMEOUT:
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        break;
      case KEY_PRESS_LONG:
        servo_replay();
        break;
      case KEY_PRESS_SHORT:
        servo_reset();
        break;
      default:
        break;
    }
    
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
    
    if (key & PSB_L3) {
      beep_start();
    } else {
      beep_stop();
    }

    int16_t speed = (127-ly)/12*100; // map 0..255 to 1000..-1000
    if (speed != 0) {
      motor_control(speed, speed);
    } else if (lx == 0) { 
      // turn left
      motor_control(-1000, 1000);
    } else if (lx == 255) {
      // turn right
      motor_control(1000, -1000);
    } else {
      // stop
      motor_control(0, 0);
    }
    
    osDelay(1);
  }
}
