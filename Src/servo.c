#include "main.h"
#include "cmsis_os.h"
#include "servo.h"
#include "ps2controller.h"

#define SERVO_CNT         6
#define SERVO_PWM_INC     20

#define D2P(d)  ((d)*2000/180+500)    /* Convert degree to PWM duty cycle */
#define P2D(p)  (((p)-500)*180/2000)  /* Convert PWM duty cycle to degree */

static const uint16_t servo_pwm_init[] = {
  D2P(90), D2P(90), D2P(90), D2P(40), D2P(60), D2P(92)
};

static __IO uint16_t servo_pwm[SERVO_CNT];
static __IO uint16_t servo_pwm_set[SERVO_CNT];

static GPIO_TypeDef* const servo_ports[] = {
  SERVO1_GPIO_Port,
  SERVO2_GPIO_Port,
  SERVO3_GPIO_Port,
  SERVO4_GPIO_Port,
  SERVO5_GPIO_Port,
  SERVO6_GPIO_Port,
};

static uint32_t const servo_pins[] = {
  SERVO1_Pin,
  SERVO2_Pin,
  SERVO3_Pin,
  SERVO4_Pin,
  SERVO5_Pin,
  SERVO6_Pin,
};

void servo_init(void) {
  for (int i=0; i<SERVO_CNT; i++) {
    servo_pwm[i]     = servo_pwm_init[i];
    servo_pwm_set[i] = servo_pwm_init[i];
  }
  
  HAL_TIM_Base_Start_IT(SERVO_TIM);
  HAL_TIM_OC_Start_IT(SERVO_TIM, SERVO_TIM_CHANNEL);
}

void servo_reset(void) {
  for (int i=0; i<SERVO_CNT; i++) {
    servo_pwm_set[i] = servo_pwm_init[i];
  }
}

uint8_t servo_set(uint8_t id, int angle) {
  if (id==0 || id>SERVO_CNT)
    return 0;
  if (angle < 0)
    angle = 0;
  if (angle > 180)
    angle = 180;
  
  servo_pwm_set[id-1] = (uint16_t)D2P(angle);
  if (id<=2)
    servo_pwm[id-1] = (uint16_t)D2P(angle);
  
  return 1;
}

uint8_t servo_add(uint8_t id, int delta) {
  return servo_set(id, servo_get(id) + delta);
}

int servo_get(uint8_t id) {
  if (id==0 || id>SERVO_CNT)
    return 0;
  
  int angle = ((int)servo_pwm[id-1]-500)*180;
  if ((angle%2000) > 1000)
    return angle/2000+1;
  else
    return angle/2000;
}

void servo_pwm_start(void) {
  uint16_t min_pwm = 0xFFFF;

  // set target PWM duty cycles
  for (int i=0; i<SERVO_CNT; i++) {
    uint16_t cur = servo_pwm[i];
    uint16_t set = servo_pwm_set[i];
    if (cur != set) {
      if (cur < set) {
        if ((cur+=SERVO_PWM_INC) > set)
          cur = set;
      } else {
        if ((cur-=SERVO_PWM_INC) < set)
          cur = set;
      }
      servo_pwm[i] = cur;
    }
  }
  
  // start PWM pulse rising edges
  for (int i=0; i<SERVO_CNT; i++) {
    HAL_GPIO_WritePin(servo_ports[i], servo_pins[i], GPIO_PIN_SET);
    if (servo_pwm[i] < min_pwm) {
      min_pwm = servo_pwm[i];
    }
  }
  
  // set next PWM falling edge time
  __HAL_TIM_SET_COMPARE(SERVO_TIM, SERVO_TIM_CHANNEL, min_pwm);
}

void servo_pwm_pulse(void) {
  uint16_t pwm = __HAL_TIM_GET_COMPARE(SERVO_TIM, SERVO_TIM_CHANNEL);
  uint16_t next_pwm = 0xFFFF;
  
  // set PWM falling edges
  for (int i=0; i<SERVO_CNT; i++) {
    if (servo_pwm[i] <= pwm) {
      HAL_GPIO_WritePin(servo_ports[i], servo_pins[i], GPIO_PIN_RESET);
    } else if (servo_pwm[i] < next_pwm) {
      next_pwm = servo_pwm[i];
    }
  }
  
  // set next PWM failling edge time
  if (next_pwm != 0xFFFF) {
    __HAL_TIM_SET_COMPARE(SERVO_TIM, SERVO_TIM_CHANNEL, next_pwm);
  }
}

///////////////////////////////////////////////////////////////////////////////

#define SERVO_MAX_RECORD    50

static uint16_t servo_records[SERVO_MAX_RECORD][SERVO_CNT];
static uint8_t  servo_record_cnt = 0;

void servo_start_record(void) {
  servo_record_cnt = 0;
}

void servo_record(void) {
  if (servo_record_cnt < SERVO_MAX_RECORD) {
    for (int i=0; i<SERVO_CNT; i++) {
      servo_records[servo_record_cnt][i] = servo_pwm[i];
    }
    servo_record_cnt++;
  }
}

static uint8_t is_servo_pwm_set(void) {
  for (int i=0; i<SERVO_CNT; i++) {
    if (servo_pwm[i] != servo_pwm_set[i])
      return 0;
  }
  return 1;
}

static uint8_t is_key_pressed(void) {
  return ps2_get_key(0) != 0;
}

void servo_replay(void) {
  if (servo_record_cnt == 0) {
    return;
  }
  
  while (!is_key_pressed()) {
    for (int j=0; j<servo_record_cnt; j++) {
      // Play a single record in the sequence
      for (int i=0; i<SERVO_CNT; i++) {
        servo_pwm_set[i] = servo_records[j][i];
      }
      
      // Wait until servo run into place
      while (!is_servo_pwm_set()) {
        if (is_key_pressed())
          return;
      }
      
      // Wait a little time
      uint16_t time = 50;
      while (--time) {
        if (is_key_pressed())
          return;
        osDelay(1);
      }
    }
  }
}
