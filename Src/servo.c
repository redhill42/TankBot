#include "main.h"
#include "cmsis_os.h"
#include "servo.h"

///////////////////////////////////////////////////////////////////////////////
// Definitions

#define SERVO_PWM_INC     20
#define SERVO_MAX_RECORD  50
#define SERVO_DELAY       20

/* Convert degree to PWM duty cycle */
#define D2P(d)  ((d)*2000/180+500)

///////////////////////////////////////////////////////////////////////////////
// Servo GPIO Pins

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

///////////////////////////////////////////////////////////////////////////////
// Servo PWM duty cycles

static const uint16_t servo_pwm_init[] = {
  D2P(90), D2P(90), D2P(50), D2P(0), D2P(60), D2P(96)
};

static __IO uint16_t servo_pwm[SERVO_CNT];
static __IO uint16_t servo_pwm_set[SERVO_CNT];

static __IO uint8_t servo_action; /* servo movement is in progress */

///////////////////////////////////////////////////////////////////////////////
// Servo replay sequence

static uint16_t  servo_records[SERVO_MAX_RECORD][SERVO_CNT];
static uint8_t   servo_record_cnt = 0;
static uint8_t   servo_replay_started = 0;
static uint8_t   servo_replay_step = 0;

static int16_t const (*servo_sequence)[SERVO_CNT];
static size_t    servo_sequence_length;
static size_t    servo_sequence_step;
static uint32_t  servo_sequence_delay;

///////////////////////////////////////////////////////////////////////////////
// Servo background event handling

#define SIG_START_RECORD  0x0001
#define SIG_RECORD        0x0002
#define SIG_START_REPLAY  0x0004
#define SIG_STOP_REPLAY   0x0008

extern osThreadId servo_taskHandle;
extern osMutexId  servo_mutexHandle;

#define signal(sig)     osSignalSet(servo_taskHandle,(sig))
#define lock(timeout)   (osMutexWait(servo_mutexHandle,timeout)==osOK)
#define unlock()        osMutexRelease(servo_mutexHandle);

///////////////////////////////////////////////////////////////////////////////
// Global functions

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
  return 1;
}

uint8_t servo_add(uint8_t id, int delta) {
  return servo_set(id, servo_get(id) + delta);
}

static void set_target_pwm(void) {
  uint8_t action = 0;
  
  // Set target PWM duty cycles. This is done in every 20ms (50Hz)
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
      action = 1;
    }
  }
  
  servo_action = action;
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

uint8_t servo_in_action(void) {
  return servo_action;
}

void servo_pwm_start(void) {
  uint16_t min_pwm = 0xFFFF;

  set_target_pwm();
  
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

void servo_start_record(void) {
  signal(SIG_START_RECORD);
}

void servo_record(void) {
  signal(SIG_RECORD);
}

void servo_start_replay(void) {
  signal(SIG_START_REPLAY);
}

void servo_stop_replay(void) {
  signal(SIG_STOP_REPLAY);
}

uint8_t servo_play_sequence(const int16_t (*sequence)[SERVO_CNT], size_t length, uint32_t delay) {
  if (!lock(100)) {
    return 0;
  }
  
  if (servo_sequence != NULL || servo_replay_started) {
    unlock();
    return 0;
  }
  
  servo_sequence = sequence;
  servo_sequence_length = length;
  servo_sequence_step = 0;
  servo_sequence_delay = delay;
  
  unlock();
  return 1;
}

uint8_t servo_sequence_finished(void) {
  return servo_sequence == NULL;
}

///////////////////////////////////////////////////////////////////////////////
// The servo event handling

static void do_start_record() {
  servo_replay_started = 0;
  servo_record_cnt = 0;
}

static void do_record(void) {
  if (!servo_replay_started && servo_record_cnt<SERVO_MAX_RECORD) {
    for (int i=0; i<SERVO_CNT; i++) {
      servo_records[servo_record_cnt][i] = servo_pwm[i];
    }
    servo_record_cnt++;
  }
}

static void do_start_replay(void) {
  if (!servo_replay_started && servo_record_cnt>0) {
    servo_replay_started = 1;
    servo_replay_step = 0;
  }
}

static void do_stop_replay(void) {
  servo_replay_started = 0;
}

static void do_replay(void) {
  for (int i=0; i<SERVO_CNT; i++)
    servo_pwm_set[i] = servo_records[servo_replay_step][i];
  servo_replay_step = (servo_replay_step+1)%servo_record_cnt;
}

static void do_play_sequence(void) {
  if (!lock(0)) {
    return;
  }
  
  if (servo_sequence != NULL) {
    for (int i=0; i<SERVO_CNT; i++) {
      int16_t angle = servo_sequence[servo_sequence_step][i];
      if (angle>=0 && angle<=180)
        servo_set(i+1, angle);
    }
    
    if (++servo_sequence_step == servo_sequence_length) {
      servo_sequence = NULL;
    }
  }
  
  unlock();
}

void servo_daemon(void const* args) {
  uint32_t delay = SERVO_DELAY;
  
  for (;;) {
    osEvent evt = osSignalWait(0xFFFF, delay);
    delay = SERVO_DELAY;
    
    if (evt.status == osEventSignal) {
      if (evt.value.signals & SIG_START_RECORD) {
        do_start_record();
      } else if (evt.value.signals & SIG_RECORD) {
        do_record();
      } else if (evt.value.signals & SIG_START_REPLAY) {
        do_start_replay();
      } else if (evt.value.signals & SIG_STOP_REPLAY) {
        do_stop_replay();
      }
    }

    if (servo_action) {
      continue;
    }
    
    if (servo_replay_started) {
      do_replay();
      delay = 5*SERVO_DELAY;
    } else if (servo_sequence != NULL) {
      do_play_sequence();
      delay = servo_sequence_delay;
    }
  }
}
