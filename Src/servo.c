#include "main.h"
#include "cmsis_os.h"
#include "servo.h"
#include "storage.h"

///////////////////////////////////////////////////////////////////////////////
// Definitions

#define SERVO_PWM_INC     20
#define SERVO_DELAY       20

/* Convert degree to PWM duty cycle */
#define D2P(d)  ((d)*2000/1800+500)

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
  D2P(900), D2P(900), D2P(500), D2P(0), D2P(600), D2P(850)
};

static __IO uint16_t servo_pwm[SERVO_CNT];
static __IO uint16_t servo_pwm_set[SERVO_CNT];

static __IO bool servo_action; /* servo movement is in progress */

static uint8_t  servo_pulse_id;
static uint16_t servo_pulse_time;

///////////////////////////////////////////////////////////////////////////////
// Servo replay sequence

static bool      servo_recording = false;
static uint16_t  servo_records[SERVO_MAX_RECORDS][SERVO_CNT];
static uint8_t   servo_record_cnt = 0;
static bool      servo_replay_started = false;
static uint8_t   servo_replay_step = 0;

static int16_t const (*servo_sequence)[SERVO_CNT];
static size_t    servo_sequence_length;
static size_t    servo_sequence_step;
static uint32_t  servo_sequence_delay;

static bool load_records(void);
static bool store_records(void);

///////////////////////////////////////////////////////////////////////////////
// Servo background event handling

#define SIG_TOGGLE_RECORDING  0x0001
#define SIG_RECORD            0x0002
#define SIG_START_REPLAY      0x0004
#define SIG_STOP_REPLAY       0x0008

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
  
  load_records();
  
  HAL_TIM_Base_Start_IT(SERVO_TIM);
}

void servo_reset(void) {
  for (int i=0; i<SERVO_CNT; i++) {
    servo_pwm_set[i] = servo_pwm_init[i];
  }
}

bool servo_set(uint8_t id, int angle) {
  if (id==0 || id>SERVO_CNT)
    return false;

  if (angle < 0)
    angle = 0;
  if (angle > 1800)
    angle = 1800;

  servo_pwm_set[id-1] = (uint16_t)D2P(angle);
  return true;
}

bool servo_add(uint8_t id, int delta) {
  return servo_set(id, servo_get(id) + delta);
}

int servo_get(uint8_t id) {
  if (id==0 || id>SERVO_CNT)
    return 0;
  
  int angle = ((int)servo_pwm[id-1]-500)*1800;
  if ((angle%2000) > 1000)
    return angle/2000+1;
  else
    return angle/2000;
}

bool servo_in_action(void) {
  return servo_action;
}

void servo_pwm_pulse(void) {
  uint8_t id = servo_pulse_id++;
  uint16_t cur, set;

  if (id == 0) {
    servo_action = false;
  } else {
    // lower previous PWM pulse
    servo_ports[id-1]->ODR &= ~servo_pins[id-1];
    
    if (id == SERVO_CNT) {
      // start a new 50Hz period
      SERVO_TIM->Instance->ARR = 20000 - servo_pulse_time;
      servo_pulse_id = 0;
      servo_pulse_time = 0;
      return;
    }
  }
  
  // raising current PWM pulse
  servo_ports[id]->ODR |= servo_pins[id];

  // update PWM duty cycle
  cur = servo_pwm[id];
  set = servo_pwm_set[id];
  if (cur != set) {
    if (cur < set) {
      if ((cur+=SERVO_PWM_INC) > set)
        cur = set;
    } else {
      if ((cur-=SERVO_PWM_INC) < set)
        cur = set;
    }
    servo_pwm[id] = cur;
    servo_action = true;
  }
  
  // trigger next PWM pulse with appropriate duty cycle
  servo_pulse_time += cur;
  SERVO_TIM->Instance->ARR = cur;
}

void servo_toggle_recording(void) {
  signal(SIG_TOGGLE_RECORDING);
}

void servo_record(void) {
  signal(SIG_RECORD);
}

bool servo_is_recording(void) {
  return servo_recording;
}

uint8_t servo_record_count(void) {
  return servo_record_cnt;
}

void servo_start_replay(void) {
  signal(SIG_START_REPLAY);
}

void servo_stop_replay(void) {
  signal(SIG_STOP_REPLAY);
}

bool servo_play_sequence(const int16_t (*sequence)[SERVO_CNT], size_t length, uint32_t delay) {
  if (!lock(100)) {
    return false;
  }
  
  if (servo_sequence != NULL || servo_replay_started) {
    unlock();
    return false;
  }
  
  servo_sequence = sequence;
  servo_sequence_length = length;
  servo_sequence_step = 0;
  servo_sequence_delay = delay;
  
  unlock();
  return true;
}

bool servo_sequence_finished(void) {
  return servo_sequence == NULL;
}

///////////////////////////////////////////////////////////////////////////////
// The servo event handling

#define MAGIC 0130

static void do_stop_replay(void);

static bool load_records(void) {
  uint8_t buf[2];
  
  // read magic and count
  if (read_store(0, buf, 2) != HAL_OK)
    return false;
  if (buf[0]!=MAGIC || buf[1]>SERVO_MAX_RECORDS)
    return false;
  
  // load servo records
  uint8_t cnt = buf[1];
  uint16_t size = cnt * sizeof(*servo_records);
  if (read_store(2, (uint8_t*)servo_records, size) != HAL_OK)
    return false;
  
  servo_record_cnt = cnt;
  return true;
}

static bool store_records(void) {
  uint8_t buf[2];
  
  // write magic and count
  buf[0] = MAGIC;
  buf[1] = servo_record_cnt;
  if (write_store(0, buf, 2, 100) != HAL_OK)
    return false;
  
  // write servo records
  uint16_t size = servo_record_cnt * sizeof(*servo_records);
  if (write_store(2, (uint8_t*)servo_records, size, 100) != HAL_OK)
    return false;
  
  return true;
}

static void do_toggle_recording() {
  // stop current replay before recording
  do_stop_replay();
  
  if (servo_recording) {
    if (servo_record_cnt != 0)
      store_records();
    servo_recording = false;
  } else {
    servo_recording = true;
    servo_record_cnt = 0;
  }
}

static void do_record(void) {
  if (servo_recording && servo_record_cnt<SERVO_MAX_RECORDS) {
    for (int i=0; i<SERVO_CNT; i++)
      servo_records[servo_record_cnt][i] = servo_pwm[i];
    servo_record_cnt++;
  }
}

static void do_start_replay(void) {
  if (!servo_replay_started && !servo_recording && servo_record_cnt>0) {
    servo_replay_started = true;
    servo_replay_step = 0;
  }
}

static void do_stop_replay(void) {
  servo_replay_started = false;
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
      if (angle>=0 && angle<=1800)
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
      if (evt.value.signals & SIG_TOGGLE_RECORDING) {
        do_toggle_recording();
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
