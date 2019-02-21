#include "main.h"
#include "cmsis_os.h"
#include "servo.h"
#include "storage.h"

///////////////////////////////////////////////////////////////////////////////
// Definitions

#define SERVO_DEG_INC     20
#define SERVO_DELAY       20

/* Convert degree to PWM duty cycle */
#define DEG2PWM(d)  (((d)*2000+900)/1800+500)

///////////////////////////////////////////////////////////////////////////////
// Servo rotate degrees

static const uint16_t servo_deg_init[] = {
  900, 900, 500, 0, 600, 900
};

static __IO uint16_t servo_deg[SERVO_CNT];
static __IO uint16_t servo_deg_set[SERVO_CNT];

static __IO bool servo_action; /* servo movement is in progress */

static void update_servo_data(void);

#if PCA9685

///////////////////////////////////////////////////////////////////////////////
// PCA9685 Driver

#include "i2c.h"
#include "delay.h"

#define PCA9685_BASE_ADDRESS  0x80

#define PCA9685_MODE1_REG     0x00
#define PCA9685_MODE2_REG     0x01
#define PCA9685_PWM_REG       0x06 // Start of PWM regs, 4B per reg, 2B on phase, 2B off phase, little-endian
#define PCA9685_PRESCALE_REG  0xFE

#define PCA9685_MODE_RESTART  0x80
#define PCA9685_MODE_EXTCLK   0x40
#define PCA9685_MODE_AUTOINC  0x20
#define PCA9685_MODE_SLEEP    0x10
#define PCA9685_MODE_SUBADR1  0x08
#define PCA9685_MODE_SUBADR2  0x04
#define PCA9685_MODE_SUBADR3  0x02
#define PCA9685_MODE_ALLCALL  0x01

#define PCA9685_MODE_INVRT          0x10  // Inverts polarity of channel output signal
#define PCA9685_MODE_OUTPUT_ONACK   0x08  // Channel update happens upon ACK (post-set) rather than on STOP (endTransmission)
#define PCA9685_MODE_OUTPUT_TPOLE   0x04  // Use a totem-pole (push-pull) style output, typical for boards using this chipset
#define PCA9685_MODE_OUTNE_HIGHZ    0x02  // For active low output enable, sets channel output to high-impedance state
#define PCA9685_MODE_OUTNE_LOW      0x01  // Similarly, sets channel output to high if in totem-pole mode, otherwise high-impedance state

#define SERVO_FREQ_PRESCALE   122   // in 50Hz

static const uint8_t servo_channels[] = {
  PCA9685_PWM_REG + 13*4,
  PCA9685_PWM_REG + 12*4,
  PCA9685_PWM_REG + 11*4,
  PCA9685_PWM_REG + 10*4,
  PCA9685_PWM_REG +  9*4,
  PCA9685_PWM_REG +  8*4
};

static void pca9685_write_reg(uint8_t address, uint8_t value) {
  I2C_WriteMem(PCA9685_BASE_ADDRESS, address, 1, &value, 1);
}

static void pca9685_init(void) {
  // The PRE_SCALE register can only be set when the SLEEP bit of MODE1 register is set to logic 1.
  pca9685_write_reg(PCA9685_MODE1_REG, PCA9685_MODE_SLEEP);
  
  // Set PWM frequency in 50Hz
  pca9685_write_reg(PCA9685_PRESCALE_REG, SERVO_FREQ_PRESCALE);
  
  // Restart PCA9685
  pca9685_write_reg(PCA9685_MODE1_REG, PCA9685_MODE_RESTART | PCA9685_MODE_AUTOINC);
  pca9685_write_reg(PCA9685_MODE2_REG, PCA9685_MODE_OUTPUT_TPOLE);

  // It takes 500us max for the oscillator to be up and running once SLEEP bit has been set to logic 0.
  HAL_Delay_us(500);
}

#define PWM_MIN 102 /* 2.5% duty cycle, 500us in 50Hz */
#define PWM_MAX 512 /* 12.5% duty cycle, 2500us in 50Hz */

static void pca9685_set_pwm(uint8_t id, uint16_t angle) {
  int width = PWM_MIN + ((int)angle*(PWM_MAX-PWM_MIN)+900)/1800;
  int begin = id*4096/SERVO_CNT; // distribute pulse over entire period to balance load
  int end   = begin + width;
  
  if (end > 4096) {
    end = 4096;
    begin = end - width;
  }
  
  uint8_t data[4] = {
    (uint8_t)(begin & 0xFF),
    (uint8_t)((begin>>8) & 0xFF),
    (uint8_t)(end & 0xFF),
    (uint8_t)((end>>8) & 0xFF)
  };
  
  I2C_WriteMem(PCA9685_BASE_ADDRESS, servo_channels[id], 1, data, 4);
}

#else

///////////////////////////////////////////////////////////////////////////////
// GPIO PWM Driver

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

static uint8_t  servo_pulse_id;
static uint16_t servo_pulse_time;

void servo_pwm_pulse(void) {
  uint8_t id = servo_pulse_id++;

  if (id > 0) {
    // lower previous PWM pulse
    servo_ports[id-1]->ODR &= ~servo_pins[id-1];
    
    // start next 50Hz period
    if (id == SERVO_CNT) {
      SERVO_TIM->Instance->ARR = 20000 - servo_pulse_time;
      servo_pulse_id = 0;
      servo_pulse_time = 0;
      update_servo_data();
      return;
    }
  }
  
  // raising current PWM pulse
  servo_ports[id]->ODR |= servo_pins[id];
 
  // trigger next PWM pulse with appropriate duty cycle
  uint16_t pwm = (uint16_t)DEG2PWM(servo_deg[id]);
  servo_pulse_time += pwm;
  SERVO_TIM->Instance->ARR = pwm;
}

#endif

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
    servo_deg[i]     = servo_deg_init[i];
    servo_deg_set[i] = servo_deg_init[i];
  }
  
  load_records();
  
#if PCA9685
  pca9685_init();
  for (int i=0; i<SERVO_CNT; i++) {
    pca9685_set_pwm(i, servo_deg[i]);
  }
#else
  HAL_TIM_Base_Start_IT(SERVO_TIM);
#endif
}

void servo_reset(void) {
  for (int i=0; i<SERVO_CNT; i++) {
    servo_deg_set[i] = servo_deg_init[i];
  }
}

bool servo_set(uint8_t id, int angle) {
  if (id==0 || id>SERVO_CNT)
    return false;

  if (angle < 0)
    angle = 0;
  if (angle > 1800)
    angle = 1800;

  servo_deg_set[id-1] = (uint16_t)angle;
  return true;
}

bool servo_add(uint8_t id, int delta) {
  return servo_set(id, servo_get(id) + delta);
}

int servo_get(uint8_t id) {
  if (id==0 || id>SERVO_CNT)
    return 0;
  return (int)servo_deg[id-1];
}

bool servo_in_action(void) {
  return servo_action;
}

static void update_servo_data(void) {
  int diff, steps;

  servo_action = false;

  // compute maximum angle delta
  diff = 0;
  for (int i=0; i<SERVO_CNT; i++) {
    int d = servo_deg_set[i] - servo_deg[i];
    if (d < 0)
      d = -d;
    if (d > diff)
      diff = d;
  }
  if (diff == 0) {
    // no servo need to update
    return;
  }
  
  // compute angle change steps to smoothly update all servos
  steps = (diff+SERVO_DEG_INC-1)/SERVO_DEG_INC;

  // update individual servo pwm data
  for (int i=0; i<SERVO_CNT; i++) {
    int cur = servo_deg[i];
    int set = servo_deg_set[i];
    if (cur != set) {
      int new = cur + (set-cur+steps-1)/steps;
      if (cur<set ? new>set : new<set)
        new = set;
      servo_deg[i] = new;
      servo_action = true;
#if PCA9685
      pca9685_set_pwm(i, new);
#endif
    }
  }
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
      servo_records[servo_record_cnt][i] = servo_deg[i];
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
    servo_deg_set[i] = servo_records[servo_replay_step][i];
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
  uint32_t delay = 0;
  
  for (;;) {
    osEvent evt = osSignalWait(0xFFFF, SERVO_DELAY);
    update_servo_data();
    
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

    if (delay != 0)
      delay--;
    if (servo_action || delay)
      continue;
    
    if (servo_replay_started) {
      do_replay();
      delay = 5;
    } else if (servo_sequence != NULL) {
      do_play_sequence();
      delay = servo_sequence_delay / SERVO_DELAY;
    }
  }
}
