#ifndef __SERVO_H
#define __SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define SERVO_CNT           6

#define SERVO_PAW           1
#define SERVO_WRIST_ROT     2
#define SERVO_WRIST_UD      3
#define SERVO_ELBOW_UD      4
#define SERVO_SHOULDER_UD   5
#define SERVO_SHOULDER_ROT  6

#define SERVO_MAX_RECORDS  20

/**
 * @brief  Initialize servo control
 */
void servo_init(void);

/**
 * @brief  Set a value to the servo, controlling the shaft accordingly.
 * @param  id: The servo ID (1..6)
 * @param  angle: the value to write to the servo, from 0 to 180
 * @retval 1 for success, 0 for invalid servo ID
 */
bool servo_set(uint8_t id, int angle);

/**
 * @brief  Add a step value to the servo, controlling the shaft accordingly.
 * @param  id: The servo ID (1..6)
 * @param  inc: The angle increment, may be negative
 * @retval 1 for success, 0 for invalid servo ID
 */
bool servo_step(uint8_t id, int inc);

/**
 * @brief  Get the current angle of the servo (the value passed to the last call to servo_set()).
 * @param  id: The servo ID (1..6)
 * @retval The angle of the servo, from 0 to 180 degrees.
 */
int servo_get(uint8_t id);

/**
 * @brief  Determine whether the servo is in action.
 * @retval 1 if servo in action, 0 otherwise.
 */
bool servo_in_action(void);

/**
 * @brief  Reset all servo to initial angles
 */
void servo_reset(void);

void servo_toggle_recording(void);
void servo_record(void);
bool servo_is_recording(void);
uint8_t servo_record_count(void);

void servo_start_replay(void);
void servo_stop_replay(void);

bool servo_play_sequence(const int16_t (*sequence)[SERVO_CNT], size_t length, uint32_t delay);
bool servo_sequence_finished(void);

// Private
#ifndef PCA9685
#define PCA9685 1
#endif

#if !PCA9685
extern TIM_HandleTypeDef htim2;
#define SERVO_TIM (&htim2)
void servo_pwm_pulse(void);
#endif

#ifdef __cplusplus
}
#endif

#endif // __SERVO_H
