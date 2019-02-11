#ifndef __SERVO_H
#define __SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define SERVO_CNT           6

#define SERVO_PAW           1
#define SERVO_WRIST_ROT     2
#define SERVO_WRIST_UD      3
#define SERVO_ELBOW_UD      4
#define SERVO_SHOULDER_UD   5
#define SERVO_SHOULDER_ROT  6

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
uint8_t servo_set(uint8_t id, int angle);

/**
 * @brief  Add a value to the servo, controlling the shaft accordingly.
 * @param  id: The servo ID (1..6)
 * @param  inc: The angle increment, may be negative
 * @retval 1 for success, 0 for invalid servo ID
 */
uint8_t servo_add(uint8_t id, int inc);

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
uint8_t servo_in_action(void);

/**
 * @brief  Reset all servo to initial angles
 */
void servo_reset(void);

void servo_start_record(void);
void servo_record(void);
void servo_start_replay(void);
void servo_stop_replay(void);

uint8_t servo_play_sequence(const int16_t (*sequence)[SERVO_CNT], size_t length, uint32_t delay);
uint8_t servo_sequence_finished(void);

// Private
extern TIM_HandleTypeDef htim2;
#define SERVO_TIM (&htim2)
#define SERVO_TIM_CHANNEL TIM_CHANNEL_1

void servo_pwm_start(void);
void servo_pwm_pulse(void);

#ifdef __cplusplus
}
#endif

#endif // __SERVO_H
