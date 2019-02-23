/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "cmsis_os.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BEEP_Pin GPIO_PIN_13
#define BEEP_GPIO_Port GPIOC
#define E1_Pin GPIO_PIN_15
#define E1_GPIO_Port GPIOC
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOC
#define E2_Pin GPIO_PIN_1
#define E2_GPIO_Port GPIOC
#define E6_Pin GPIO_PIN_2
#define E6_GPIO_Port GPIOC
#define ADC_BAT_Pin GPIO_PIN_3
#define ADC_BAT_GPIO_Port GPIOC
#define M1N_Pin GPIO_PIN_4
#define M1N_GPIO_Port GPIOA
#define M1P_Pin GPIO_PIN_5
#define M1P_GPIO_Port GPIOA
#define M2N_Pin GPIO_PIN_6
#define M2N_GPIO_Port GPIOA
#define M2P_Pin GPIO_PIN_7
#define M2P_GPIO_Port GPIOA
#define PS2_CMD_Pin GPIO_PIN_5
#define PS2_CMD_GPIO_Port GPIOC
#define PS2_DATA_Pin GPIO_PIN_0
#define PS2_DATA_GPIO_Port GPIOB
#define TRIG_Pin GPIO_PIN_1
#define TRIG_GPIO_Port GPIOB
#define ECHO_Pin GPIO_PIN_12
#define ECHO_GPIO_Port GPIOB
#define ECHO_EXTI_IRQn EXTI15_10_IRQn
#define PS2_CS_Pin GPIO_PIN_14
#define PS2_CS_GPIO_Port GPIOB
#define PS2_CLK_Pin GPIO_PIN_15
#define PS2_CLK_GPIO_Port GPIOB
#define E3_Pin GPIO_PIN_9
#define E3_GPIO_Port GPIOC
#define SERVO1_Pin GPIO_PIN_10
#define SERVO1_GPIO_Port GPIOC
#define SERVO2_Pin GPIO_PIN_11
#define SERVO2_GPIO_Port GPIOC
#define SERVO3_Pin GPIO_PIN_12
#define SERVO3_GPIO_Port GPIOC
#define SERVO4_Pin GPIO_PIN_2
#define SERVO4_GPIO_Port GPIOD
#define SPK_Pin GPIO_PIN_5
#define SPK_GPIO_Port GPIOB
#define WS2812_Pin GPIO_PIN_8
#define WS2812_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_9
#define LED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
