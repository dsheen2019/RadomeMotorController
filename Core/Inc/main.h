/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "constants.h"
#include "stdint.h"
#include "stdbool.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define AZIMUTH
//#define ELEVATION

#ifdef AZIMUTH
#ifdef MOTOR_DEFINED
#error Multiple motor definition!
#endif
#define MOTOR_DEFINED

#define ID_STR "AZIMUTH"

#define OFFSET 0x90000000
#define ID 1

#define GEAR_REDUCTION 488.7f
#define CAN_ID 0x10

#endif

#ifdef ELEVATION
#ifdef MOTOR_DEFINED
#error Multiple motor definition!
#endif
#define MOTOR_DEFINED

#define ID_STR "ELEVATION"

#define OFFSET 0xa2000000
#define ID 2

#define GEAR_REDUCTION 1078.21f
#define CAN_ID 0x20

#endif

#ifndef MOTOR_DEFINED
#error No motor defined!
#endif

#define MAX_MOTOR_SLEW 40.0f
#define MAX_MOTOR_VELOCITY 200.0f
//#define MAX_MOTOR_VELOCITY 50.0f
#define VELOCITY_LSB ((3.0f * PI) / (3600.0f * 180.0f)) // 3 arcseconds/s, max speed is about 27 dps, earth rotation is 5 LSB
#define CURRENT_LSB 0.001f // LSB is 1 mA

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
#define ADC_SAMPLES 4
#define CLOCK_FREQ 170000000
#define DEAD_TIME 140
#define ADC_SEQ_LEN 2
#define PWM_ADC_TRIG (ADC_CLK_SCALE * ADC_SEQ_LEN * ADC_SAMPLES / 2)
#define PWM_FREQ 20000
#define ADC_CLK_SCALE (3 * 25)
#define PWM_PERIOD (CLOCK_FREQ / (PWM_FREQ * 2))
#define ADC_SW_OFFSET 180
#define SOC_Pin GPIO_PIN_0
#define SOC_GPIO_Port GPIOA
#define SOB_Pin GPIO_PIN_1
#define SOB_GPIO_Port GPIOA
#define SOA_Pin GPIO_PIN_2
#define SOA_GPIO_Port GPIOA
#define VBUS_SENSE_Pin GPIO_PIN_3
#define VBUS_SENSE_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
