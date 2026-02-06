/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define RADIO_NRST_Pin GPIO_PIN_13
#define RADIO_NRST_GPIO_Port GPIOC
#define RADIO_CS_Pin GPIO_PIN_14
#define RADIO_CS_GPIO_Port GPIOC
#define RADIO_INT_Pin GPIO_PIN_15
#define RADIO_INT_GPIO_Port GPIOC
#define CONT_4_Pin GPIO_PIN_0
#define CONT_4_GPIO_Port GPIOC
#define CONT_3_Pin GPIO_PIN_3
#define CONT_3_GPIO_Port GPIOC
#define RADIO_DIO3_Pin GPIO_PIN_4
#define RADIO_DIO3_GPIO_Port GPIOA
#define CONT_2_Pin GPIO_PIN_6
#define CONT_2_GPIO_Port GPIOA
#define CONT1_Pin GPIO_PIN_4
#define CONT1_GPIO_Port GPIOC
#define SPI1_CS_Pin GPIO_PIN_0
#define SPI1_CS_GPIO_Port GPIOB
#define CONT_YN_4_Pin GPIO_PIN_7
#define CONT_YN_4_GPIO_Port GPIOE
#define CONT_YN_3_Pin GPIO_PIN_8
#define CONT_YN_3_GPIO_Port GPIOE
#define SPI4_INT_Pin GPIO_PIN_10
#define SPI4_INT_GPIO_Port GPIOE
#define SPI4_CS_Pin GPIO_PIN_11
#define SPI4_CS_GPIO_Port GPIOE
#define NRST_GPS_Pin GPIO_PIN_15
#define NRST_GPS_GPIO_Port GPIOE
#define RADIO_DIO4_Pin GPIO_PIN_12
#define RADIO_DIO4_GPIO_Port GPIOB
#define RADIO_DIO5_Pin GPIO_PIN_13
#define RADIO_DIO5_GPIO_Port GPIOB
#define CONT_YN_2_Pin GPIO_PIN_14
#define CONT_YN_2_GPIO_Port GPIOB
#define PY4_Pin GPIO_PIN_15
#define PY4_GPIO_Port GPIOB
#define PY3_Pin GPIO_PIN_8
#define PY3_GPIO_Port GPIOD
#define PY2_Pin GPIO_PIN_9
#define PY2_GPIO_Port GPIOD
#define PY1_Pin GPIO_PIN_10
#define PY1_GPIO_Port GPIOD
#define BUZZER_Pin GPIO_PIN_14
#define BUZZER_GPIO_Port GPIOD
#define SPI2_CS_Pin GPIO_PIN_15
#define SPI2_CS_GPIO_Port GPIOD
#define I2C_3_INT_Pin GPIO_PIN_8
#define I2C_3_INT_GPIO_Port GPIOC
#define CONT_YN_1_Pin GPIO_PIN_10
#define CONT_YN_1_GPIO_Port GPIOA
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define SPI3_INT_Pin GPIO_PIN_2
#define SPI3_INT_GPIO_Port GPIOD
#define GPS_TIMEPULSE_Pin GPIO_PIN_4
#define GPS_TIMEPULSE_GPIO_Port GPIOD
#define RADIO_DIO2_Pin GPIO_PIN_6
#define RADIO_DIO2_GPIO_Port GPIOD
#define RADIO_DIO1_Pin GPIO_PIN_7
#define RADIO_DIO1_GPIO_Port GPIOD
#define I2C1_INT_Pin GPIO_PIN_0
#define I2C1_INT_GPIO_Port GPIOE
#define I2C2_INT_Pin GPIO_PIN_1
#define I2C2_INT_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
