/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_RED_Pin GPIO_PIN_13
#define LED_RED_GPIO_Port GPIOC
#define LED_GREEN_Pin GPIO_PIN_14
#define LED_GREEN_GPIO_Port GPIOC
#define LED_BLUE_Pin GPIO_PIN_15
#define LED_BLUE_GPIO_Port GPIOC
#define MotorCurrent_Pin GPIO_PIN_0
#define MotorCurrent_GPIO_Port GPIOC
#define ADC_ODR1_Pin GPIO_PIN_1
#define ADC_ODR1_GPIO_Port GPIOC
#define ADC_ODR2_Pin GPIO_PIN_2
#define ADC_ODR2_GPIO_Port GPIOC
#define MotorVoltage1_Pin GPIO_PIN_0
#define MotorVoltage1_GPIO_Port GPIOA
#define MotorVoltage2_Pin GPIO_PIN_1
#define MotorVoltage2_GPIO_Port GPIOA
#define SoundLevel_Pin GPIO_PIN_2
#define SoundLevel_GPIO_Port GPIOA
#define ADC_RDY_Pin GPIO_PIN_3
#define ADC_RDY_GPIO_Port GPIOA
#define ADC_PDOWN_Pin GPIO_PIN_4
#define ADC_PDOWN_GPIO_Port GPIOA
#define ADC_PGA2_Pin GPIO_PIN_4
#define ADC_PGA2_GPIO_Port GPIOC
#define ADC_PGA1_Pin GPIO_PIN_5
#define ADC_PGA1_GPIO_Port GPIOC
#define SPI_CS1_Pin GPIO_PIN_0
#define SPI_CS1_GPIO_Port GPIOB
#define SPI_CS2_Pin GPIO_PIN_1
#define SPI_CS2_GPIO_Port GPIOB
#define ESTOP_Pin GPIO_PIN_2
#define ESTOP_GPIO_Port GPIOB
#define ESTOP_EXTI_IRQn EXTI2_IRQn
#define IO1_Pin GPIO_PIN_12
#define IO1_GPIO_Port GPIOB
#define TEMP_CS_Pin GPIO_PIN_13
#define TEMP_CS_GPIO_Port GPIOB
#define SERVO_CTRL_Pin GPIO_PIN_6
#define SERVO_CTRL_GPIO_Port GPIOC
#define MOTOR_PWM1_Pin GPIO_PIN_7
#define MOTOR_PWM1_GPIO_Port GPIOC
#define MOTOR_PWM2_Pin GPIO_PIN_8
#define MOTOR_PWM2_GPIO_Port GPIOC
#define IO2_Pin GPIO_PIN_9
#define IO2_GPIO_Port GPIOA
#define POT_CS_Pin GPIO_PIN_10
#define POT_CS_GPIO_Port GPIOA
#define ADC_SPI_SCK_Pin GPIO_PIN_10
#define ADC_SPI_SCK_GPIO_Port GPIOC
#define ADC_SPI_MISO_Pin GPIO_PIN_11
#define ADC_SPI_MISO_GPIO_Port GPIOC
#define IO3_Pin GPIO_PIN_6
#define IO3_GPIO_Port GPIOB
#define IO4_Pin GPIO_PIN_7
#define IO4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
