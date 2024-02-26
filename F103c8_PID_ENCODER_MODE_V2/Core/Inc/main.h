/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define TIM2_CH1_ENCODER_C1_Pin GPIO_PIN_0
#define TIM2_CH1_ENCODER_C1_GPIO_Port GPIOA
#define TIM2_CH2_ENCODER_C2_Pin GPIO_PIN_1
#define TIM2_CH2_ENCODER_C2_GPIO_Port GPIOA
#define TIM1_CH1_ENCODER_C1_Pin GPIO_PIN_8
#define TIM1_CH1_ENCODER_C1_GPIO_Port GPIOA
#define TIM1_CH2_ENCODER_C2_Pin GPIO_PIN_9
#define TIM1_CH2_ENCODER_C2_GPIO_Port GPIOA
#define TIM4_CH1_MOTOR_1_L_Pin GPIO_PIN_6
#define TIM4_CH1_MOTOR_1_L_GPIO_Port GPIOB
#define TIM4_CH2_MOTOR_1_R_Pin GPIO_PIN_7
#define TIM4_CH2_MOTOR_1_R_GPIO_Port GPIOB
#define TIM4_CH1_MOTOR_2_L_Pin GPIO_PIN_8
#define TIM4_CH1_MOTOR_2_L_GPIO_Port GPIOB
#define TIM4_CH4_MOTOR_2_R_Pin GPIO_PIN_9
#define TIM4_CH4_MOTOR_2_R_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
