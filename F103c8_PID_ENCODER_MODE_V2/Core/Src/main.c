/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

	#include "PID.h"
	#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

	PIDController MPID;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

	/* WITH TUA = 0.044 */

		/*

				#define MKp 1.92
				#define MKi 169.6

				#define MKp 2.06
				#define MKi 119.1

				#define MKp 1.83
				#define MKi 156.7

		*/

				#define MKp 7.92
				#define MKi 136.48

				#define MKd 0.0

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

		/* PID */

				#define pi 3.1415
				#define Motor1 0

				#define CPR 911  /* COUNT PER REVOTION */
				#define Sample_time 20 // MS

				uint16_t cnt[1];
				uint16_t Enc_count[1];

				uint16_t v;
				uint16_t count[1]; /* COUNT PULSE FOR ENCODER */
				uint16_t new_count[1];
				uint8_t count_state[1];
				uint16_t diff[1]; /* DIFFENCE BETWEEN COUNT AND NEW_COUNT IN A SAMPLE TIME  */
				uint16_t c;

				float speedM[1];
				float rdps[1];

				float Motor1_speed ;
				float V1 ; /* TARGET SPEED OF MOTOR 1 */
				float pwm_M1 ;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

	/* MAP */

		float map(float Input, float Min_Input, float Max_Input, float Min_Output, float Max_Output) {
			return (float) ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output);
		}


	/* ENCODER MODE */

		float Motors_RPS(int j, float SampleTime, float N_round)
		{

			/* ENCODER MODE */

				new_count[j] = TIM1->CNT;
				count_state[j] = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1);

				if (count_state[j])
					{
						if (new_count[j] <= count[j])
						{
							diff[j] = count[j] - new_count[j]; /* CHECK FOR COUNTER UNDERFLOW */
						}
						else
						{
							diff[j] = (65536 - new_count[j]) + count[j];
						}
						speedM[j] = (float)diff[j] * 1000.0f / (N_round * SampleTime)*-1;
					}
					else
					{
						if (new_count[j] >= count[j])
						{
							diff[j] = new_count[j] - count[j]; /* CHECK FOR COUNTER OVERFLOW */
						}
						else
						{
							diff[j] = (65536 - count[j]) + new_count[j];
						}
						speedM[j] = (float)diff[j] * 1000.0f / (N_round * SampleTime);
					}

					rdps[j] = -2.0f * pi * speedM[j];
					count[j] = new_count[j];

					return rdps[j];
		}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

		/* INNITAIL PARAMET PID */

			PID_Init(&MPID, 1);
			MPID.T = 0.02;		/* T = 10MS */
			MPID.limMax = 1000;
			MPID.limMin = -1000;
			MPID.limMaxInt = 1000;
			MPID.limMinInt = -1000;
			MPID.tau = 0;	/* FOR Kd */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

		/* TIMER ENCODER MODE */

			 HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
			 HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

		/* TIMER */

			HAL_TIM_Base_Start_IT(&htim3);

		/* TIMER 4 MOTOR */

			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

		void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
			if (htim->Instance == TIM3) {

				/* TEST SPEED MOTOR */

					/*

						Motor1_speed = Motors_RPS(Motor1, Sample_time, CPR);

						 TIM4->CCR1 = V1;
						 TIM4->CCR2 = 0;

					*/

				/* PID */

					/* PID NEED TO CHANGE FOR USING */

						pwm_M1 = PID(&MPID, V1, Motor1_speed, MKp, MKi, MKd, Motor1);

					/* MEASURMENT FEEDBACK SPEED */

						Motor1_speed = Motors_RPS(Motor1, Sample_time, CPR);

						if (V1 > 0) {
							TIM4->CCR1 = pwm_M1;
							TIM4->CCR2 = 0
									;
						}
						else if(V1 < 0) {
							TIM4->CCR1 = 0;
							TIM4->CCR2 = -1 * pwm_M1;
						}
						else
						{
							TIM4->CCR2 = 0;
							TIM4->CCR1 = 0;
						}
				}
		}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
