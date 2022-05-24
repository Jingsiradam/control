/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t ubuf[128];
#define INFO(...) HAL_UART_Transmit(&huart4,\
										 (uint8_t *)ubuf,\
										 sprintf((char *)ubuf,__VA_ARGS__),\
										 0xffff)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
unsigned char buf[28];
union speed
{
    float speed;
    unsigned char data[4];
} motorspeed[6];

uint8_t buf_jy901[55];
const unsigned char header[2] = { 0x55, 0xaa };
const unsigned char ender[2] = { 0x0d, 0x0a };

void motor_test()
{	 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,60);
	
	 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	 HAL_Delay(1000);
	 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	 HAL_Delay(500);
	 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,60);
	 HAL_Delay(1000);
	 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,60);
	 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
	 HAL_Delay(1000);
	 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
	 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
	 HAL_Delay(500);
	 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
	 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,60);
	 HAL_Delay(1000);
	 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
	 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
	 
	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,60);
	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,0);
	 HAL_Delay(1000);
	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);
	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,0);
	 HAL_Delay(500);
	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);
	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,60);
	 HAL_Delay(1000);
	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);
	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,0);
	 
	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,60);
	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,0);
	 HAL_Delay(1000);
	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);
	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,0);
	 HAL_Delay(500);
	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);
	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,60);
	 HAL_Delay(1000);
	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);
	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,0);
	 
	 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,60);
	 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
	 HAL_Delay(1000);
	 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
	 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
	 HAL_Delay(500);
	 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
	 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,60);
	 HAL_Delay(1000);
	 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
	 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
	 
	 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,60);
	 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);
	 HAL_Delay(1000);
	 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
	 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);
	 HAL_Delay(500);
	 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
	 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,60);
	 HAL_Delay(1000);
	 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
	 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);
}




void set_motorspeed(float speed,uint8_t num)
{ switch(num)
	{case 3:
		if(speed>0)
		{__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,speed);
		 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
		}
		else
		{__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
		 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,-speed);
		}
	case 2:
		if(speed>0)
		{__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,speed);
		 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
		}
		else
		{__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
		 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,-speed);
		}
	case 1:
		if(speed>0)
		{__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,speed);
		 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,0);
		}
		else
		{__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);
		 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,-speed);
		}
	case 6:
		if(speed>0)
		{__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,speed);
		 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,0);
		}
		else
		{__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);
		 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,-speed);
		}
	case 5:
		if(speed>0)
		{__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,speed);
		 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
		}
		else
		{__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
		 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,-speed);
		}
	case 4:
		if(speed>0)
		{__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,speed);
		 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);
		}
		else
		{__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
		 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,-speed);
		}
	}
}

void motor_start()
{
	for(size_t i=0;i<6;i++)
	{set_motorspeed(motorspeed[i].speed,i+1);}
}
void PWM_init()
{	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//right 1 motor 1
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);//right 2  motor 2
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);//left 3  motor 3
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);//left 2  motor4
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);// error motor
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == &huart4)
  {
	  INFO("success!");
  if (buf[0]==header[0]&&buf[1]==header[1])
{		
		for(size_t i =0;i<4;i++)
{		 motorspeed[0].data[i]=buf[i + 2];  
         motorspeed[1].data[i]=buf[i + 6]; 
		 motorspeed[2].data[i]=buf[i + 10]; 
		 motorspeed[3].data[i]=buf[i + 14];
		 motorspeed[4].data[i]=buf[i + 18];
		 motorspeed[5].data[i]=buf[i + 22];
		}

	}

//	HAL_UART_Receive_IT(&huart4,buf,sizeof(buf));
  }
//	if(huart == &huart5)
//	{
//		HAL_UART_Transmit(&huart4,buf_jy901,sizeof(buf_jy901),0xffff);
//	}
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM7_Init();
  MX_UART5_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_DMA_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  PWM_init();
  
//  HAL_UART_Receive_IT(&huart4,buf,sizeof(buf));
  motor_test();
  HAL_UART_Receive_DMA(&huart4,buf,sizeof(buf));

//  HAL_UART_Receive_DMA(&huart5,buf_jy901,sizeof(buf_jy901));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE BEGIN 2 */
	  motor_start();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

#pragma pack()

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
