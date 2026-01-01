/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "eth.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int stan_do_svw =0;
char jeden[2] = "+";
char dwa[2] = "-";
char komunikat[] = "niepoprawny komunikat, napisz + lub - (+CR)";
uint8_t receivedChar;
char odebrane[100];
uint8_t ind = 0;
uint32_t zad3pomocnicza =0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  if(GPIO_Pin == USER_Btn_Pin){
	  HAL_TIM_Base_Start_IT(&htim14);
	  stan_do_svw=1;
  }
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
  if(GPIO_Pin == sync_Pin){
	  HAL_TIM_Base_Start_IT(&htim14);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  if (htim->Instance == TIM14){
	  HAL_TIM_Base_Stop_IT(&htim14);
	  stan_do_svw=0;
	  HAL_GPIO_WritePin(GPIOA, triac_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, triac_Pin, GPIO_PIN_RESET);

//	  if(stan_do_svw){
//		  stan_do_svw =0;
//	  }
//	  else {
//		  stan_do_svw =1;
//	  }
	  //HAL_Delay(10);

  }


  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
}

void zad1(){
	  if(__HAL_TIM_GET_FLAG(&htim14, TIM_FLAG_UPDATE)){
	  __HAL_TIM_CLEAR_FLAG(&htim14, TIM_FLAG_UPDATE);
	  HAL_GPIO_TogglePin(zewn_pin_GPIO_Port, zewn_pin_Pin);
	  stan_do_svw = HAL_GPIO_ReadPin(zewn_pin_GPIO_Port, zewn_pin_Pin);
	  }
}

void zad3(){
    // Odbierz jeden znak
    HAL_UART_Receive(&huart3, &receivedChar, 1, HAL_MAX_DELAY);

    if(receivedChar == '\r' || receivedChar == '\n')
    {
        // Koniec linii - wyślij całą linię z powrotem
        odebrane[ind] = '\0'; // Zakończ string
        HAL_UART_Transmit(&huart3, (uint8_t*)odebrane, strlen(odebrane), HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY); // Nowa linia
        ind = 0; // Resetuj indeks

    if (strcmp(jeden,odebrane)==0){
    	zad3pomocnicza += 1000;
    			if(zad3pomocnicza >= 100000){
    				zad3pomocnicza =100000;
    			}
    	__HAL_TIM_SET_AUTORELOAD(&htim14, zad3pomocnicza);
    }
    else if (strcmp(dwa,odebrane)==0){
    	zad3pomocnicza -= 1000;
    			if(zad3pomocnicza <= 1){
    				zad3pomocnicza =0;
    			}
    	__HAL_TIM_SET_AUTORELOAD(&htim14, zad3pomocnicza);
    }
    else{
        HAL_UART_Transmit(&huart3, (uint8_t*)komunikat, strlen(komunikat), HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY); // Nowa linia
    }
    }
    else
    {
        odebrane[ind++] = receivedChar; // Dodaj znak do bufora
        if(ind >= 99) ind = 0; // Zabezpieczenie przed przepełnieniem
    };
}
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
  MX_ETH_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

//HAL_TIM_Base_Start(&htim14);//do zad1
//HAL_TIM_Base_Start_IT(&htim14);//do zad2 i 3

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
zad1();
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
#ifdef USE_FULL_ASSERT
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
