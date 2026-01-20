/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "VL53L1X_api.h"
#include "Ibus.h"
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
/* Kod do sensora*/
#define sensor_adress (0x52 << 1) //adres 7 bitowy przesuniety w lewo na potrzeby 8 bitowego adresu obslugiwanego w bibliotece hal
uint8_t data[2];
uint16_t distance;

uint8_t sense_booted = 0;
uint8_t dataReady = 0;
uint16_t distance_mm;
VL53L1X_ERROR status=1;

#define sensor_adress 0x52 // 0x29 << 1
uint16_t sensorId = 0;
uint16_t distance;

//kod do RXTX
static uint8_t tx_busy = 0;
volatile uint8_t ibus_tx_ready = 1;
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // Debug Blink
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
  HAL_Delay(2000);
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
  HAL_Delay(2000);
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
  HAL_Delay(2000);
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);

  //VL53_InitRegisters();
  /* Kod do RX*/
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)ibus_rx_buf, IBUS_FRAME_LEN);

  /* Kod do sensora */
    while (sense_booted == 0) //Czekaj aż skończy się bootowanie sensora
    {
        status = VL53L1X_BootState(sensor_adress, &sense_booted);
        if (status != 0) {
            // Communication error - stay in loop or signal error
            HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
            HAL_Delay(100);
            continue;
        }
        HAL_Delay(2);
    }

    if (sense_booted) {
        status = VL53L1X_SensorInit(sensor_adress);
        status |= VL53L1X_GetSensorId(sensor_adress, &sensorId); // Powinno być 0xEEAC

        status |= VL53L1X_SetDistanceMode(sensor_adress, 2);      // 1 = Short, 2 = Long
        status |= VL53L1X_SetTimingBudgetInMs(sensor_adress, 200); // 15–500 ms
        status |= VL53L1X_SetInterMeasurementInMs(sensor_adress, 400);
        status |= VL53L1X_StartRanging(sensor_adress); //Zacznij mierzyc
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 1. Process new RX frame if available
    if (ibus_rx_ready)
    {
        ibus_rx_ready = 0;
        // Data is now in ibus_ch_rx
    }

    // 2. Decide between Passthrough and Manual Control
    // Let's use Channel 7 (AUX1) as the switch. Usually > 1800 is HIGH
    if (ibus_ch_rx[6] > 1800)
    {
        // MANUAL OVERRIDE MODE (from code)
        // Keep other channels as they are
        for (int i = 0; i < IBUS_CHANNELS; i++) {
        	ibus_ch_tx[i] = ibus_ch_rx[i];
        }

        // Specifically set the values you want to test
        // Example: Set throttle (ch2) to 1800
        ibus_ch_tx[2] = 1800;
        // Or call ibus_test() which ramp up all channels
        // ibus_test();
    }
    else
    {
        // PASSTHROUGH MODE (normal operation)
        for (int i = 0; i < IBUS_CHANNELS; i++)
        {
            ibus_ch_tx[i] = ibus_ch_rx[i];
        }
    }

    // 3. Build & send every 7ms
    static uint32_t last_tx_time = 0;
    uint32_t now = HAL_GetTick();

    if (now - last_tx_time >= 7)
    {
        // 4. Debug: Toggle LED to show we are trying to send
        HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);

        ibus_build();

        // Use standard Transmit (polling) for now to be 100% sure it works.
        // 32 bytes at 115200 takes 2.7ms, which is fine in a 7ms window.
        if (HAL_UART_Transmit(&huart2, (uint8_t*)ibus_frame, IBUS_FRAME_LEN, 10) == HAL_OK)
        {
            last_tx_time = now;
        }

        // Clear any UART errors that might have locked the peripheral (Overrun, etc)
        __HAL_UART_CLEAR_OREFLAG(&huart1);
        __HAL_UART_CLEAR_OREFLAG(&huart2);
    }

    VL53L1X_CheckForDataReady(sensor_adress, &dataReady);
    if (dataReady != 0)
    {
        dataReady = 0;
        VL53L1X_GetRangeStatus(sensor_adress, &status);
        if(status==0)
        {
            VL53L1X_GetDistance(sensor_adress, &distance_mm);
            VL53L1X_ClearInterrupt(sensor_adress);
        }
    }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* Kod do RX*/
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//    if (htim->Instance == TIM3)
//    {
//        HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
//
//        //ibus_test();
//        if(ibus_rx_ready)
//        {
//        	ibus_rx_ready=0;
//        	if (ibus_ch[4] > 1500)
//        	            {
//        	                // TEST MODE
//        	                ibus_test();
//        	            }
//        }
//        ibus_build();
//
//        if (!tx_busy)
//        {
//            tx_busy = 1;
//            HAL_UART_Transmit_DMA(&huart2, ibus_frame, IBUS_FRAME_LEN);
//        }
//    }
//}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        // iBUS frames are always 32 bytes and start with 0x20 0x40
        if (Size == IBUS_FRAME_LEN && ibus_rx_buf[0] == 0x20 && ibus_rx_buf[1] == 0x40)
        {
            ibus_decode((uint8_t*)ibus_rx_buf, (uint16_t*)ibus_ch_rx);
            ibus_rx_ready = 1;
        }

        // Restart reception to idle
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)ibus_rx_buf, IBUS_FRAME_LEN);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	        ibus_tx_ready = 1;
}

void ibus_test(void);
void ibus_decode(const uint8_t *b, uint16_t *ch);
void ibus_build(void);

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
