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
#include "fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "VL53L1X_api.h"
#include "Ibus.h"
#include <stdio.h>
#include <string.h>
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

/*Sensor*/
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

//Regulator
uint32_t elapsed_ms = 0;
int32_t descent_mm = 0;
uint8_t  autoland_active = 0;
uint16_t hover_start = 1500;   // captured hover throttle
uint32_t autoland_start_time = 0;
uint16_t autoland_start_alt_mm = 0;
uint16_t desired_alt_mm = 0;
float Kp_alt = 1.f;   // proportional gain (safe starting value)
float ch5_norm = 0.f;
static uint8_t prev_autoland_switch = 0;
int32_t target = 0;
uint8_t autoland_switch = 0;
int32_t error_mm = 0;
int32_t throttle_cmd = 0;

//SD
FATFS SDFatFs;
FIL logFile;
UINT bytesWritten;
char logBuf[64];
uint8_t sd_ready = 0;
static uint32_t last_sync = 0;
FRESULT res;
static uint8_t log_div = 0;
int sd_len = 0;


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
  MX_SPI1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  // Debug Blink
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
  HAL_Delay(2000);
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
  HAL_Delay(2000);
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
  HAL_Delay(2000);
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);

  if (f_mount(&SDFatFs, "", 1) == FR_OK)
  {
      if (f_open(&logFile, "log.csv", FA_OPEN_ALWAYS | FA_WRITE) == FR_OK)
      {
          f_lseek(&logFile, f_size(&logFile)); // append
          sd_ready = 1;
      }
  }
  if (sd_ready && f_size(&logFile) == 0)
  {
      strcpy(logBuf, "time_ms,distance_mm\r\n");
      sd_len = strlen(logBuf);
      res = f_write(&logFile, logBuf, sd_len, &bytesWritten);
      if (res != FR_OK || bytesWritten != sd_len)
      {
          sd_ready = 0; // disable logging safely
      }
  }
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

    for (int i = 0; i < IBUS_CHANNELS; i++)
            {
                ibus_ch_tx[i] = ibus_ch_rx[i];
            }

    autoland_switch = (ibus_ch_rx[6] > 1800);

    // Clamp for safety
    if (ibus_ch_rx[4] < 1000) ibus_ch_rx[4] = 1000;
    if (ibus_ch_rx[4] > 2000) ibus_ch_rx[4] = 2000;

    // Normalize to 0.0–1.0
    ch5_norm = (ibus_ch_rx[4] - 1000.0f) / 1000.0f;

    // Map to Kp range 0.5–1.5
    Kp_alt = 0.5f + ch5_norm* 1.0f;

    // 2. Decide between Passthrough and Auto Control

    // Rising edge: OFF → ON
    if (autoland_switch && !prev_autoland_switch)
    {
        autoland_active = 1;

        // Capture hover throttle at activation moment
        hover_start = ibus_ch_rx[2];   // throttle channel
    }

    // Falling edge: ON → OFF
    if (!autoland_switch && prev_autoland_switch)
    {
        autoland_active = 0;
    }

    prev_autoland_switch = autoland_switch;

    if (autoland_active)
    {
        elapsed_ms = HAL_GetTick() - autoland_start_time;

        descent_mm = (elapsed_ms * 100) / 1000; // 100 mm/s
        target = autoland_start_alt_mm - descent_mm;

        if (target < 300)
            target = 300;

        desired_alt_mm = (uint16_t)target;

        error_mm = (int32_t)desired_alt_mm - (int32_t)distance_mm;

        // Proportional control
        throttle_cmd = hover_start + (int32_t)(Kp_alt * error_mm);

        // Safety limits
        if (throttle_cmd > 1800) throttle_cmd = 1800;
        if (throttle_cmd < 1100) throttle_cmd = 1100;

        ibus_ch_tx[2] = throttle_cmd;
    }
    else
    {
        // Normal passthrough
        ibus_ch_tx[2] = ibus_ch_rx[2];
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
            if (sd_ready)
            {
            	if (++log_div < 5)
            	    goto skip_log;
            	log_div = 0;

            	sd_len = snprintf(logBuf, sizeof(logBuf),
            	                   "%lu,%u\r\n",
            	                   HAL_GetTick(),
            	                   distance_mm);

            	res = f_write(&logFile, logBuf, sd_len, &bytesWritten);
            	if (res != FR_OK || bytesWritten != sd_len)
            	{
            	    sd_ready = 0;
            	}

            	if (HAL_GetTick() - last_sync > 1000)
            	{
            	    f_sync(&logFile);
            	    last_sync = HAL_GetTick();
            	}

            	skip_log:;
            }
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

//komemtarz

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
	if (sd_ready)
	{
	    f_sync(&logFile);
	    f_close(&logFile);
	    f_mount(NULL, "", 0);
	}
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
