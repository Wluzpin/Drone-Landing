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
#include "spi.h"
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
#define BMP2_SPI_BUFFER_LEN 28 //! @see BMP280 technical note p. 24 //do sensora z instrukcji lab
#define BMP2_DATA_INDEX 1 //! @see BMP280 technical note p. 31-32
#define BMP2_REG_ADDR_INDEX 0 //! @see BMP280 technical note p. 31-32
#define BMP2_REG_ADDR_LEN 1 //! @see BMP280 technical note p. 31-32 //do sensora z instrukcji lab

#define BMP_CS_PORT CS_GPIO_Port //do pinu cs
#define BMP_CS_PIN  CS_Pin

#define CS_LOW()  HAL_GPIO_WritePin(BMP_CS_PORT, BMP_CS_PIN, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(BMP_CS_PORT, BMP_CS_PIN, GPIO_PIN_SET) //do pinu cs

#define BMP2_INTF_RET_TYPE int8_t//dofcji z pomocą czatu gpt
#define BMP2_INTF_RET_SUCCESS 0
#define BMP2_SPI &hspi4
#define BMP2_TIMEOUT 100
#define BMP2_NUM_OF_SENSORS 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t cs_index = 0;//do fcji z instrukcji bmp2_spi_read/write
int32_t t_fine; //z fcji czujnika

uint16_t dig_T1;   // bezznakowy
int16_t  dig_T2;   // ze znakiem
int16_t  dig_T3;   // ze znakiem

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.  //z dokumentacji czujnika
// t_fine carries fine temperature as global value

int32_t bmp280_compensate_T(int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
            ((int32_t)dig_T3)) >> 14;

    t_fine = var1 + var2;

    T = (t_fine * 5 + 128) >> 8;   // wynik w 0.01°C
    return T;
}


/*!
 * @brief Function for reading the sensor's registers through SPI bus.
 *
 * @param[in] reg_addr : Register address.
 * @param[out] reg_data : Pointer to the data buffer to store the read data.
 * @param[in] length : No of bytes to read.
 * @param[in] intf_ptr : Interface pointer
 *
 * @return Status of execution
 *
 * @retval BMP2_INTF_RET_SUCCESS-> Success.
 * @retval != BMP2_INTF_RET_SUCCESS-> Failure.
 *
 */
BMP2_INTF_RET_TYPE bmp2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    /* Implement the SPI read routine according to the target machine. */
    HAL_StatusTypeDef status = HAL_OK;
    int8_t iError = BMP2_INTF_RET_SUCCESS;
    uint8_t cs = *(uint8_t*)intf_ptr;

    /* Software slave selection procedure */
    HAL_GPIO_WritePin(BMP_CS_PORT, BMP_CS_PIN, GPIO_PIN_RESET);

    /* Data exchange */
    status = HAL_SPI_Transmit(BMP2_SPI, &reg_addr, BMP2_REG_ADDR_LEN, BMP2_TIMEOUT);
    status += HAL_SPI_Receive(BMP2_SPI, reg_data, length, BMP2_TIMEOUT);

    /* Disable all slaves */
    for(uint8_t i = 0; i < BMP2_NUM_OF_SENSORS; i++)
        HAL_GPIO_WritePin(BMP_CS_PORT, BMP_CS_PIN, GPIO_PIN_SET);

#ifdef DEBUG
    uint8_t data[BMP2_SPI_BUFFER_LEN] = {0,};
    memcpy(data, reg_data, length);
#endif

    // The BMP2xx API calls for 0 return value as a success, and -1 returned as failure
    if (status != HAL_OK)
        iError = -1;

    return iError;
}


/*!
 * @brief Function for writing the sensor's registers through SPI bus.
 *
 * @param[in] reg_addr : Register address.
 * @param[in] reg_data : Pointer to the data buffer whose data has to be written.
 * @param[in] length : No of bytes to write.
 * @param[in] intf_ptr : Interface pointer
 *
 * @return Status of execution
 *
 * @retval BMP2_INTF_RET_SUCCESS-> Success.
 * @retval != BMP2_INTF_RET_SUCCESS-> Failure.
 *
 */
BMP2_INTF_RET_TYPE bmp2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    /* Implement the SPI write routine according to the target machine. */
    HAL_StatusTypeDef status = HAL_OK;
    int8_t iError = BMP2_INTF_RET_SUCCESS;
    uint8_t cs = *(uint8_t*)intf_ptr;

#ifdef DEBUG
    uint8_t data[BMP2_SPI_BUFFER_LEN] = {0,};
    memcpy(data, reg_data, length);
#endif

    /* Software slave selection procedure */
    HAL_GPIO_WritePin(BMP_CS_PORT, BMP_CS_PIN, GPIO_PIN_RESET);

    /* Data exchange */
    status = HAL_SPI_Transmit(BMP2_SPI, &reg_addr, BMP2_REG_ADDR_LEN, BMP2_TIMEOUT);
    status += HAL_SPI_Transmit(BMP2_SPI, reg_data, length, BMP2_TIMEOUT);

    /* Disable all slaves */
    for(uint8_t i = 0; i < BMP2_NUM_OF_SENSORS; i++)
        HAL_GPIO_WritePin(BMP_CS_PORT, BMP_CS_PIN, GPIO_PIN_SET);

    // The BMP2xx API calls for 0 return value as a success, and -1 returned as failure
    if (status != HAL_OK)
        iError = -1;

    return iError;
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
  MX_SPI4_Init();
  /* USER CODE BEGIN 2 */
  uint8_t calib_data[6];
  bmp2_spi_read(0x88, calib_data, 6, &cs_index);

  dig_T1 = (uint16_t)(calib_data[1] << 8 | calib_data[0]);
  dig_T2 = (int16_t)(calib_data[3] << 8 | calib_data[2]);
  dig_T3 = (int16_t)(calib_data[5] << 8 | calib_data[4]);

  char msg2[64];
  snprintf(msg2, sizeof(msg2), "T1=%u  T2=%d  T3=%d\r\n", dig_T1, dig_T2, dig_T3);
  HAL_UART_Transmit(&huart3, (uint8_t*)msg2, strlen(msg2), HAL_MAX_DELAY);

  uint8_t ctrl_meas = 0x27; // np. normal mode, temp oversampling x1
  bmp2_spi_write(0xF4, &ctrl_meas, 1, &cs_index);

  uint8_t temp_data[3];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    // Odczyt 3 bajtów temperatury z rejestrów 0xFA, 0xFB, 0xFC
	    bmp2_spi_read(0xFA, temp_data, 3, &cs_index);

	    // Sklejanie bajtów w 20-bitową wartość adc_T
	    int32_t adc_T = ((int32_t)temp_data[0] << 12) |
	                    ((int32_t)temp_data[1] << 4) |
	                    ((temp_data[2] >> 4) & 0x0F);

	    // Obliczenie temperatury w setnych stopniach C (0.01°C)
	    int32_t T = bmp280_compensate_T(adc_T);

	    // Konwersja na float w °C i zapis do bufora ASCII
	    char msg[32];
	    //snprintf(msg, sizeof(msg), "Temp: %.2f C\r\n", T / 100.0f);

	    // Wysłanie przez UART3
	    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	    // Opcjonalne opóźnienie 1 s
	    HAL_Delay(1000);
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
