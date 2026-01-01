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
char buffer[150];
char spibuffer[20];
float TEMPERATURA = 0.0f;
float cisnienie = 0.0f;

int32_t temp_int = 0;
    int32_t press_hpa = 0;

//bity wewnątrz czujnika
uint8_t msb = 0;
uint8_t lsb = 0;
uint8_t xlsb = 0;

uint16_t dig_T1 =0;//dane kalibracyjne
uint16_t dig_T2 =0;//same
uint16_t dig_T3 =0;//same

uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;


int32_t TempBezSkalowania =0;

int32_t t_fine =0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void BMP_Write_Register(uint8_t reg_addr, uint8_t value) {
    uint8_t data[2];
    data[0] = reg_addr & 0x7F;  // Bit 7 = 0 dla zapisu
    data[1] = value;

    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi4, data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

void HUART_Transmitmessage(char *msg){
	HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void BMP280_init(){
    uint8_t reg = 0xD0 | 0x80;
    uint8_t id = 0;
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi4, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi4, &id, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

    if (id == 0x58) {
        HUART_Transmitmessage("BMP280 OK\r\n");

        // Konfiguracja czujnika
        // Rejestr 0xF4: os_over_t=1, os_over_p=1, mode=1 (forced mode)
        BMP_Write_Register(0xF4, 0x25);  // Temp oversampling x1, Pressure oversampling x1, Forced mode
        HAL_Delay(100);  // Poczekaj na pomiar

    } else {
        sprintf(buffer, "ERROR BMP280 read id=%u\r\n", id);
        HUART_Transmitmessage(buffer);
    }
}

uint8_t BMP_Readyte(uint8_t rejestr){
	uint8_t val = 0;
	uint8_t reg = 0xD0;//tutaj podajesz adres swojego regulatora który Ci wypluł przez uart przy inicie
	rejestr |= 0x80; //zeby 7 bajt byl 1 czyli wiadomosc do odczytania
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);//odpala czujnik
	HAL_SPI_Transmit(&hspi4, &rejestr, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi4, &val, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);//wyłącza czujnik
	return val;
}

int32_t SPI_Odbierz_Temp(){
    // Upewnij się, że czujnik wykonał pomiar
    HAL_Delay(10);

    msb = BMP_Readyte(0xFA);
    lsb = BMP_Readyte(0xFB);
    xlsb = BMP_Readyte(0xFC);

    sprintf(buffer, "MSB=%u, LSB=%u, XLSB=%u\r\n", msb, lsb, xlsb);
    HUART_Transmitmessage(buffer);

    int32_t raw = ((uint32_t)msb << 12) | ((uint32_t)lsb << 4) | (xlsb >> 4);
    sprintf(buffer, "Raw calculated: %ld\r\n", raw);
    HUART_Transmitmessage(buffer);

    return raw;
}

void BMP280_Calibrate (){
    dig_T1 = BMP_Readyte(0x88) | ((uint16_t)BMP_Readyte(0x89) << 8);
    dig_T2 = (int16_t)(BMP_Readyte(0x8A) | ((uint16_t)BMP_Readyte(0x8B) << 8));
    dig_T3 = (int16_t)(BMP_Readyte(0x8C) | ((uint16_t)BMP_Readyte(0x8D) << 8));
}

float BMP280_Calctemp(int32_t adc_T) {
    int64_t var1, var2;
    int32_t temperature;

    // Rzutowanie danych kalibracyjnych
    int32_t dig_T2_signed = (int16_t)dig_T2;
    int32_t dig_T3_signed = (int16_t)dig_T3;

    // Obliczenia z większą precyzją
    var1 = (((int64_t)adc_T) >> 3) - ((int64_t)dig_T1 << 1);
    var1 = (var1 * dig_T2_signed) >> 11;

    var2 = (((int64_t)adc_T) >> 4) - (int64_t)dig_T1;
    var2 = (var2 * var2) >> 12;
    var2 = (var2 * dig_T3_signed) >> 14;

    t_fine = var1 + var2;

    // Obliczenie temperatury
    temperature = (t_fine * 5 + 128) >> 8;

    // Konwersja do float
    return ((float)temperature) / 100.0f;
}

//void zad2(){
//	BMP280_Calibrate();
//	TEMPERATURA = BMP280_Calctemp(SPI_Odbierz_Temp());
//	sprintf(buffer, "Temperatura to: %.2f", TEMPERATURA);
//	HUART_Transmitmessage(buffer);
//	HAL_Delay(500);
//}

//void zad2(){
//    BMP280_Calibrate();
//
//    // Debugowanie danych kalibracyjnych
//    sprintf(buffer, "dig_T1=%u, dig_T2=%d, dig_T3=%d\r\n", dig_T1, dig_T2, dig_T3);
//    HUART_Transmitmessage(buffer);
//
//    int32_t raw_temp = SPI_Odbierz_Temp();
//    sprintf(buffer, "Raw temp: %ld\r\n", raw_temp);
//    HUART_Transmitmessage(buffer);
//
//    TEMPERATURA = BMP280_Calctemp(raw_temp);
//    sprintf(buffer, "Temperatura to: %.2f\r\n", TEMPERATURA);
//    HUART_Transmitmessage(buffer);
//    HAL_Delay(1000);
//}

void zad2(){
    char temp_buffer[100];

    BMP280_Calibrate();
    int32_t raw_temp = SPI_Odbierz_Temp();

    float temperatura = BMP280_Calctemp(raw_temp);

    // Sprawdź czy wynik jest poprawny
    if (temperatura != temperatura) {  // Sprawdza NaN
        sprintf(temp_buffer, "Temperatura: NaN\r\n");
    } else if (temperatura > -50.0f && temperatura < 100.0f) {
        sprintf(temp_buffer, "Temperatura to: %.2f\r\n", temperatura);
    } else {
        sprintf(temp_buffer, "Temperatura out of range: %.2f\r\n", temperatura);
    }

    HUART_Transmitmessage(temp_buffer);
    HAL_Delay(1000);
}

int32_t SPI_Odbierz_Press() {
    HAL_Delay(10); // upewnij się, że pomiar zakończony

    uint8_t msb = BMP_Readyte(0xF7);
    uint8_t lsb = BMP_Readyte(0xF8);
    uint8_t xlsb = BMP_Readyte(0xF9);

    sprintf(buffer, "P_MSB=%u, P_LSB=%u, P_XLSB=%u\r\n", msb, lsb, xlsb);
    HUART_Transmitmessage(buffer);

    int32_t raw = ((uint32_t)msb << 12) |
                  ((uint32_t)lsb << 4)  |
                  (xlsb >> 4);

    sprintf(buffer, "Raw pressure: %ld\r\n", raw);
    HUART_Transmitmessage(buffer);

    return raw;
}

void BMP280_Calibrate_Press() {
    dig_P1 =  (uint16_t)(BMP_Readyte(0x8E) | ((uint16_t)BMP_Readyte(0x8F) << 8));
    dig_P2 =  (int16_t)(BMP_Readyte(0x90) | ((uint16_t)BMP_Readyte(0x91) << 8));
    dig_P3 =  (int16_t)(BMP_Readyte(0x92) | ((uint16_t)BMP_Readyte(0x93) << 8));
    dig_P4 =  (int16_t)(BMP_Readyte(0x94) | ((uint16_t)BMP_Readyte(0x95) << 8));
    dig_P5 =  (int16_t)(BMP_Readyte(0x96) | ((uint16_t)BMP_Readyte(0x97) << 8));
    dig_P6 =  (int16_t)(BMP_Readyte(0x98) | ((uint16_t)BMP_Readyte(0x99) << 8));
    dig_P7 =  (int16_t)(BMP_Readyte(0x9A) | ((uint16_t)BMP_Readyte(0x9B) << 8));
    dig_P8 =  (int16_t)(BMP_Readyte(0x9C) | ((uint16_t)BMP_Readyte(0x9D) << 8));
    dig_P9 =  (int16_t)(BMP_Readyte(0x9E) | ((uint16_t)BMP_Readyte(0x9F) << 8));
}

float BMP280_CalcPress(int32_t adc_P)
{
    int64_t var1, var2, p;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) +
           ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

    if (var1 == 0)
        return 0; // dzielenie przez zero = błąd

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;

    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + ((int64_t)dig_P7 << 4);

    return p / 256.0f;  // wynik w Pascalach
}

//float zad4() {
//    char msg[100];
//
//    BMP280_Calibrate();       // dla temp → generuje t_fine
//    BMP280_Calibrate_Press(); // dla ciśnienia
//
//    int32_t raw_temp = SPI_Odbierz_Temp();
//    float temp = BMP280_Calctemp(raw_temp);
//
//    int32_t raw_press = SPI_Odbierz_Press();
//    float press = BMP280_CalcPress(raw_press);
//
//    sprintf(msg, "Temp=%.2f C, Pressure=%.2f Pa\r\n", temp, press);
//    HUART_Transmitmessage(msg);
//
//    HAL_Delay(1000);
//    return press;
//}

int zad4() {
    BMP280_Calibrate();
    BMP280_Calibrate_Press();

    int32_t raw_temp = SPI_Odbierz_Temp();
    float temperatura = BMP280_Calctemp(raw_temp);

    int32_t raw_press = SPI_Odbierz_Press();
    float cisnienie = BMP280_CalcPress(raw_press);

    // --- KONWERSJA DO INTÓW ---
 temp_int = (int32_t)(temperatura * 100 * 1000);   // °C ×100 mC * 1000*C
 press_hpa = (int32_t)(cisnienie * 0.01f);  // Pa → hPa



    sprintf(buffer,
        "Temp: %ld.%02ld C, Pressure: %ld hPa\r\n",
        temp_int/100, temp_int%100,
        press_hpa
    );
    HUART_Transmitmessage(buffer);
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
BMP280_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
//zad2();
zad4();

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
