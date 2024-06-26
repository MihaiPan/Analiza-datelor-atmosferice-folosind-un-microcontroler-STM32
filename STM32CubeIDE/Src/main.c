/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "bmp280.h"

#include "UartRingbuffer.h"
#include "NMEA.h"

//#include "hdc1080.h"
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

//Variabile definite pentru BMP280
BMP280_HandleTypedef bmp280;
float humidity;
float sea_hPa=101325;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*Printf translation to UART*/
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

//Definirea variabilelor DHT22
#define DHT22_PIN GPIO_PIN_12
#define DHT22_PORT GPIOC
uint8_t RHD, RHI, TCD, TCI, SUM;
uint32_t pMillis, cMillis;
float temp=0;
float humid=0;

//Definirea funcțiilor DHT22
uint8_t DHT22_Start(void);
uint8_t DHT22_Read();

//Variabile BMP
float BMP_Temp=0;
float BMP_Press=0;
float BMP_Alt =0;

/*Variabile HDC1080*/
uint16_t HDC_temp;
uint16_t HDC_humi;
uint8_t read_data[2];
uint16_t reg=0;

//Declararea parametrilor pentru GPS
char GGA[80];
GPSSTRUCT gpsData;

/*Definirea parametrilor LCD*/
#define LCD_I2C_ADDR 0x27
#define RS_BIT 0
#define EN_BIT 2
#define BL_BIT 3
#define D4_BIT 4
#define D5_BIT 5
#define D6_BIT 6
#define D7_BIT 7
#define LCD_ROWS 2
#define LCD_COLS 16
uint8_t backlight_state = 1;
extern volatile uint8_t press_count;

/*Declararea funcțiilor LCD*/
void lcd_write_nibble(uint8_t nibble, uint8_t rs);
void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_init();
void lcd_write_string(char *str);
void lcd_set_cursor(uint8_t row, uint8_t column);
void lcd_clear(void);
void lcd_backlight(uint8_t state);



//delay function in microseconds
void delay (uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim1))<time);
}


#define HDC1080_ADDRESS 0x40 << 1  // Adresa HDC1080
HAL_StatusTypeDef HDC1080_Init(void);
HAL_StatusTypeDef HDC1080_ReadTempHumidity(uint16_t *temperature, uint16_t *humidity);


#define CCS811_ADDRESS 0x5A << 1   // Adresa CCS811 (0x5A << 1 pentru formatul HAL)
HAL_StatusTypeDef CCS811_SoftReset(void);
HAL_StatusTypeDef CCS811_Init(void);
HAL_StatusTypeDef CCS811_ReadData(uint16_t *eco2, uint16_t *tvoc);
void CCS811_Env(float temperature, float humidity);


char ESP_msg[50];


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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim1);
  HAL_ADCEx_Calibration_Start(&hadc1);

  PWR->CR |= PWR_CR_DBP;

  // Dezactivează LSE
  RCC->BDCR &= ~RCC_BDCR_LSEON;

  // Dezactivează alimentarea domeniului de backup
  RCC->BDCR &= ~RCC_BDCR_BDRST;

  //Inițializare LCD
  lcd_init();
  lcd_backlight(1); // Turn on backlight
  char int_to_str_1[20];
  char int_to_str_2[20];

  //Inițializare BMP280
  bmp280_init_default_params(&bmp280.params);
  bmp280.addr = BMP280_I2C_ADDRESS_0;
  bmp280.i2c = &hi2c2;

  while (!bmp280_init(&bmp280, &bmp280.params)) {
  	printf("BMP280 initialization failed\n\r");
  	HAL_Delay(2000);
  	break;
  }

  // Inițializare Buffer circular
  Ringbuf_init();
  HAL_Delay(500);

  // Inițializare HDC1080 și CCS811
  if(HDC1080_Init()!= HAL_OK){
	  printf("Nu merge HDC\n\r");
	  HAL_Delay(1000);
  }
  if (CCS811_Init() != HAL_OK) {
	  printf("Nu merge CCS\n\r");
  }
  uint16_t eco2, tvoc;
  float tempC, humRH;
  float latitude=44.4,longitutde=26.06;
  float UVIndex=0;




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //DHT22 ---------------------------------------------------------------------
	  if(DHT22_Start())
	  {
		  RHI = DHT22_Read();
		  RHD = DHT22_Read();
		  TCI = DHT22_Read();
		  TCD = DHT22_Read();
		  SUM = DHT22_Read();
		  printf("RHI: %u RHD: %u TCI: %u TCD %u SUM: %u \n\r",RHI,RHD, TCI,TCD,SUM);

		  if(TCI>127)
		  {
		  temp= (float)TCD/10*(-1);
		  }
		  else
		  {
		  temp = (float)((TCI<<8)|TCD)/10;
		  }
		  humid = (float)((RHI<<8|RHD))/10;
		  printf("%.1f %.1f\n\r",temp,humid);
	  }
	  HAL_Delay(1000);
	  //------------------------------------------------------------------------

	  //GY-NEO6MV2--------------------------------------------------------------
	  if(Wait_for("GGA")==1){
		  Copy_upto("*", GGA);
		  decodeGGA(GGA,&gpsData.ggastruct);
		  if(gpsData.ggastruct.lcation.latitude!=0){
			  latitude=gpsData.ggastruct.lcation.latitude;
		  }
		  if(gpsData.ggastruct.lcation.longitude!=0){
			  longitutde=gpsData.ggastruct.lcation.longitude;
		  }
		  printf("lat: %.5lf",gpsData.ggastruct.lcation.latitude);
		  printf(" long: %.5lf \n\r",gpsData.ggastruct.lcation.longitude);
	  }
	  HAL_Delay(1000);
	  //------------------------------------------------------------------------

	  //BMP280 -----------------------------------------------------------------
	  while (!bmp280_read_float(&bmp280, &BMP_Temp, &BMP_Press, &humidity)) {
	  	printf("Temperature/pressure reading failed\n\r");
	  	HAL_Delay(2000);
	  }

	  BMP_Alt= 44330*(1.0- pow(BMP_Press/sea_hPa, 0.1903));
	  printf("Pressure: %.2f Pa, Temp: %.2f C, Alt: %.2f \n\r",BMP_Press, BMP_Temp, BMP_Alt);

	  HAL_Delay(500);
	  //------------------------------------------------------------------------

	  // HDC și CCS811 ---------------------------------------------------------
	  if (HDC1080_ReadTempHumidity(&HDC_temp, &HDC_humi) == HAL_OK)
	  {
		  tempC = (HDC_temp / 65536.0) * 165.0 - 40.0;
	      humRH = (HDC_humi / 65536.0) * 100.0;
	      printf("HDC_temp: %.2f  HDC_humi: %.2f \n\r",tempC,humRH);
	      CCS811_Env(tempC, humRH);
	  }
	  if (CCS811_ReadData(&eco2, &tvoc) == HAL_OK) {
		  printf("ECO: %u TVOC: %u \n\r",eco2,tvoc);
	  }
	  HAL_Delay(1000);
	  //------------------------------------------------------------------------

	  // UV light sensor--------------------------------------------------------
	  HAL_ADC_Start(&hadc1);
	  if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
	  {
	        uint32_t adcValue = HAL_ADC_GetValue(&hadc1);
	        float voltage = (adcValue / 4095.0) * 5;
	        float current= voltage/4.3;
	        UVIndex=voltage/0.1;
	        printf("UV voltage: %.2f UV phcurrent: %.2f Index: %.0f\n\r",voltage, current, voltage/0.1);
	  }
	  //------------------------------------------------------------------------

	  // ESP8266-----------------------------------------------------------------------------
	  snprintf(ESP_msg,sizeof(ESP_msg),"%.2f,%.2f,%.2f,%.1f,%u,%u,%.5f,%.5f\n",temp,humid,BMP_Press,UVIndex,eco2,tvoc,latitude,longitutde);
	  HAL_UART_Transmit(&huart1, (uint8_t*)ESP_msg, strlen(ESP_msg), HAL_MAX_DELAY);
	  //------------------------------------------------------------------------

	  //LCD --------------------------------------------------------------------
  	  switch(press_count){
  	  case 0:
  		  	  char *str1 = "Temperature:";
  		  	  char *str2 = "Humidity:";

  		  	  sprintf(int_to_str_1, "%.1f", tempC);
  		  	  lcd_clear();
  		  	  lcd_set_cursor(0, 0);
  		  	  lcd_write_string(str1);
  		  	  lcd_set_cursor(0, 12);
  		  	  lcd_write_string(int_to_str_1);

  		  	  sprintf(int_to_str_2, "%.1f", humRH);
  		  	  lcd_set_cursor(1, 0);
  		  	  lcd_write_string(str2);
  		  	  lcd_set_cursor(1, 12);
  		  	  lcd_write_string(int_to_str_2);

  		  	  memset(int_to_str_1, 0, sizeof(int_to_str_1));
  		  	  memset(int_to_str_2, 0, sizeof(int_to_str_2));
  		  break;
  	  case 1:
  		  	  char *str3 = "eCO2:";
  		  	  char *str4 = "TVOC:";

  		  	  sprintf(int_to_str_1, "%d", eco2);
  		  	  lcd_clear();
  		  	  lcd_set_cursor(0, 0);
  		  	  lcd_write_string(str3);
  		  	  lcd_set_cursor(0, 10);
  		  	  lcd_write_string(int_to_str_1);

  		  	  sprintf(int_to_str_2, "%d", tvoc);
  		  	  lcd_set_cursor(1, 0);
  		  	  lcd_write_string(str4);
  		  	  lcd_set_cursor(1, 10);
  		  	  lcd_write_string(int_to_str_2);

  		  	  memset(int_to_str_1, 0, sizeof(int_to_str_1));
  		  	  memset(int_to_str_2, 0, sizeof(int_to_str_2));
  		  break;
  	  case 2:
  		  	  char *str5 = "Pressure:";
  		  	  char *str6 = "UVI:";

  		  	  sprintf(int_to_str_1, "%.2f", BMP_Press);
  		  	  lcd_clear();
  		  	  lcd_set_cursor(0, 0);
  		  	  lcd_write_string(str5);
  		  	  lcd_set_cursor(0, 10);
  		  	  lcd_write_string(int_to_str_1);

  		  	  sprintf(int_to_str_2, "%.1f", UVIndex);
  		  	  lcd_set_cursor(1, 0);
  		  	  lcd_write_string(str6);
  		  	  lcd_set_cursor(1, 10);
  		  	  lcd_write_string(int_to_str_2);

  		  	  memset(int_to_str_1, 0, sizeof(int_to_str_1));
  		  	  memset(int_to_str_2, 0, sizeof(int_to_str_2));
  		  break;
  	  case 3:
  		  	  char *str7 = "Lat:";
  		  	  char *str8 = "Long:";

  		  	  sprintf(int_to_str_1, "%.4f", latitude);
  		  	  lcd_clear();
  		  	  lcd_set_cursor(0, 0);
  		  	  lcd_write_string(str7);
  		  	  lcd_set_cursor(0, 8);
  		  	  lcd_write_string(int_to_str_1);

  		  	  sprintf(int_to_str_2, "%.4f", longitutde);
  		  	  lcd_set_cursor(1, 0);
  		  	  lcd_write_string(str8);
  		  	  lcd_set_cursor(1, 8);
  		  	  lcd_write_string(int_to_str_2);

  		  	  memset(int_to_str_1, 0, sizeof(int_to_str_1));
  		  	  memset(int_to_str_2, 0, sizeof(int_to_str_2));
  		  break;
  	  }
  	//------------------------------------------------------------------------
  	// Buzzer-----------------------------------------------------------------
  	  if((UVIndex >= 7) || (eco2>=2300) || (tvoc>=500)){
  		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
  		HAL_Delay(100);
  		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
  		HAL_Delay(100);
  		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
  		HAL_Delay(100);
  		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
  		HAL_Delay(100);
  		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
  		HAL_Delay(100);
  	  }
  	  //------------------------------------------------------------------------
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7200-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Temperature and humidity sensor-----------------------------------------
uint8_t DHT22_Start(void)
{
	uint8_t Response=0;

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DHT22_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStruct);

	HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, 0);
	delay(18000);
	HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, 1);
	delay(30);

	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStruct);

	delay(40);
	if(!(HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN)))
	{
		delay(80);
		if((HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN)))Response=1;
		else Response=-1;
	}
	pMillis = HAL_GetTick();
	cMillis = HAL_GetTick();
	while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis + 2 > cMillis)
    {
		cMillis = HAL_GetTick();
	}
	return Response;
}

uint8_t DHT22_Read()
{
	uint8_t x,y;
	for (x=0;x<8;x++)
	{
	    pMillis = HAL_GetTick();
	    cMillis = HAL_GetTick();
	    while (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis + 2 > cMillis)
	    {
	      cMillis = HAL_GetTick();
	    }
	    delay(50);
	    if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)))
	      y&= ~(1<<(7-x));
	    else
	      y|= (1<<(7-x));
	    pMillis = HAL_GetTick();
	    cMillis = HAL_GetTick();
	    while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis + 2 > cMillis)
	    {  // wait for the pin to go low
	      cMillis = HAL_GetTick();
	    }
	}
	return y;
}

HAL_StatusTypeDef HDC1080_Init(void) {
    uint8_t buffer[3];
    buffer[0] = 0x02;  // Adresa registrului de configurare
    buffer[1] = 0x10;  // Configurație: mod de achiziție umiditate și temperatură
    buffer[2] = 0x00;  // Configurație standard

    return HAL_I2C_Master_Transmit(&hi2c2, HDC1080_ADDRESS, buffer, 3, HAL_MAX_DELAY);
}

HAL_StatusTypeDef HDC1080_ReadTempHumidity(uint16_t *temperature, uint16_t *humidity) {
    uint8_t buffer[4];
    uint8_t command = 0x00;  // Comandă pentru a citi temperatura și umiditatea
    HAL_StatusTypeDef ret;

    // Trimite comanda pentru a citi temperatura și umiditatea
    ret = HAL_I2C_Master_Transmit(&hi2c2, HDC1080_ADDRESS, &command, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        return ret;
    }

    // Asteapta timpul necesar pentru masurare (maxim 15 ms)
    HAL_Delay(15);

    // Citeste 4 octeti (2 pentru temperatura, 2 pentru umiditate)
    ret = HAL_I2C_Master_Receive(&hi2c2, HDC1080_ADDRESS, buffer, 4, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        return ret;
    }

    // Convertim valorile citite
    *temperature = (buffer[0] << 8) | buffer[1];
    *humidity = (buffer[2] << 8) | buffer[3];

    return HAL_OK;
}
//----------------------------------------------------------------------

//CCS811 functions------------------------------------------------------
HAL_StatusTypeDef CCS811_SoftReset(void) {
    uint8_t buffer[4] = {0x11, 0xE5, 0x72, 0x8A};  // Secvența de reset

    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(&hi2c2, CCS811_ADDRESS, 0xFF, I2C_MEMADD_SIZE_8BIT, buffer, 4, HAL_MAX_DELAY);
    HAL_Delay(100);  // Așteaptă după reset
    if (ret != HAL_OK) {
    	printf("Failed to perform software reset: %d\n\r", ret);
    }
    return ret;
}

HAL_StatusTypeDef CCS811_Init(void) {
    HAL_StatusTypeDef ret;

    // Soft reset pentru senzor
    ret = CCS811_SoftReset();
    if (ret != HAL_OK) {
        return ret;
    }

    // Verifică starea senzorului după reset
    uint8_t status;
    ret = HAL_I2C_Mem_Read(&hi2c2, CCS811_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, &status, 1, HAL_MAX_DELAY);
    //printf("Status215 %u \n\r",status);
    if (ret != HAL_OK) {
    	return ret;
    }

    uint8_t error_data=0;
    // Read the error register
    ret = HAL_I2C_Mem_Read(&hi2c2, CCS811_ADDRESS, 0xE0, I2C_MEMADD_SIZE_8BIT, &error_data, 1, HAL_MAX_DELAY);
    //printf("Status223 %u ERROR: %u \n\r",status,error_data);
    if (ret != HAL_OK) {
    	printf("ERR STAT: %u\n\r",error_data);
    	return ret;
    }

    uint8_t app_start_cmd =0xF4; //Starting App
    ret = HAL_I2C_Mem_Write(&hi2c2, CCS811_ADDRESS, app_start_cmd,1,0,0,HAL_MAX_DELAY);
    if (ret != HAL_OK) {
    	printf("Failed to start application mode: %d\n\r", ret);
        return ret;
    }

    uint8_t desired_mode= 0x10;
    ret= HAL_I2C_Mem_Write(&hi2c2, CCS811_ADDRESS, 0x01, I2C_MEMADD_SIZE_8BIT, &desired_mode, 1, HAL_MAX_DELAY);
    //printf("%d",ret);


    uint8_t mode = 0;
    ret = HAL_I2C_Mem_Read(&hi2c2, CCS811_ADDRESS, 0x01, I2C_MEMADD_SIZE_8BIT, &mode, 1, HAL_MAX_DELAY);
    //printf("Mode : %u \n\r",mode);

    HAL_Delay(1000);

    return HAL_OK;
}

HAL_StatusTypeDef CCS811_ReadData(uint16_t *eco2, uint16_t *tvoc) {
    uint8_t buffer[8];
    uint8_t status;
    HAL_StatusTypeDef ret;

    // Verificare disponibilitate date
    ret = HAL_I2C_Mem_Read(&hi2c2, CCS811_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, &status, 1, HAL_MAX_DELAY);
    //printf("Status %u \n\r",status);
    if (ret != HAL_OK) {
           return ret;
       }
    // Citire date
    ret = HAL_I2C_Mem_Read(&hi2c2, CCS811_ADDRESS, 0x02, I2C_MEMADD_SIZE_8BIT, buffer, 8, HAL_MAX_DELAY);
    //printf("ajunge pana aici\n\r");
    if (ret != HAL_OK) {
        return ret;
    }

    *eco2 = (buffer[0] << 8) | buffer[1];
    *tvoc = (buffer[2] << 8) | buffer[3];

    ret = HAL_I2C_Mem_Read(&hi2c2, CCS811_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, &status, 1, HAL_MAX_DELAY);
        //printf("Status %u \n\r",status);
        if (ret != HAL_OK) {
               return ret;
           }

    return HAL_OK;
}

void CCS811_Env(float temperature, float humidity) {
	uint8_t env[4];
	uint8_t Env_Data=0x05;
	HAL_StatusTypeDef ret;

	uint16_t temp = (uint16_t)round(temperature * 512.0);
	env[0] = (temp >>8) & 0xFF;
	env[1] =  temp & 0xFF;

	int16_t hum = (int16_t)round(humidity * 512.0);
	env[2] = (hum >> 8) & 0xFF;
	env[3] = hum & 0xFF;

	ret = HAL_I2C_Mem_Write(&hi2c2, CCS811_ADDRESS, Env_Data, 1, env, 4, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		//printf("Nu mere env setting\n\r");
	}
	//printf(" A  AJUNS ACII\n\r");

}
//----------------------------------------------------------------------

//LCD functions --------------------------------------------------------
void lcd_write_nibble(uint8_t nibble, uint8_t rs) {
  uint8_t data = nibble << D4_BIT;
  data |= rs << RS_BIT;
  data |= backlight_state << BL_BIT; // Include backlight state in data
  data |= 1 << EN_BIT;
  HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADDR << 1, &data, 1, 100);
  HAL_Delay(1);
  data &= ~(1 << EN_BIT);
  HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADDR << 1, &data, 1, 100);
}

void lcd_send_cmd(uint8_t cmd) {
  uint8_t upper_nibble = cmd >> 4;
  uint8_t lower_nibble = cmd & 0x0F;
  lcd_write_nibble(upper_nibble, 0);
  lcd_write_nibble(lower_nibble, 0);
  if (cmd == 0x01 || cmd == 0x02) {
    HAL_Delay(2);
  }
}

void lcd_send_data(uint8_t data) {
  uint8_t upper_nibble = data >> 4;
  uint8_t lower_nibble = data & 0x0F;
  lcd_write_nibble(upper_nibble, 1);
  lcd_write_nibble(lower_nibble, 1);
}

void lcd_init() {
  HAL_Delay(50);
  lcd_write_nibble(0x03, 0);
  HAL_Delay(5);
  lcd_write_nibble(0x03, 0);
  HAL_Delay(1);
  lcd_write_nibble(0x03, 0);
  HAL_Delay(1);
  lcd_write_nibble(0x02, 0);
  lcd_send_cmd(0x28);
  lcd_send_cmd(0x0C);
  lcd_send_cmd(0x06);
  lcd_send_cmd(0x01);
  HAL_Delay(2);
}

void lcd_write_string(char *str) {
  while (*str) {
    lcd_send_data(*str++);
  }
}

void lcd_set_cursor(uint8_t row, uint8_t column) {
    uint8_t address;
    switch (row) {
        case 0:
            address = 0x00;
            break;
        case 1:
            address = 0x40;
            break;
        default:
            address = 0x00;
    }
    address += column;
    lcd_send_cmd(0x80 | address);
}

void lcd_clear(void) {
	lcd_send_cmd(0x01);
    HAL_Delay(2);
}

void lcd_backlight(uint8_t state) {
  if (state) {
    backlight_state = 1;
  } else {
    backlight_state = 0;
  }
}
//End of LCD functions -----------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==B1_Pin || GPIO_Pin==GPIO_PIN_3){
		press_count++;
				if(press_count>3)
					press_count=0;
	}
	if(GPIO_Pin==GPIO_PIN_2){
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)== GPIO_PIN_RESET){
								if(press_count==0){
									press_count=3;
								}else
									press_count--;

							}
		}
/*

*/

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
