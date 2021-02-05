/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BUFFER_LENGTH 4
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t MAX30105_ADDRESS      =    0x57; //7-bit I2C Address

uint16_t MAX30105_AAAAAAAAAA	= 	0xFF;
uint16_t MAX30105_FIFOCONFIG 	=	0x08;
uint16_t MAX30105_MODECONFIG 	=	0x09;
uint16_t MAX30105_PARTICLECONFIG	 =	0x0A;
uint16_t MAX30105_LED1_PULSEAMP 	=	0x0C;
uint16_t MAX30105_LED2_PULSEAMP 	=	0x0D;
uint16_t MAX30105_LED3_PULSEAMP 	=	0x0E;
uint16_t MAX30105_LED_PROX_AMP 	=	0x10;
uint16_t MAX30105_MULTILEDCONFIG1 =	0x11;
uint16_t MAX30105_MULTILEDCONFIG2 =	0x12;
uint16_t MAX30105_ROLLOVER_ENABLE =	0x10;
uint16_t MAX30105_MODE_MULTILED = 	0x07;
uint16_t MAX30105_ADCRANGE_4096 = 	0x20;
uint16_t MAX30105_SAMPLERATE_100 = 	0x04;
uint16_t MAX30105_SAMPLERATE_50 = 	0x00;
uint16_t MAX30105_PULSEWIDTH_118 = 	0x01;
uint16_t MAX30105_LED_AMPLITUDE = 0x1F;
uint16_t MAX30105_INT_A_FULL_DISABLE = 0x00;
uint16_t MAX30105_SAMPLEAVG_16 = 	0x80;
uint8_t MAX30105_RESET = 			0x40;
uint8_t SLOT_RED_LED = 			0x01;
uint8_t SLOT_IR_LED = 				0x02;
uint8_t SLOT_GREEN_LED = 			0x03;
uint8_t MAX30105_ROLLOVER_MASK = 	0xEF;
uint8_t MAX30105_A_FULL_MASK = 	0xF0;
uint8_t MAX30105_RESET_MASK = 		0xBF;
uint8_t MAX30105_MODE_MASK = 		0xF8;
uint8_t MAX30105_ADCRANGE_MASK = 	0x9F;
uint8_t MAX30105_SAMPLERATE_MASK = 0xE3;
uint8_t MAX30105_PULSEWIDTH_MASK = 0xFC;
uint16_t MAX30105_SAMPLEAVG_MASK =	~0b11100000;
uint16_t MAX30105_INT_A_FULL_MASK =	~0b10000000;
uint8_t MAX30105_INT_DATA_RDY_MASK = ~0b01000000;
uint8_t MAX30105_FIFOWRITEPTR = 	0x04;
uint8_t MAX30105_FIFOOVERFLOW = 	0x05;
uint8_t MAX30105_FIFOREADPTR = 	0x06;
uint8_t MAX30105_FIFODATA =		0x07;

uint8_t MAX30105_SLOT1_MASK = 		0xF8;
uint8_t MAX30105_SLOT2_MASK = 		0x8F;
uint8_t MAX30105_SLOT3_MASK = 		0xF8;
uint8_t MAX30105_SLOT4_MASK = 		0x8F;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t buffer_rx[512] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void setup();
void read();
void clearFIFO(void);
void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);
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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  setup();
  while (1)
  {
	  read();
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart3.Init.BaudRate = 115200;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void setup()
{
	bitMask(MAX30105_MODECONFIG, MAX30105_RESET_MASK, MAX30105_RESET);
	bitMask(MAX30105_FIFOCONFIG, MAX30105_SAMPLEAVG_MASK, MAX30105_SAMPLEAVG_16);
	bitMask(MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_ENABLE);
	bitMask(MAX30105_MODECONFIG, MAX30105_MODE_MASK, MAX30105_MODE_MULTILED);
	bitMask(MAX30105_PARTICLECONFIG, MAX30105_ADCRANGE_MASK, MAX30105_ADCRANGE_4096);
	bitMask(MAX30105_PARTICLECONFIG, MAX30105_SAMPLERATE_MASK, MAX30105_SAMPLERATE_50);
	bitMask(MAX30105_PARTICLECONFIG, MAX30105_PULSEWIDTH_MASK, MAX30105_PULSEWIDTH_118);
	HAL_Delay(100);
	HAL_I2C_Mem_Write(&hi2c1, (MAX30105_ADDRESS << 1), (MAX30105_LED1_PULSEAMP),
			1, &MAX30105_LED_AMPLITUDE, sizeof(MAX30105_LED_AMPLITUDE),
			HAL_MAX_DELAY);
	HAL_Delay(100);
	HAL_I2C_Mem_Write(&hi2c1, (MAX30105_ADDRESS << 1), (MAX30105_LED2_PULSEAMP),
			1, &MAX30105_LED_AMPLITUDE, sizeof(MAX30105_LED_AMPLITUDE),
			HAL_MAX_DELAY);
	HAL_Delay(100);
	HAL_I2C_Mem_Write(&hi2c1, (MAX30105_ADDRESS << 1), (MAX30105_LED3_PULSEAMP),
			1, &MAX30105_LED_AMPLITUDE, sizeof(MAX30105_LED_AMPLITUDE),
			HAL_MAX_DELAY);
	HAL_Delay(100);
	HAL_I2C_Mem_Write(&hi2c1, (MAX30105_ADDRESS << 1), (MAX30105_LED_PROX_AMP),
			1, &MAX30105_LED_AMPLITUDE, sizeof(MAX30105_LED_AMPLITUDE),
			HAL_MAX_DELAY);
	HAL_Delay(100);
	bitMask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT1_MASK,SLOT_RED_LED);
	bitMask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT2_MASK,(SLOT_IR_LED<<4));
	bitMask(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT3_MASK,SLOT_GREEN_LED);

	clearFIFO();
}

void read(){
		 static volatile uint8_t readPointer=0;
		 HAL_I2C_Mem_Read(&hi2c1,(MAX30105_ADDRESS << 1), MAX30105_FIFOREADPTR, 1, &readPointer, sizeof(uint8_t), HAL_MAX_DELAY);

		 static volatile uint8_t writePointer=0;
		 HAL_I2C_Mem_Read(&hi2c1,(MAX30105_ADDRESS << 1), MAX30105_FIFOWRITEPTR, 1, &writePointer, sizeof(uint8_t), HAL_MAX_DELAY);

		 uint8_t numberOfSamples = 0;
		 if (readPointer != writePointer) {
			 if (writePointer > readPointer){
				 numberOfSamples = writePointer - readPointer;
			 }
			 else{
				 numberOfSamples = writePointer - readPointer + 32;
			 }

			 uint16_t bytesLeftToRead = numberOfSamples * 3 * 3;

			 if (bytesLeftToRead > 0){
				 HAL_I2C_Mem_Read(&hi2c1,(MAX30105_ADDRESS << 1), MAX30105_FIFODATA, 1, buffer_rx, bytesLeftToRead, HAL_MAX_DELAY);

				 static char s[20];
					 static uint32_t t=0;
					 for (int i=0; i < numberOfSamples; ++i) {
						 for (int j=0; j<3; ++j) {
							 t=0;
							 for (int k=0; k<3; ++k) {
								 t >>= 8;
								 t |= (buffer_rx[i*9+j*3+k]<<16);
							 }
							 if (j==0)
								 sprintf(s, "r: %" PRIu32 "\n", t);
							 if (j==1)
								 sprintf(s, "i: %" PRIu32 "\n", t);
							 if (j==2)
								 sprintf(s, "g: %" PRIu32 "\n", t);
							HAL_UART_Transmit(&huart3, (uint8_t*)s, strlen(s), 1000);
							HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 1000);
						 }
					 }
			 }

		 }
}
void clearFIFO(void) {
	uint8_t ziobro = 0;
	HAL_I2C_Mem_Write(&hi2c1, (MAX30105_ADDRESS << 1), (MAX30105_FIFOWRITEPTR),1, &ziobro, sizeof(ziobro), HAL_MAX_DELAY);
	HAL_Delay(100);
	HAL_I2C_Mem_Write(&hi2c1, (MAX30105_ADDRESS << 1), (MAX30105_FIFOOVERFLOW),1, &ziobro, sizeof(ziobro), HAL_MAX_DELAY);
	HAL_Delay(100);
	HAL_I2C_Mem_Write(&hi2c1, (MAX30105_ADDRESS << 1), (MAX30105_FIFOREADPTR),1, &ziobro, sizeof(ziobro), HAL_MAX_DELAY);

}
void bitMask(uint8_t reg, uint8_t mask, uint8_t thing)
{
	//bitMask(0x08, 0xEF, 0x10);
	HAL_Delay(100);
	volatile uint8_t originalContent;

  // Grab current register context
	 //HAL_I2C_IsDeviceReady(&hi2c1, reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1,(MAX30105_ADDRESS << 1) ,reg , 1, &originalContent, sizeof(uint8_t), HAL_MAX_DELAY);


	// Zero-out the portions of the register we're interested in
	 originalContent = originalContent & mask;
  // Change contents
	 uint8_t content = 0;
	 content = originalContent | thing;
	 HAL_I2C_Mem_Write(&hi2c1, (MAX30105_ADDRESS << 1),
	 			(reg), 1, &content , sizeof(content),
	 			HAL_MAX_DELAY);
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
