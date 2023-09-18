/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
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


	// Define variables as its name and type from data sheet
	uint8_t dig_H1, dig_H3, dig_H6, rawData[8];
	int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9, dig_H2, dig_H4, dig_H5;
	uint16_t dig_T1, dig_P1;
	int32_t tRaw, pRaw, hRaw, t_fine;
	char sendData[80];
	int len, temp, pressure, humid;


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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */


	// Copy functions from data sheet that compensate data from sensor and switch "BME280_S32_t" to "int32_t"
	int32_t BME280_compensate_T_int32(int32_t adc_T) {

		int32_t var1, var2, T;
		var1 = ((((adc_T >> 3) - ((int32_t) dig_T1 << 1))) * ((int32_t) dig_T2))
				>> 11;
		var2 = (((((adc_T >> 4) - ((int32_t) dig_T1))
				* ((adc_T >> 4) - ((int32_t) dig_T1))) >> 12)
				* ((int32_t) dig_T3)) >> 14;
		t_fine = var1 + var2;
		T = (t_fine * 5 + 128) >> 8;
		return T;
	}

	uint32_t BME280_compensate_P_int64(int32_t adc_P) {

		int64_t var1, var2, p;
		var1 = ((int64_t) t_fine) - 128000;
		var2 = var1 * var1 * (int64_t) dig_P6;
		var2 = var2 + ((var1 * (int64_t) dig_P5) << 17);
		var2 = var2 + (((int64_t) dig_P4) << 35);
		var1 = ((var1 * var1 * (int64_t) dig_P3) >> 8)
				+ ((var1 * (int64_t) dig_P2) << 12);
		var1 = (((((int64_t) 1) << 47) + var1)) * ((int64_t) dig_P1) >> 33;
		if (var1 == 0) {
			return 0; // avoid exception caused by division by zero
		}
		p = 1048576 - adc_P;
		p = (((p << 31) - var2) * 3125) / var1;
		var1 = (((int64_t) dig_P9) * (p >> 13) * (p >> 13)) >> 25;
		var2 = (((int64_t) dig_P8) * p) >> 19;
		p = ((p + var1 + var2) >> 8) + (((int64_t) dig_P7) << 4);
		return (uint32_t) p;
	}

	uint32_t bme280_compensate_H_int32(int32_t adc_H) {

		int32_t v_x1_u32r;
		v_x1_u32r = (t_fine - ((int32_t) 76800));
		v_x1_u32r = (((((adc_H << 14) - (((int32_t) dig_H4) << 20)
				- (((int32_t) dig_H5) * v_x1_u32r)) + ((int32_t) 16384)) >> 15)
				* (((((((v_x1_u32r * ((int32_t) dig_H6)) >> 10)
						* (((v_x1_u32r * ((int32_t) dig_H3)) >> 11)
								+ ((int32_t) 32768))) >> 10)
						+ ((int32_t) 2097152)) * ((int32_t) dig_H2) + 8192)
						>> 14));
		v_x1_u32r = (v_x1_u32r
				- (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7)
						* ((int32_t) dig_H1)) >> 4));
		v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
		v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
		return (uint32_t) (v_x1_u32r >> 12);
	}

	// Function for getting compensation variables.
	void calibData(uint16_t *dig_T1,int16_t *dig_T2,int16_t *dig_T3,uint16_t *dig_P1,int16_t *dig_P2,int16_t *dig_P3,int16_t *dig_P4,int16_t *dig_P5,int16_t *dig_P6,int16_t *dig_P7,int16_t *dig_P8,int16_t *dig_P9,uint8_t *dig_H1,int16_t *dig_H2,uint8_t *dig_H3,int16_t *dig_H4,int16_t *dig_H5,uint8_t *dig_H6){

		uint8_t compData[25], compData2[7]; // List to temporary store data
		uint8_t tempPresAddress = 0x88; //From data sheet, it says to pass address, but the 7-bit should be 1 for read. As it already is
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, &tempPresAddress, 1, 1000);
		HAL_SPI_Receive(&hspi1, compData, 25, 1000); // From data sheet it says when reading more than one byte it will read from the next address. This is taken advantage of by reading multiple bytes from one line and adding them to a list.
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay(100);

		uint8_t humAddress = 0xE1; // Here the 7bit already is 1.
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, &humAddress, 1, 1000);
		HAL_SPI_Receive(&hspi1, compData2, 7, 1000);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay(100);

		*dig_T1 = (compData[1] << 8) | compData[0]; // Organize data from list to their corresponding variable as it is in data sheet.
		*dig_T2 = (compData[3] << 8) | compData[2];
		*dig_T3 = (compData[5] << 8) | compData[4];
		*dig_P1 = (compData[7] << 8) | compData[5];
		*dig_P2 = (compData[9] << 8) | compData[6];
		*dig_P3 = (compData[11] << 8) | compData[10];
		*dig_P4 = (compData[13] << 8) | compData[12];
		*dig_P5 = (compData[15] << 8) | compData[14];
		*dig_P6 = (compData[17] << 8) | compData[16];
		*dig_P7 = (compData[19] << 8) | compData[18];
		*dig_P8 = (compData[21] << 8) | compData[20];
		*dig_P9 = (compData[23] << 8) | compData[22];
		*dig_H1 = compData[24];
		*dig_H2 = (compData2[1] << 8) | compData2[0];
		*dig_H3 = (compData2[2]);
		*dig_H4 = (compData2[3] << 4) | (compData2[4] & 0x0f);
		*dig_H5 = (compData2[6] << 4) | (compData2[5] >> 4);
		*dig_H6 = (compData2[7]);

	}

	// Sets Chip select pin to high on init.
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_Delay(100);

	uint8_t activateHumidity[2];
	activateHumidity[0] = 0xF2 & 0b01111111;  //From data sheet, it says to pass address, but the 7-bit should be 0 for write.
	activateHumidity[1] = 0x03; //00000011  // From data sheet its only the first 3 bits that counts. And 011 says its oversampling ×4

	uint8_t activateTempPres[2];
	activateTempPres[0] = 0xF4 & 0b01111111; //
	activateTempPres[1] = 0x27; //00100111 //bit 7,6,5 = temperature sampling (001), bit 4,3,2 = hum sampling (001), bit 1,0 = normal mode (11)

	// Sets chip select to low and sends address and value to initialization the sensor to normal mode with selected samples.
	//  From data sheet the humidity need to be initialization first, and only works after temperature and pressure also has been initialization.
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, activateHumidity, 2, 1000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	HAL_Delay(100);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, activateTempPres, 2, 1000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	HAL_Delay(100);

	// Get compensating numbers
	calibData(&dig_T1, &dig_T2, &dig_T3, &dig_P1, &dig_P2, &dig_P3, &dig_P4, &dig_P5, &dig_P6, &dig_P7, &dig_P8, &dig_P9, &dig_H1, &dig_H2, &dig_H3, &dig_H4, &dig_H5, &dig_H6);

	HAL_Delay(100);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	uint8_t chipAddress = 0xD0 | 0b10000000; //From data sheet, it says to pass address, but the 7-bit should be 1 for read
	uint8_t chipID; // Shall write memory from sensor to this memory address on stm32
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &chipAddress, 1, 1000);
	HAL_SPI_Receive(&hspi1, &chipID, 1, 1000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_Delay(100);

	uint8_t rawDataAddress = 0xF7; // Here the 7bit already is 1.
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &rawDataAddress, 1, 1000);
	HAL_SPI_Receive(&hspi1, rawData, 8, 1000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_Delay(100);

	pRaw = (rawData[0]<<12)|(rawData[1]<<4)|(rawData[2]>>4); // Organize data from list to their corresponding variable as it is in data sheet.
	tRaw = (rawData[3]<<12)|(rawData[4]<<4)|(rawData[5]>>4);
	hRaw = (rawData[6]<<8)|(rawData[7]);

	temp = BME280_compensate_T_int32(tRaw); // Put data in function and receive corrected data
	pressure = BME280_compensate_P_int64(pRaw);
	humid = bme280_compensate_H_int32(hRaw);

	len = sprintf(sendData, "%u,%u,%u\n", temp, pressure, humid); // Turn initialization to string

	HAL_UART_Transmit(&huart2, (uint8_t*) sendData, len, 10); // Sending with UART
	HAL_Delay(100);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS1_Pin */
  GPIO_InitStruct.Pin = CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
