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
int main(void) {
	/* USER CODE BEGIN 1 */
	uint16_t dig_T1 = 0;
	int16_t dig_T2 = 0;
	int16_t dig_T3 = 0;
	uint16_t dig_P1 = 0;
	int16_t dig_P2 = 0;
	int16_t dig_P3 = 0;
	int16_t dig_P4 = 0;
	int16_t dig_P5 = 0;
	int16_t dig_P6 = 0;
	int16_t dig_P7 = 0;
	int16_t dig_P8 = 0;
	int16_t dig_P9 = 0;
	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	uint8_t dig_H6;
	uint8_t trimdata[25];
	uint8_t humiddata[7];
	uint8_t RawData[8];
	int32_t tRaw;
	int32_t pRaw;
	int32_t hRaw;
	char SendData[80];
	int32_t t_fine;
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

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_Delay(100);

	uint8_t active_h[2];
	active_h[0] = 0xF2 & ~ 0b10000000;
	active_h[1] = 0x03;

	uint8_t active_t_p[2];
	active_t_p[0] = 0xF4 & ~ 0b10000000;
	active_t_p[1] = 0x27; //(0x03 << 5) | (0x05 << 2) | 0x03:
	//write to humidity

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, active_h, 2, 1000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	HAL_Delay(100);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, active_t_p, 2, 1000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	HAL_Delay(100);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		uint8_t chip = 0xD0 | 0b10000000;
		uint8_t test;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, &chip, 1, 1000);
		HAL_SPI_Receive(&hspi1, &test, 1, 1000);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay(100);

		uint8_t raw_data = 0xF7;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, &raw_data, 1, 1000);
		HAL_SPI_Receive(&hspi1, &RawData, 8, 1000);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay(100);


		uint8_t temp_pres_1 = 0x88;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, &temp_pres_1, 1, 1000);
		HAL_SPI_Receive(&hspi1, &trimdata, 25, 1000);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay(100);


		uint8_t temp_pres_2 = 0xE1;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, &temp_pres_2, 1, 1000);
		HAL_SPI_Receive(&hspi1, &humiddata, 7, 1000);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay(100);


		dig_T1 = (trimdata[1] << 8) | trimdata[0];
		dig_T2 = (trimdata[3] << 8) | trimdata[2];
		dig_T3 = (trimdata[5] << 8) | trimdata[4];
		dig_P1 = (trimdata[7] << 8) | trimdata[5];
		dig_P2 = (trimdata[9] << 8) | trimdata[6];
		dig_P3 = (trimdata[11] << 8) | trimdata[10];
		dig_P4 = (trimdata[13] << 8) | trimdata[12];
		dig_P5 = (trimdata[15] << 8) | trimdata[14];
		dig_P6 = (trimdata[17] << 8) | trimdata[16];
		dig_P7 = (trimdata[19] << 8) | trimdata[18];
		dig_P8 = (trimdata[21] << 8) | trimdata[20];
		dig_P9 = (trimdata[23] << 8) | trimdata[22];
		dig_H1 = trimdata[24];
		dig_H2 = (humiddata[1] << 8) | humiddata[0];
		dig_H3 = (humiddata[2]);
		dig_H4 = (humiddata[3] << 4) | (humiddata[4] & 0x0f);
		dig_H5 = (humiddata[6] << 4) | (humiddata[5] >> 4);
		dig_H6 = (humiddata[7]);

		pRaw = (RawData[0]<<12)|(RawData[1]<<4)|(RawData[2]>>4);
		tRaw = (RawData[3]<<12)|(RawData[4]<<4)|(RawData[5]>>4);
		hRaw = (RawData[6]<<8)|(RawData[7]);



		int temp = BME280_compensate_T_int32(tRaw);
		int pressure = BME280_compensate_P_int64(pRaw);
		int humid = bme280_compensate_H_int32(hRaw);

		int len = sprintf(SendData, "%u,%u,%u \r\n", temp, pressure, humid);

		HAL_UART_Transmit(&huart2, (uint8_t*) SendData, len, 10); // Sending in normal mode
		HAL_Delay(1000);

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

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
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
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
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CS_Pin_GPIO_Port, CS_Pin_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : CS_Pin_Pin */
	GPIO_InitStruct.Pin = CS_Pin_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CS_Pin_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
