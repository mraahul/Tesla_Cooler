/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#define THRESHOLD 20
const int  maxTemp = 20;
const int  minTemp = -20;

/* Private define ------------------------------------------------------------*/
float adcResolution = 4095.0;
float vref = 4200; //Supply voltage of temperature sensor(some offset is kept)
float Tc = 10.0;
float V0 = 500.0;
int currentTemperature = 0;

char speed[20] = { 0 };
char temp[10] = { 0 };
__IO float ADCConvertedValue = 0.0;
__IO float volt = 0.0, temperatureValue = 0.0;

char txData[30];
char rxData[30];
bool led_state = false;

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C2_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void AutoMode(void);
void ManualMode(void);
void IncreaseTemperature(void);
void DecreaseTemperature(void);
void ScreenSend(uint8_t * pData);
void BatteryLife(void);

/* USER CODE END PFP */
void ScreenSend(uint8_t * pData) {
	int length = strlen((char*) pData);	//Calculate string length
	HAL_I2C_Master_Transmit(&hi2c2, 0x72 << 1, pData, length, 10);//Address 0x72
}

void AutoMode(void) {

	if (HAL_ADC_Start(&hadc1) != HAL_OK) {
		printf("HAL_ADC_Start Error. \r\n");
	}
	if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) {
		printf("HAL_ADC_PollForConversion Error. \r\n");
	} else {
		ADCConvertedValue = HAL_ADC_GetValue(&hadc1);
		volt = (ADCConvertedValue * vref) / adcResolution;
		temperatureValue = (volt - V0) / Tc;
//		temperatureValue = (volt * Tc ) - 273.15 - 2.5;   //lm335

	}
	printf("Analog value: %4.2f  Temperature: %4.2f 'C\r\n", ADCConvertedValue,
			temperatureValue);
	sprintf(txData, "Temp: %0.1f 'C\r\n", temperatureValue);

	// Control Temperature
	if (temperatureValue >= currentTemperature) {
		printf("PELTIER ON\r\n");
		HAL_GPIO_WritePin(GPIOB, Peltier_2_Pin | Peltier_1_Pin, GPIO_PIN_RESET); //peltier on ACTIVE LOW
	} else {
		printf("PELTIER OFF\r\n");
		HAL_GPIO_WritePin(GPIOB, Peltier_2_Pin | Peltier_1_Pin, GPIO_PIN_SET); //peltier off
	}

	sprintf(temp, "|-Temp: %0.1f\rSetpoint: %d", temperatureValue,
			currentTemperature);
	ScreenSend((unsigned char*) temp);
	HAL_Delay(200);

	if (HAL_UART_Receive(&huart1, (uint8_t*) rxData, 50, 500)) {
	if (rxData[0] == 'm' && rxData[1] == 'a' && rxData[2] == 'n') {

		ManualMode();
		HAL_UART_Transmit(&huart1, (uint8_t *) txData,
				sprintf(txData, "Manual Mode\n"), 500);
//		rxData[0] = '\0';
//		rxData[1] = '\0';
//		rxData[2] = '\0';
//		rxData[3] = '\0';
	}
	}
}

void ManualMode(void) {
	//		Read the button values
	if (HAL_GPIO_ReadPin(GPIOA, Inc_button_Pin)) {
		IncreaseTemperature();
	} else if (HAL_GPIO_ReadPin(GPIOA, Dec_button_Pin)) {
		DecreaseTemperature();
	} else if (HAL_GPIO_ReadPin(GPIOB, Battery_life_Pin)) {
		BatteryLife();
	} else if (HAL_GPIO_ReadPin(Auto_mode_GPIO_Port, Auto_mode_Pin)) {
		while (1) {
			AutoMode();

			if (HAL_GPIO_ReadPin(GPIOA, Inc_button_Pin)) {
				IncreaseTemperature();
				break;
			} else if (HAL_GPIO_ReadPin(GPIOA, Dec_button_Pin)) {
				DecreaseTemperature();
				break;
			} else if (HAL_GPIO_ReadPin(GPIOB, Battery_life_Pin)) {
				BatteryLife();
				break;
			} else if (HAL_GPIO_ReadPin(GPIOB, Manual_mode_Pin)) {
				ManualMode();
				break;
			}

		}
	}

}

void IncreaseTemperature(void) {
	if (currentTemperature >= maxTemp) {
//		htim1.Instance->CCR1 = maxTemp;
		currentTemperature = maxTemp;
	} else {
		currentTemperature = currentTemperature + 2;
//		htim1.Instance->CCR1 = currentTemperature;
	}
	printf("Increase Temp = %d\r\n", currentTemperature);
	sprintf(speed, "|-Manual Mode\rSetpoint: %i     ", currentTemperature);
	ScreenSend((unsigned char*) speed);

	HAL_Delay(100);

}

void DecreaseTemperature(void) {
	if (currentTemperature <= minTemp) {
//		htim1.Instance->CCR1 = minTemp;
		currentTemperature = minTemp;
	} else {
		currentTemperature = currentTemperature - 2;
		htim1.Instance->CCR1 = currentTemperature;
	}
	printf("Decrease Temp = %d\r\n", currentTemperature);
	sprintf(speed, "|-Manual Mode\rSetpoint: %i     ", currentTemperature);
	ScreenSend((unsigned char*) speed);
	HAL_Delay(100);
}

void BatteryLife(void) {
	ScreenSend((unsigned char*) "|-Manual Mode\rBattery: 100%");
	printf("Battery: 100\r\n");
	HAL_Delay(100);
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* MCU Configuration--------------------------------------------------------*/
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_TIM1_Init();
	MX_I2C2_Init();

	printf("Tesla Cooler. \r\n");
	HAL_GPIO_WritePin(GPIOB, Peltier_2_Pin | Peltier_1_Pin, GPIO_PIN_SET); //peltier off

	while (HAL_I2C_IsDeviceReady(&hi2c2, 0x72 << 1, 2, 10) != HAL_OK) {
	}
#ifdef SETBACKLIGHT
	uint8_t setupData[] = {'|', 128 + 15, '|', 158 + 15, '|', 188 + 20}; //set backlight (R, G, B)
	ScreenSend(setupData);
#endif

	ScreenSend((uint8_t *) "|-Tesla cooler");	//Boot message
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (HAL_UART_Receive(&huart1, (uint8_t*) rxData, 50, 500)) {

			if (rxData[0] == 'u' && rxData[1] == 'p') {

				IncreaseTemperature();
				HAL_UART_Transmit(&huart1, (uint8_t *) txData,
						sprintf(txData, "Increased Temp\n"), 500);
				rxData[0] = '\0';
				rxData[1] = '\0';
				rxData[2] = '\0';
				rxData[3] = '\0';
			} else if (rxData[0] == 'd' && rxData[1] == 'o' && rxData[2] == 'w'
					&& rxData[3] == 'n') {

				DecreaseTemperature();
				HAL_UART_Transmit(&huart1, (uint8_t *) txData,
						sprintf(txData, "Decreased Temp\n"), 500);
				rxData[0] = '\0';
				rxData[1] = '\0';
				rxData[2] = '\0';
				rxData[3] = '\0';
			} else if (rxData[0] == 'b' && rxData[1] == 'a'
					&& rxData[2] == 't') {

				BatteryLife();
				HAL_UART_Transmit(&huart1, (uint8_t *) txData,
						sprintf(txData, "Battery Life\n"), 500);
				rxData[0] = '\0';
				rxData[1] = '\0';
				rxData[2] = '\0';
				rxData[3] = '\0';
			} else if (rxData[0] == 'a' && rxData[1] == 'u' && rxData[2] == 't'
					&& rxData[3] == 'o') {

				AutoMode();
				HAL_UART_Transmit(&huart1, (uint8_t *) txData,
						sprintf(txData, "Automatic Mode\n"), 500);
			} else if (rxData[0] == 'm' && rxData[1] == 'a'
					&& rxData[2] == 'n') {

				ManualMode();
				HAL_UART_Transmit(&huart1, (uint8_t *) txData,
						sprintf(txData, "Manual Mode\n"), 500);
			} else {
				rxData[0] = '\0';
				rxData[1] = '\0';
				rxData[2] = '\0';
				rxData[3] = '\0';
			}
		}
//		ManualMode();
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1
			| RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_I2C2 | RCC_PERIPHCLK_TIM1
			| RCC_PERIPHCLK_ADC12;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
	PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
	PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = { 0 };
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x2000090E;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
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
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 0;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
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

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOF_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, Peltier_2_Pin | Peltier_1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Inc_button_Pin Dec_button_Pin */
	GPIO_InitStruct.Pin = Inc_button_Pin | Dec_button_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : Manual_mode_Pin Battery_life_Pin */
	GPIO_InitStruct.Pin = Manual_mode_Pin | Battery_life_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : Auto_mode_Pin */
	GPIO_InitStruct.Pin = Auto_mode_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Auto_mode_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Peltier_2_Pin Peltier_1_Pin */
	GPIO_InitStruct.Pin = Peltier_2_Pin | Peltier_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
void assert_failed(char *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
