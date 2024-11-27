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
#include "lcd.h"
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t  rx_indx, rx_data[6], rx_buffer[100], transfer_cplt, val_f=0, wr_f=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC2_Init(void);
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
	MX_ADC1_Init();
	MX_SPI1_Init();
	MX_TIM3_Init();
	MX_USART2_UART_Init();
	MX_ADC2_Init();

	Lcd_PortType ports[] = { GPIOA, GPIOB, GPIOA, GPIOA };
	Lcd_PinType pins[] = {GPIO_PIN_1, GPIO_PIN_6, GPIO_PIN_8, GPIO_PIN_6};
	Lcd_HandleTypeDef lcd;

	lcd = Lcd_create(ports, pins, GPIOB, GPIO_PIN_5, GPIOB, GPIO_PIN_4, LCD_4_BIT_MODE);
	Lcd_clear(&lcd);
	Lcd_cursor(&lcd, 0,0);


	uint8_t var1=0x33, var2=0xFF; // Two 8 bit variables to send SPI for DAC, Initial 4 bits are always fixed, make sure to look at datasheet
	uint16_t current_v=0,adc_val=0,counter=0, voltage=0;
	uint32_t Samples=0;
	float current_reading, current_set=0, p_err=0, current_avg=0, c=0, current_sum=0;
	uint8_t var[2]={var1,var2}, count2=0, f=0, cv_flag=0, kp=21;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // Initialize LED to OFF

	HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
	HAL_UART_Receive_IT(&huart2, rx_data, 1);


	while (1)
	{

		Lcd_cursor(&lcd, 0,0);
		Lcd_string(&lcd, "RC:");
		// Rotary turn
		counter=TIM3->CNT; // Rotary encoder is just counter in encoder mode for turns, check rotary data sheet for better understanding

		Lcd_cursor(&lcd, 0,3);
		Lcd_float(&lcd, 1.0*counter/100);

		// Detect rotary press
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET){
			current_set=0.01*counter;
			//Average resistance is different for each current range
			if (current_set<1.1)
				current_v=4095*12*current_set/500;
			else if (current_set<2.1)
				current_v=4095*11*current_set/500;
			else
				current_v=4095*10*current_set/500;
		}


		Lcd_cursor(&lcd, 0,8);
		Lcd_string(&lcd, "SC:");
		Lcd_cursor(&lcd, 0,11);
		Lcd_float(&lcd, current_set);

		// Reading of potential for current from Hall Based sensor
		Samples=0;
		for (int x = 0; x < 150; x++){ //Get 150 samples
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1,20);
			adc_val = HAL_ADC_GetValue(&hadc1);    //Read current sensor values
			HAL_ADC_Stop(&hadc1);
			Samples = Samples + adc_val;  //Add samples together
		}
		adc_val=Samples/150;//Taking Average of Samples

		if (current_set<1.5)
			current_reading=(-adc_val*3.42/4036+1.669)/0.084; // Equation is based on sensitivity and trial & error
		else
			current_reading=(-adc_val*3.0/4095+1.442)/0.084;

		if (current_reading<0){
			current_reading=0;
		}


		// PD controller for both CC and CV
		if (cv_flag==1){
			if ((voltage>1700)|| (voltage<1600))
				kp=10;
			else
				kp=1;

			if ((voltage>1645))
				current_v=current_v+kp;

			else if ((voltage<1645)){
				if (current_v > kp)
					current_v=current_v-kp;
			}
		}
		else{
			if (current_reading-current_set > 0.05){
				if (current_v > 3)
					current_v=current_v-3+0*(current_reading-current_set -p_err);
			}
			else if(current_reading-current_set < 0.05){

				current_v=current_v+3+0*(current_reading-current_set -p_err);
			}
			p_err=current_reading-current_set;
		}

		// Transition condition (0.1 A is the accuracy)
		Lcd_cursor(&lcd, 1,10);
		if (((current_reading+0.1>current_set) && ((voltage>1750) || ((voltage<10) ||current_set<0.08))) || ((f==0) && (count2<41)) )
		{ //CC
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

			Lcd_string(&lcd, "CC");
			count2++;
			f=0;
			if (cv_flag==1){
				if (current_set<1.1)
					current_v=4095*12*current_set/500;
				else if (current_set<2.1)
					current_v=4095*11*current_set/500;
				else
					current_v=4095*10*current_set/500;
				cv_flag=0;
			}

		}
		else if(count2==0 || count2>40)
		{//CV

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

			Lcd_string(&lcd, "CV");
			count2=0;
			f=1;
			cv_flag=1;
		}

		//Measuring potential of lower end of load scaled to 3.3V for ADC, upper end is always 9V
		Samples=0;
		for (int x = 0; x < 150; x++){ //Get 150 samples
			HAL_ADC_Start(&hadc2);
			HAL_ADC_PollForConversion(&hadc2,20);
			adc_val = HAL_ADC_GetValue(&hadc2);    //Read potential values
			HAL_ADC_Stop(&hadc2);
			Samples = Samples + adc_val;  //Add samples together
		}
		adc_val=Samples/150;//Taking Average of Samples
		voltage=adc_val;
		if (current_set==0.00)
			current_v=0;
		if (current_v > 320) // Value of potential (in digital) above which no value would ever be required for any kind of load, thus limiting it to prevent any garbage value
			current_v=320;

		// Transmission to DAC using SPI
		var2=current_v&0xFF;
		var1=current_v/256;

		var1=var1&0x0F;
		var1=var1|0x30;
		var[0]=var1;
		var[1]=var2;
		Lcd_cursor(&lcd, 1,0);
		Lcd_string(&lcd, "AC:");
		Lcd_cursor(&lcd, 1,4);

		// Displaying avg current in some cases to prevent fluctuations
		current_sum=current_reading+current_sum;
		if (c==9){
			current_avg=current_sum/10;
			c=0;
			current_sum=0;
		}
		if (current_avg==0)
			current_avg=current_reading;
		c++;

		if ((current_avg>current_set+0.1) || (current_avg+0.1<current_set) || (!current_reading))
			Lcd_float(&lcd, current_reading);
		else
			Lcd_float(&lcd, current_avg);

		// Transmiting to DAC
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); //LDAC_bar is set while transmission
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); //CS_bar is transitioned to 0 just before transmission starts (falling edge)
		HAL_SPI_Transmit(&hspi1, var, 2, 100); //Higher byte with 1st nibble as the config bits, 2nd one is the data

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); //CS_bar rising edge to signify end of transmission of data
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // LDAC_bar falling edge when not transmitting


		// UART (There is a function near the end of the, RxCplt check that out for more details)
		uint8_t Test[]={48,46,48,48, '\n', '\r'}; //Data to send
		if (val_f==1){
			// Converting numbers to ASCII
			Test[0]=48+current_reading/10*10;
			Test[2]=(10*current_reading);
			Test[2]=48+Test[2]%10;
			Test[3]=100*current_reading;
			Test[3]=48+Test[3]%10;
			HAL_UART_Transmit(&huart2,Test,sizeof(Test),10);
		}

		if ((wr_f==1) && (transfer_cplt==1)){
			// ASCII to numbers
			float c_set_dummy;
			c_set_dummy=(rx_buffer[0]-48)*100+(rx_buffer[2]-48)*10+(rx_buffer[3]-48);
			transfer_cplt=0;
			if ((rx_buffer[0]<52) && (rx_buffer[0]>47) && (rx_buffer[2]>47) && (rx_buffer[3]>47)){
				current_set=c_set_dummy;

				current_set=0.01*(current_set);

				if (current_set<1.1)
					current_v=4095*12*current_set/500;
				else if (current_set<2.1)
					current_v=4095*11*current_set/500;
				else
					current_v=4095*10*current_set/500;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_4;
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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 301;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA6 PA8 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	HAL_UART_Receive_IT(&huart2, rx_data, 1);
//	HAL_UART_Transmit(&huart2, rx_data, 6, 100);
	if (rx_indx==0){
		uint8_t i=0;
		for (i=0; i<100; i++){
			rx_buffer[i]=0;
		}
	}

	if (rx_data[0] != 13){ // 13 is newline character (\r)
		rx_buffer[rx_indx++]= rx_data[0];
	}
	else{
		rx_indx=0;
		transfer_cplt=1;
		HAL_UART_Transmit(&huart2, "\r\n", 2, 100);
		if (!strcmp(rx_buffer, "read")){
			val_f=1;
		}
		else if (!strcmp(rx_buffer, "stop read")){
			val_f=0;
		}
		else if (!strcmp(rx_buffer, "write")){
			wr_f=1;
			uint8_t i=0;
			for (i=0; i<100; i++){
				rx_buffer[i]=0;
			}
			// making it 0.00 when write was typed to prevent setting ascii of "write" as the current
			rx_buffer[0]=48;
			rx_buffer[1]=46;
			rx_buffer[2]=48;
			rx_buffer[3]=48;

		}
		else if (!strcmp(rx_buffer, "stop write")){
			wr_f=0;
		}
	}



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
     ex: printf
     ("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
