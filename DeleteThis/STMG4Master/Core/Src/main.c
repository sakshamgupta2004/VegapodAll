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
#include<string.h>
#include<stdlib.h>
#include <stdbool.h>
#include <ctype.h>
#include<math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_SLAVES 20

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;
ADC_HandleTypeDef hadc5;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char dataRx[1];
char *received;
const char *tempString = "Temp: ";
const char *cellString = "Cell: ";
const char *balanceString = "Balancing: ";

uint64_t receivedVals[MAX_SLAVES][3];
float temp[MAX_SLAVES * 6];
uint16_t cells[MAX_SLAVES * 6];
bool balancing[MAX_SLAVES * 6];
uint8_t slaveNum;
int timerToSend = -1;
int numReadingsReceived = 0;
double totalV = 0;
float SOC = 0;
long adc = 0;
double currentma;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */
int indexOf(char *source, char *key, int startIndex) {
	char *found = strstr((source + startIndex), key);
	if (found != NULL) {
		int index = found - source;
		return index;
	}
	return -1;
}
int lastIndexOf(char *source, char *key) {
	int i = -1;
	int index = -1;
	do {
		i = indexOf(source, key, i + 1);
		if (i != -1) {
			index = i;
		}
	} while (i != -1);
	return index;
}
bool startsWith(char *source, char *key) {
	return indexOf(source, key, 0) == 0;
}
char* substring(char *s, int a, int b) {
	char *t = malloc(sizeof(char) * ((b - a) + 1));
	for (int z = 0; z < (b - a) + 1; z++) {
		t[z] = 0x00;
	}
	strncpy(t, s + a, b - a);
	return t;
}
char* substring1(char *s, int a) {
	char *t = malloc(sizeof(char) * ((strlen(s) - a) + 1));
	for (int z = 0; z < ((strlen(s) - a) + 1); z++) {
		t[z] = 0x00;
	}
	strcpy(t, s + a);
	free(s);
	s = NULL;
	return t;
}
uint8_t extractDataInULL(char *serialIn) {
	uint8_t slaveNum = 0;
	if (indexOf(serialIn, "</C>", 0) != -1 && indexOf(serialIn, "<T>", 0) != -1
			&& (lastIndexOf(serialIn, "</T>\r\n") == (strlen(serialIn) - 6)
					|| lastIndexOf(serialIn, "</T>\n") == (strlen(serialIn) - 5))
			&& startsWith(serialIn, "<C>")) {
		uint8_t ReceivedCellCount = 0, ReceivedBalanceCount = 0,
				ReceivedTemperatureCount = 0;
		char *vals = substring(serialIn, indexOf(serialIn, "<C>", 0) + 3,
				indexOf(serialIn, "</C>", 0));
		serialIn = substring1(serialIn, indexOf(serialIn, "<B>", 0));
		while (indexOf(vals, ",", 0) != -1) {
			char *val = substring(vals, 0, indexOf(vals, ",", 0));
			receivedVals[ReceivedCellCount][0] = strtoull(val, NULL, 16);
			free(val);
			val = NULL;
			vals = substring1(vals, indexOf(vals, ",", 0) + 1);
			ReceivedCellCount++;
		}
		receivedVals[ReceivedCellCount][0] = strtoull(vals, NULL, 16);
		ReceivedCellCount++;

		free(vals);
		vals = substring(serialIn, indexOf(serialIn, "<B>", 0) + 3,
				indexOf(serialIn, "</B>", 0));
		serialIn = substring1(serialIn, indexOf(serialIn, "<T>", 0));
		while (indexOf(vals, ",", 0) != -1) {
			char *val = substring(vals, 0, indexOf(vals, ",", 0));
			receivedVals[ReceivedBalanceCount][1] = strtoull(val, NULL, 16);
			free(val);
			val = NULL;
			vals = substring1(vals, indexOf(vals, ",", 0) + 1);
			ReceivedBalanceCount++;
		}
		receivedVals[ReceivedBalanceCount][1] = strtoull(vals, NULL, 16);
		ReceivedBalanceCount++;

		free(vals);
		vals = substring(serialIn, indexOf(serialIn, "<T>", 0) + 3,
				indexOf(serialIn, "</T>", 0));
		free(serialIn);
		serialIn = NULL;
		while (indexOf(vals, ",", 0) != -1) {
			char *val = substring(vals, 0, indexOf(vals, ",", 0));
			receivedVals[ReceivedTemperatureCount][2] = strtoull(val, NULL, 16);
			free(val);
			val = NULL;
			vals = substring1(vals, indexOf(vals, ",", 0) + 1);
			ReceivedTemperatureCount++;
		}
		receivedVals[ReceivedTemperatureCount][2] = strtoull(vals, NULL, 16);
		ReceivedTemperatureCount++;

		free(vals);
		vals = NULL;
		if (ReceivedCellCount != ReceivedBalanceCount
				|| ReceivedCellCount != ReceivedTemperatureCount) {
			ReceivedCellCount = 0;
			ReceivedBalanceCount = 0;
			ReceivedTemperatureCount = 0;
		}
		slaveNum = ReceivedCellCount;

	}

	/*  vals = serialIn.substring(serialIn.indexOf("<B>") + 3, serialIn.indexOf("</B>"));
	 serialIn = serialIn.substring(serialIn.indexOf("<T>"));

	 while (vals.indexOf(',') != -1) {
	 receivedVals[ReceivedBalanceCount][1] = stringToUint(vals.substring(0, vals.indexOf(',')).c_str());
	 vals = vals.substring(vals.indexOf(',') + 1);
	 ReceivedBalanceCount++;
	 }
	 receivedVals[ReceivedBalanceCount][1] = stringToUint(vals.c_str());
	 ReceivedBalanceCount++;

	 vals = serialIn.substring(serialIn.indexOf("<T>") + 3, serialIn.indexOf("</T>"));
	 serialIn = "";

	 while (vals.indexOf(',') != -1) {
	 receivedVals[ReceivedTemperatureCount][2] = stringToUint(vals.substring(0, vals.indexOf(',')).c_str());
	 vals = vals.substring(vals.indexOf(',') + 1);
	 ReceivedTemperatureCount++;
	 }
	 receivedVals[ReceivedTemperatureCount][2] = stringToUint(vals.c_str());
	 ReceivedTemperatureCount++;

	 if (ReceivedCellCount != ReceivedBalanceCount || ReceivedCellCount != ReceivedTemperatureCount) {
	 ReceivedCellCount = 0;
	 ReceivedBalanceCount = 0;
	 ReceivedTemperatureCount = 0;
	 }*/
	//  slaveNum = ReceivedCellCount;
	//}
	return slaveNum;
}

void extractTemperatures() {
	for (int i = 0; i < MAX_SLAVES * 6; i++) {
		temp[i] = 0;
	}
	for (int i = 0; i < slaveNum; i++) {
		for (int j = 0; j < 6; j++) {
			temp[(i * 6) + j] = ((float)((receivedVals[i][2] >> j * 10) & 0x3FF))/10.0;
		}
	}
	for (int i = 0; i < MAX_SLAVES * 6; i++) {
		//Serial.print("Temp " + String(i+1) + ": ");
		//HAL_UART_Transmit(&huart1, tempString, strlen(tempString), 10);
		char d[5];
		sprintf(d, "%d\n", temp[i]);
		//HAL_UART_Transmit(&huart1, d, strlen(d), 10);
		// Serial.println(temp[i]);
	}
}

void extractCells() {
	for (int i = 0; i < MAX_SLAVES * 6; i++) {
		cells[i] = 0;
		balancing[i] = 0;
	}
	for (int i = 0; i < slaveNum; i++) {
		for (int j = 0; j < 6; j++) {
			cells[(i * 6) + j] =
					(((receivedVals[i][0] >> j * 10) & 0x3FF) * 5.0) / 1.023;
			balancing[(i * 6) + j] = (receivedVals[i][1] >> j) & 0x01;
		}
	}
	for (int i = 0; i < MAX_SLAVES * 6; i++) {

		char d[30];
		sprintf(d, "%s%d\t%s%d\n", cellString, cells[i], balanceString,
				balancing[i]);
		//HAL_UART_Transmit(&huart1, d, strlen(d), 50);
		// Serial.println("Cell " + String(i+1) + ": " + String(cells[i]) + "\t Balancing: " + balancing[i]);
	}
}

void calcSOC(){
	totalV = 0;
	for (int i = 0; i < MAX_SLAVES * 6; i++) {
			totalV += cells[i]/1000.0;
		}
	SOC = ((cbrt((totalV/((double)slaveNum*6.0))-3.70000)+0.9)/1.7)*100;
}

void parseDataIfValid(char *data) {
	slaveNum = extractDataInULL(data);
	if (slaveNum > 0) {
		extractTemperatures();
		extractCells();
		calcSOC();
		numReadingsReceived++;
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
GPIO_PinState a,b;




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (received == NULL) {
		received = malloc(sizeof(char) * 2);
		received[0] = 0x00;
		received[1] = 0x00;
	} else {
		received = realloc(received, sizeof(char) * (strlen(received) + 2));
	}

	strcat(received, dataRx);
	if (dataRx[0] == '\n') {
		/*for (int i = 0; i<500; i++)
		 rx[i] = 0x00;
		 strcpy(rx, received);*/
		parseDataIfValid(received);
		//HAL_UART_Transmit(&huart1, received, strlen(received), strlen(received)*2);
		free(received);
		received = NULL;
		timerToSend = 100;
	} else if (strlen(received) > 1000) {
		free(received);
		received = NULL;
	}
	HAL_UART_Receive_IT(&huart1, dataRx, 1);

}

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
  MX_ADC5_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
	//HAL_UART_Receive_IT(&huart1, dataRx, 1);
	timerToSend = 100;
	//HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	//HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 2);
		adc = HAL_ADC_GetValue(&hadc1);
		currentma = (adc*1.07642626)/46.0;

		HAL_ADC_Stop(&hadc1);
		HAL_Delay(100);


		//HAL_Delay(5000);
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
	 // HAL_ADC_Start(&hadc5);
	 // HAL_ADC_PollForConversion(&hadc5, 1);
	 // adc = HAL_ADC_GetValue(&hadc5);
	 // a=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
	 // b=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
	//  HAL_UART_Transmit(&huart1, "Hi\n", 3, 3);
	//  adc++;

	 // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
	  /*for (int i = 0; i < 30000000; i++) {
		  asm("nop");
	  }*/
	//  HAL_Delay(1000);


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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.GainCompensation = 0;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = ENABLE;
  hadc3.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256;
  hadc3.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
  hadc3.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc3.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief ADC5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC5_Init(void)
{

  /* USER CODE BEGIN ADC5_Init 0 */

  /* USER CODE END ADC5_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC5_Init 1 */

  /* USER CODE END ADC5_Init 1 */

  /** Common config
  */
  hadc5.Instance = ADC5;
  hadc5.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc5.Init.Resolution = ADC_RESOLUTION_12B;
  hadc5.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc5.Init.GainCompensation = 0;
  hadc5.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc5.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc5.Init.LowPowerAutoWait = DISABLE;
  hadc5.Init.ContinuousConvMode = ENABLE;
  hadc5.Init.NbrOfConversion = 1;
  hadc5.Init.DiscontinuousConvMode = DISABLE;
  hadc5.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc5.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc5.Init.DMAContinuousRequests = DISABLE;
  hadc5.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc5.Init.OversamplingMode = ENABLE;
  hadc5.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256;
  hadc5.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
  hadc5.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc5.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR_ADC5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC5_Init 2 */

  /* USER CODE END ADC5_Init 2 */

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
  huart1.Init.BaudRate = 40000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB2 PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
