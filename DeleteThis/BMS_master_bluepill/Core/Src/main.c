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
//char rx[500];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/*char hexCharToBin(char c) {
 if (isdigit(c)) {  // 0 - 9
 return c - '0';
 } else if (isxdigit(c)) { // A-F, a-f
 return (c & 0xF) + 9;
 }
 return -1;
 }

 uint64_t stringToUint(char * string)
 {
 uint64_t x = 0;
 char c;
 do {
 c = hexCharToBin(*string++);
 if (c < 0)
 break;
 x = (x << 4) | c;
 } while (1);
 return x;
 }*/
/*int indexOf(char * source, char * key) {
 char * found = strstr( source, key );
 if (found != NULL)
 {
 int index = found - source;
 return index;
 }
 return -1;
 }*/
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

void parseDataIfValid(char *data) {
	slaveNum = extractDataInULL(data);
	if (slaveNum > 0) {
		extractTemperatures();
		extractCells();
		numReadingsReceived++;
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, dataRx, 1);
	timerToSend = 100;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
