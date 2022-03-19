/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ************************* TASK HANDLE ****************************** */
TaskHandle_t Task1Handle;
TaskHandle_t Task2Handle;
TaskHandle_t ReceiverTask;

/* ************************* TASK FUNCTION *************************** */
void TASK1_CONTROL(void *params);
void TASK2_CONTROL(void *params);
void RECEIVER_TASK(void *params);

/* ************************* QUEUE HANDLE ***************************** */
QueueHandle_t SimpleQueue;

/* ************************ STRUCTURE DEFINATION ********************** */
typedef struct
{
	char *str;
	int counter;
	uint16_t largeValue;
}myStruct;

int index1;
int index2;
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
  /* USER CODE BEGIN 2 */

  /* ************************** CREATE QUEUE ************************* */
  SimpleQueue = xQueueCreate(2,sizeof(myStruct));

  if(SimpleQueue != NULL)
  {
	  char *str = "Structure Queue Created Successfully\n\n";
	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
  }
  if(SimpleQueue == NULL)
  {
	  char *str = "Unable to create Structure Queue\n\n";
	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);

  }

  /* ********************* CREATE TASK ****************************** */
  xTaskCreate(TASK1_CONTROL, "Task1", configMINIMAL_STACK_SIZE, NULL, 2, &Task1Handle);
  xTaskCreate(TASK2_CONTROL, "Task2", configMINIMAL_STACK_SIZE, NULL, 2, &Task2Handle);
  xTaskCreate(RECEIVER_TASK, "Task3", configMINIMAL_STACK_SIZE, NULL, 1, &ReceiverTask);

  /* ********************* SCHEDULER ******************************* */
  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void TASK1_CONTROL(void *params)
{
	myStruct *ptrtostruct;
	uint32_t tickDelay = pdMS_TO_TICKS(2000);
	for(;;)
	{
		char *str = "Entered TASK1- about send data to queue\n\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);

		/* ******************* ALLOCATE MEMORY TO THE PTR *********************** */
		ptrtostruct = pvPortMalloc(sizeof(myStruct));

		/* ******************* LOAD DATA TO STRUCTURE ************************** */
		ptrtostruct->counter = 1 + index1;
		ptrtostruct->largeValue = 1000 + index1 *100;
		ptrtostruct->str = "Hello From Task1\n";

		/* ******************* SEND DATA TO QUEUE ***************************** */

		if (xQueueSend(SimpleQueue,&ptrtostruct,portMAX_DELAY) == pdPASS)
		{
			char *str2 = "Successfully sent data to the queue\nLeaving Task1\n\n";
			HAL_UART_Transmit(&huart2, (uint8_t *)str2, strlen(str2), HAL_MAX_DELAY);
		}
		index1 = index1 + 1;

		vTaskDelay(tickDelay);
	}
}

void TASK2_CONTROL(void *params)
{
	myStruct *ptrtostruct;
	uint32_t tickDelay = pdMS_TO_TICKS(2000);
	for(;;)
	{
		char *str = "Entered TASK2- about send data to queue\n\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);

		/* ******************* ALLOCATE MEMORY TO THE PTR *********************** */
		ptrtostruct = pvPortMalloc(sizeof(myStruct));

		/* ******************* LOAD DATA TO STRUCTURE ************************** */
		ptrtostruct->counter = 1 + index2;
		ptrtostruct->largeValue = 2000 + index1 *200;
		ptrtostruct->str = "Hello From Task2\n";

		/* ******************* SEND DATA TO QUEUE ***************************** */

		if (xQueueSend(SimpleQueue,&ptrtostruct,portMAX_DELAY) == pdPASS)
		{
			char *str2 = "Successfully sent data to the queue\nLeaving Task1\n\n";
			HAL_UART_Transmit(&huart2, (uint8_t *)str2, strlen(str2), HAL_MAX_DELAY);
		}
		index2 = index2 + 1;

		vTaskDelay(tickDelay);
	}
}
void RECEIVER_TASK(void *params)
{
	myStruct *Rptrtostruct;
	uint32_t tickDelay = pdMS_TO_TICKS(3000);
	char *ptr;
	for(;;)
	{
		char *chr = "Entered RECEIVER TASK - about receive data from queue";
		HAL_UART_Transmit(&huart2, (uint8_t *)chr, strlen(chr), HAL_MAX_DELAY);

		/* ********************* RECEIVE FROM QUEUE ************************ */
		if(xQueueReceive(SimpleQueue, &Rptrtostruct, portMAX_DELAY) == pdPASS)
		{
			ptr = pvPortMalloc(100 * sizeof(char)); /* allocate memory for string */

			sprintf(ptr,"Received from Queue\nCounter = %d\nLarge Value = %u\nString = %s\n\n\n",Rptrtostruct->counter,
					Rptrtostruct->largeValue,Rptrtostruct->str);
			HAL_UART_Transmit(&huart2, (uint8_t *)ptr, strlen(ptr), HAL_MAX_DELAY);

			vPortFree(ptr); /* String Memory */
		}
		vPortFree(Rptrtostruct); /* free the structure memory */
		vTaskDelay(tickDelay);
	}
}
/* USER CODE END 4 */



 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
