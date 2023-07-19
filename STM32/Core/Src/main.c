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
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define g_TO_MPS2 9.81
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static const uint8_t MPU6050_I2C_ADDR 	= 0xD0;

static const uint8_t REG_WHO_AM_I 		= 0x75;
static const uint8_t REG_PWR_MGMT_1 	= 0x6B;
static const uint8_t REG_ACCEL_XOUT_H	= 0x3B; // Next 5 bytes are ACCEL Y, Z (high and low)
static const uint8_t REG_GYRO_XOUT_H 	= 0x43; // Next 5 bytes are GYRO Y, Z (high and low)
static const uint8_t REG_GYRO_CONFIG  	= 0x1B;
static const uint8_t REG_ACCEL_CONFIG 	= 0x1C;
static const uint8_t REG_SMPRT_DIV 		= 0x19;

static uint8_t VAL_WHO_AM_I 			= 0x68;
static uint8_t VAL_PWR_MGMT_1 			= 0x00;
static uint8_t VAL_GYRO_CONFIG   		= 0x00;
static uint8_t VAL_ACCEL_CONFIG   		= 0x00;
static uint8_t VAL_SMPRT_DIV      		= 0x07;

uint8_t Raw_Accel_Buffer[6];
uint8_t Raw_Gyro_Buffer[6];
int16_t Raw_Accel_X;
int16_t Raw_Accel_Y;
int16_t Raw_Accel_Z;
int16_t Raw_Gyro_X;
int16_t Raw_Gyro_Y;
int16_t Raw_Gyro_Z;
float Ax_raw;
float Ay_raw;
float Az_raw;
float Gyx_raw;
float Gyy_raw;
float Gyz_raw;


void MPU6050_Init(void) {
	uint8_t check=0;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, REG_WHO_AM_I, 1, &check, 1, 1000);

	if (check != VAL_WHO_AM_I) {
		printf("Could not find MPU6050!\r\n");
		return;
	}

	printf("Found!\r\n");

	HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, REG_PWR_MGMT_1, 1, &VAL_PWR_MGMT_1, 1, 1000);
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, REG_SMPRT_DIV, 1, &VAL_SMPRT_DIV, 1, 1000);
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, REG_GYRO_CONFIG, 1, &VAL_GYRO_CONFIG, 1, 1000);
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, REG_ACCEL_CONFIG, 1, &VAL_ACCEL_CONFIG, 1, 1000);

}

void MPU6050_Read_Accel(void) {
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, REG_ACCEL_XOUT_H, 1, Raw_Accel_Buffer, 6, 1000);

	Raw_Accel_X = (int16_t)(Raw_Accel_Buffer[0]<<8 | Raw_Accel_Buffer[1]);
	Raw_Accel_Y = (int16_t)(Raw_Accel_Buffer[2]<<8 | Raw_Accel_Buffer[3]);
	Raw_Accel_Z = (int16_t)(Raw_Accel_Buffer[4]<<8 | Raw_Accel_Buffer[5]);

	Ax_raw = (Raw_Accel_X / 16384.0f)*g_TO_MPS2;
	Ay_raw = (Raw_Accel_Y / 16384.0f)*g_TO_MPS2;
	Az_raw = (Raw_Accel_Z / 16384.0f)*g_TO_MPS2;
}

void MPU6050_Read_Gyro(void) {
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, REG_GYRO_XOUT_H, 1, Raw_Gyro_Buffer, 6, 1000);

	Raw_Gyro_X = (int16_t)(Raw_Gyro_Buffer[0]<<8 | Raw_Gyro_Buffer[1]);
	Raw_Gyro_Y = (int16_t)(Raw_Gyro_Buffer[2]<<8 | Raw_Gyro_Buffer[3]);
	Raw_Gyro_Z = (int16_t)(Raw_Gyro_Buffer[4]<<8 | Raw_Gyro_Buffer[5]);

	Gyx_raw = Raw_Gyro_X / 131.0f;
	Gyy_raw = Raw_Gyro_Y / 131.0f;
	Gyz_raw = Raw_Gyro_Z / 131.0f;
}

int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2,(uint8_t*)&ch,1,HAL_MAX_DELAY);
  return ch;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  MPU6050_Read_Accel();
	  MPU6050_Read_Gyro();
	  printf("%.3f,%.3f,%.3f\r\n", Ax_raw,Ay_raw,Az_raw);
//	  printf("%.3f,%.3f,%.3f\r\n", Gyx_raw,Gyy_raw,Gyz_raw);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
