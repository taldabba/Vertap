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
#include <math.h>
#include <stdbool.h>

#include "FIRfilter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define g_TO_MPS2 9.81f
#define RAD_TO_DEG 57.2958f
#define SAMPLE_TIME_MS 0.01f
#define COMP_FILT_ALPHA 0.05f

#define SCALE_CONSTANT 0.015713484f
#define COUNTER_PWM_LED 255.0f
#define SENSITIVITY_LED 3.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
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

// Raw_ variables hold unconverted value from IMU
int16_t Raw_Accel_X;
int16_t Raw_Accel_Y;
int16_t Raw_Accel_Z;
int16_t Raw_Gyro_X;
int16_t Raw_Gyro_Y;
int16_t Raw_Gyro_Z;

// Variables hold converted values (m/s^2 and deg/s)
float Ax_raw;
float Ay_raw;
float Az_raw;
float Gyx_raw;
float Gyy_raw;
float Gyz_raw;

// Declare low pass filters
FIRfilter lpfAccX;
FIRfilter lpfAccY;
FIRfilter lpfAccZ;

FIRfilter lpfGyrX;
FIRfilter lpfGyrY;
FIRfilter lpfGyrZ;

// Declare euler angles
float phiHat_deg;
float thetaHat_deg;

// Declare red and green pulse width values (duty cycle)
uint8_t greenPulseWidth;
uint8_t redPulseWidth;

// Declare error and its associated variables
float scaledRoll;
float scaledPitch;
float scaledError;

// Declare user set (button) position variables
float userSetPhiPosition_deg;
float userSetThetaPosition_deg;

void MPU6050_Init(void) {
	uint8_t check=0;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, REG_WHO_AM_I, 1, &check, 1, 1000);

	if (check != VAL_WHO_AM_I) {
		printf("Could not find MPU6050!\r\n");
		return;
	}

	printf("Found!\r\n");

	// Power IMU
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, REG_PWR_MGMT_1, 1, &VAL_PWR_MGMT_1, 1, 1000);

	// Set sampling rate of 1 kHz
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, REG_SMPRT_DIV, 1, &VAL_SMPRT_DIV, 1, 1000);

	// Set gyro config
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, REG_GYRO_CONFIG, 1, &VAL_GYRO_CONFIG, 1, 1000);

	// Set accel config
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, REG_ACCEL_CONFIG, 1, &VAL_ACCEL_CONFIG, 1, 1000);

}

void MPU6050_Read_Accel(void) {
	// Read accel values into buffer
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, REG_ACCEL_XOUT_H, 1, Raw_Accel_Buffer, 6, 1000);

	// Seperate buffer into individual accel axis variables
	Raw_Accel_X = (int16_t)(Raw_Accel_Buffer[0]<<8 | Raw_Accel_Buffer[1]);
	Raw_Accel_Y = (int16_t)(Raw_Accel_Buffer[2]<<8 | Raw_Accel_Buffer[3]);
	Raw_Accel_Z = (int16_t)(Raw_Accel_Buffer[4]<<8 | Raw_Accel_Buffer[5]);

	// Units converted from g to m/s^2
	Ax_raw = (Raw_Accel_X / 16384.0f)*g_TO_MPS2;
	Ay_raw = (Raw_Accel_Y / 16384.0f)*g_TO_MPS2;
	Az_raw = (Raw_Accel_Z / 16384.0f)*g_TO_MPS2;
}

void MPU6050_Read_Gyro(void) {
	// Read gyro values
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, REG_GYRO_XOUT_H, 1, Raw_Gyro_Buffer, 6, 1000);

	// Seperate buffer into individual gyro axis variables
	Raw_Gyro_X = (int16_t)(Raw_Gyro_Buffer[0]<<8 | Raw_Gyro_Buffer[1]);
	Raw_Gyro_Y = (int16_t)(Raw_Gyro_Buffer[2]<<8 | Raw_Gyro_Buffer[3]);
	Raw_Gyro_Z = (int16_t)(Raw_Gyro_Buffer[4]<<8 | Raw_Gyro_Buffer[5]);

	// Units in deg/s
	Gyx_raw = Raw_Gyro_X / 131.0f;
	Gyy_raw = Raw_Gyro_Y / 131.0f;
	Gyz_raw = Raw_Gyro_Z / 131.0f;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == BUTTON_Pin) {
		userSetPhiPosition_deg = phiHat_deg;
		userSetThetaPosition_deg = thetaHat_deg;
	}
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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  MPU6050_Init();
  FIR_Filter_Init(&lpfAccX);
  FIR_Filter_Init(&lpfAccY);
  FIR_Filter_Init(&lpfAccZ);

  // Estimate euler angles
  phiHat_deg = 0.0f;
  thetaHat_deg = 0.0f;

  // Initalize user set positions
  userSetPhiPosition_deg = 0.0f;
  userSetThetaPosition_deg = 0.0f;

  // Initalize error and its associated variables
  scaledRoll = 0.0f;
  scaledPitch = 0.0f;
  scaledError = 0.0f;

  // Initalize green and red pulse width value
  greenPulseWidth = 255;
  redPulseWidth = 0;

  // Start PWM output for Green and Red (RGB LED)
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  MPU6050_Read_Accel();
	  MPU6050_Read_Gyro();

	  // Update FIR accel values
	  FIR_Filter_Update(&lpfAccX, Ax_raw);
	  FIR_Filter_Update(&lpfAccY, Ay_raw);
	  FIR_Filter_Update(&lpfAccZ, Az_raw);

	  // Calculate phi (roll) and theta (pitch) estimate values with accel
	  float phiHat_acc_deg = atanf(lpfAccY.output / lpfAccZ.output) * RAD_TO_DEG;
	  float thetaHat_acc_deg = asinf(lpfAccX.output / g_TO_MPS2) * RAD_TO_DEG;

	  // Update FIR gyro values
	  FIR_Filter_Update(&lpfGyrX, Gyx_raw);
	  FIR_Filter_Update(&lpfGyrY, Gyy_raw);
	  FIR_Filter_Update(&lpfGyrZ, Gyz_raw);

	  // Find phi (roll) and theta (pitch) rate of change
	  float phiDot_dps = lpfGyrX.output + tanf(thetaHat_deg) * (sinf(phiHat_deg)*lpfGyrY.output + cosf(phiHat_deg)*lpfGyrZ.output);
	  float thetaDot_dps = cosf(phiHat_deg)*lpfGyrY.output - sinf(phiHat_deg)*lpfGyrZ.output;

	  // Use comp filter to find phi (roll) and theta (pitch) angles
	  phiHat_deg = (COMP_FILT_ALPHA*phiHat_acc_deg) + ((1.0f-COMP_FILT_ALPHA)*(phiHat_deg + SAMPLE_TIME_MS * phiDot_dps));
	  thetaHat_deg = (COMP_FILT_ALPHA*thetaHat_acc_deg) + ((1.0f-COMP_FILT_ALPHA)*(thetaHat_deg + SAMPLE_TIME_MS * thetaDot_dps));

	  // Calculate error
	  scaledRoll = SCALE_CONSTANT * fabs(phiHat_deg - userSetPhiPosition_deg);
	  scaledPitch = SCALE_CONSTANT * fabs(thetaHat_deg - userSetThetaPosition_deg);
	  scaledError = sqrt( pow(scaledRoll,2) + pow(scaledPitch,2) );

	  // If error is beyond 90deg, automatically set error to 1 (assuming +-90deg is the span)
	  if(scaledRoll>1.0f || scaledPitch>1.0f) scaledError=1.0f;

	  // Calculate new LED colour PWM pulse widths
	  greenPulseWidth = (uint8_t)(COUNTER_PWM_LED - (scaledError * COUNTER_PWM_LED * SENSITIVITY_LED));
	  redPulseWidth = (uint8_t)(scaledError * COUNTER_PWM_LED * SENSITIVITY_LED);

	  // Change PWM duty cycles of green and red (change LED colour)
	  if (greenPulseWidth==0) {
		  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
		  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,255);
	  }
	  else {
		  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,greenPulseWidth);
	  	  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,redPulseWidth);
	  }

	  /* DEBUGGING */
//	  printf("%.3f,%.3f\r\n", Ax_raw, lpfAccX.output);
//	  printf("%.3f,%.3f\r\n", Ay_raw, lpfAccY.output);
//	  printf("%.3f,%.3f\r\n", Az_raw, lpfAccZ.output);
//	  printf("%.3f,%.3f,0.000\r\n",phiHat_deg,thetaHat_deg);
//	  printf("%u,%u,%.3f\r\n", greenPulseWidth, redPulseWidth, scaledError);
	  printf("%.3f,%.3f,0.000,%u,%u,%.3f\r\n",phiHat_deg,thetaHat_deg,greenPulseWidth, redPulseWidth, scaledError);
	  // Delay to prevent freezing
	  HAL_Delay(1);

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 255;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 255;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
