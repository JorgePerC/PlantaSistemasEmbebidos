/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include "stdio.h"
#include "PuenteH.h"
#include "MPU6050.h"
#include "i2c-lcd.h"
#include "keypad.h"

// For a proportional control
#define DIRECTION1_PIN GPIO_PIN_14
#define DIRECTION1_PORT GPIOD


#define DIRECTION2_PIN GPIO_PIN_5
#define DIRECTION2_PORT GPIOB

#define K 1

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
	// Declare global structures
		DC_Motor_Controller* motor;
		MPU6050 gyro;
	// Control variables:
		int currentPrr = 0;
		int currentDistance = 0;

		/* First Index -> Input
		 * Second Index -> Sensor
		 * */
		int controlStuffPRR[2];
		float controlStuffDist[2];

	// Send info buffers:
	   char uart_buffer[50];
	   uint8_t uart_buff_len;
	// MPU readings:
	   float readings [3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Control System


// Convert a distance un centimeters to pulses per revolution
int cmToPrr(int distance){
	float prr = distance * (2911.0/15);
	return (int)prr;
}

// Converts pulses per revolution into distance units
int prrToCm(int distance){
	float prr = distance * (15.0/2911);
	return (int)prr;
}

void switchDirection(int diff){
	if(diff > 0){
		// Go Right
		HAL_GPIO_WritePin(DIRECTION1_PORT, DIRECTION1_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIRECTION2_PORT, DIRECTION2_PIN, GPIO_PIN_RESET);


	}
	else if(diff < 0){
		// Go Left
		HAL_GPIO_WritePin(DIRECTION1_PORT, DIRECTION1_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIRECTION2_PORT, DIRECTION2_PIN, GPIO_PIN_SET);
	}

}
void shutdownMotor(){
	TIM3->CCR1=0;
	HAL_GPIO_WritePin(DIRECTION1_PORT, DIRECTION1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DIRECTION2_PORT, DIRECTION2_PIN, GPIO_PIN_RESET);
}

void adjustPosition(int diff){
	int newPrr;
		int placeHolder = 0;
		switchDirection(diff); // Decide Moto Spin Direction
		TIM3->CCR1=90; // set PWM Signal


		// Reset Timer above threshold to avoid cycle counting bug



		// Count the cycles necesary to reach the desired position
		int t1,t2,sum= 0;
		newPrr = cmToPrr(abs(diff));



		if(diff >0){
			while(sum < newPrr){
				if(TIM1->CNT >= 15000){
				  TIM1->CNT = 0;
				}

				t1 = TIM1->CNT;
				//uart_buf_len = sprintf(uart_buf, "%d t1 %d sum\r\n", t1,sum);
				//HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_buf_len, 100);
				HAL_Delay(0.1);
				t2 = TIM1->CNT;
				sum += (t2-t1);
			}
		}
		else{
			while(sum < newPrr){
				if(TIM1->CNT >= 15000){
				  TIM1->CNT = 0;
				}

				t1 = TIM1->CNT;
				//uart_buf_len = sprintf(uart_buf, "%d t1 %d sum\r\n", t1,sum);
				//HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_buf_len, 100);
				HAL_Delay(0.1);
				t2 = TIM1->CNT;
				sum += (t1-t2);
			}

		}


		shutdownMotor();


		// Update sensor position lecture
		if(diff > 0){
			controlStuffPRR[1] += sum;
		}
		else if(diff < 0){
			controlStuffPRR[1] -= sum;

		}

	uart_buff_len = sprintf(uart_buffer, "p1 %d p2 %d sum %d newPPR %d \n", controlStuffPRR[0],controlStuffPRR[1],sum,newPrr);
	HAL_UART_Transmit(&huart3, (uint8_t *)uart_buffer, uart_buff_len, 100);

}

void adjustControl(){
	int Kp = 1;
	int error = controlStuffPRR[0] - controlStuffPRR[1];
	int controller = (int)error*Kp;

	if(error > 0){
		controlStuffPRR[0] -= controller;
	}

	else if(error < 0){
		controlStuffPRR[0] += controller;
	}

}



/* Keypad - LCD Related functions (GUI)
 *
 * */

void writeNumSelect(){
  lcd_reset();
  lcd_send_string("Select a Number:");
  HAL_Delay(500);
}

void writeSelected(char key){
  char b[20];
  //lcd_reset();
  lcd_put_cur(1,0);
  sprintf(b, "Selected %c", key);
  lcd_send_string(b);
  HAL_Delay(500);
}

void append(char* s, char c) {
  int len = strlen(s);
  s[len] = c;
  s[len+1] = '\0';
}

int getKeypadNum(){
	// Define a string of 5 chars
	char chrs[5] = "";
	char key;
	int num;

	do{
		writeNumSelect();
		key = keypad_read();
		if(key != '/'&& key != 'A' && key != '#' && key  != 'D' && key != 'C' && key != 'B' && key != '*'){
			append(chrs, key);
			writeSelected(key);
		}else if(key == 'B' || key == 'C' || key == 'D' || key == '*' || key == '#'){
			lcd_reset();
			lcd_send_string("Not valid!!");
			HAL_Delay(500);
		}
		HAL_Delay(500);
	} while(key != 'A'); //Stop selection once A is pressed.
	return atoi(chrs);
}

// Request a distance from 0 to 16 cm
int getDistanceTarget(){
	// Set 5 char string
	char chrs[25] = "";
	int dist = 0;

	do{

		lcd_reset();
		lcd_put_cur(0,0);
		lcd_send_string("Enter target:");

		lcd_put_cur(1,0);
		sprintf(chrs, "Current: %d cm", currentDistance);
		lcd_send_string(chrs);

		HAL_Delay(1000);

		dist = getKeypadNum();

		lcd_reset();
	} while(dist > 0);

	lcd_reset();
	lcd_send_string("out");
	HAL_Delay(1000);
	//&& (dist >= 0 && dist <= 16)
	return dist;
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
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */


   // Kinda initialize MPU
   gyro.direction = 0xD0;	// Default: 0xD0
   gyro.who_am_I = 0x75;   // Default: 0x75
   gyro.timeout = 1000;    // Default: 1000
   MPU6050_init(&hi2c2, &gyro);


   //Initialize PWM:
   motor->pin_Forward_Group = GPIOD;
   motor->pin_Forward_num = GPIO_PIN_14;
   motor->pin_Reverse_Group = GPIOB;
   motor->pin_Reverse_num = GPIO_PIN_5;
   motor->timer_h = &htim3;
   motor->voltage_Supplied = 8.0;
   DC_Motor_Controller_Init(motor);

   //Initialize Encoder:
   HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

   // Initialize LCD & keypad
   lcd_init ();
   keypad_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   controlStuffPRR[0] = 0;
   controlStuffPRR[1] = 0;

   int i = 0;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*
	  set_Relative_Velocity(motor, 0.5f);

	  //Change the 	way the motor turns.
	  invert_Motor(motor);
	  i = TIM1->CNT;
	  uart_buff_len = sprintf(uart_buffer, "%d clks\r\n", i);
	  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buffer, uart_buff_len, 100);
	  HAL_Delay(1000);
	 */
	  /*
	  lcd_clear();  // clean display
	  lcd_put_cur(0,0);
	  lcd_send_string("Taking samples");
	   */


	  MPU6050_get_acc(&hi2c2, &gyro, readings);
	  uart_buff_len = sprintf(uart_buffer, "%.5f gyro \r\n", readings[0]);
	  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buffer, uart_buff_len, 100);


	  //i = getDistanceTarget();
	  //--------------

	  int newDistance = getKeypadNum();
	  lcd_reset();
	  sprintf(uart_buffer,"Ref: %d", newDistance);
	  lcd_send_string(uart_buffer);

	  int diff = newDistance-currentDistance; // Funciona

	  controlStuffPRR[0] = cmToPrr(newDistance);
	  adjustPosition(diff);

	  currentDistance = prrToCm(controlStuffPRR[1]);


	  lcd_put_cur(1,0);
	  sprintf(uart_buffer,"sensor: %d",currentDistance);
	  lcd_send_string(uart_buffer);

	  i = TIM1->CNT;
	  uart_buff_len = sprintf(uart_buffer, "%d clks\r\n", i);
	  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buffer, uart_buff_len, 100);

	  HAL_Delay(1000);

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x307075B1;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
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
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 239;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Row1_Pin|Row2_Pin|Row3_Pin|Row4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MotorSentido_GPIO_Port, MotorSentido_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Motor_Sentido_2_GPIO_Port, Motor_Sentido_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Row1_Pin Row2_Pin Row3_Pin Row4_Pin */
  GPIO_InitStruct.Pin = Row1_Pin|Row2_Pin|Row3_Pin|Row4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : Col1_Pin */
  GPIO_InitStruct.Pin = Col1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Col1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Col2_Pin PF8 PF9 */
  GPIO_InitStruct.Pin = Col2_Pin|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : MotorSentido_Pin */
  GPIO_InitStruct.Pin = MotorSentido_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MotorSentido_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Motor_Sentido_2_Pin */
  GPIO_InitStruct.Pin = Motor_Sentido_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Motor_Sentido_2_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
