/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/*
 * How the car works (overview)
 * ----------------------------
 * The PC (pc_controller/fin.py) sends a 6-byte packet over Bluetooth (USART1, 9600 baud)
 * every time the gamepad changes:
 *
 *     'y' <throttle>  'x' <steering>  'h' <horn>
 *     0x79  byte 1   0x78  byte 3   0x68  byte 5
 *
 * Y = throttle, X = steering, both -127..127 (Y: forward = +, reverse = -; X: right = +):
 *     0..127   -> positive, value as is
 *     128..254 -> negative, -(value - 127)   (so 128 = -1, 254 = -127)
 *
 * Drive (car-style, in the USART1 callback): the outer wheel gets the throttle, the inner wheel
 * slows down in proportion to the steering (full lock = inner wheel stops, never reverses):
 *     X > 0 (right):  left = Y,                    right = Y * (127 - X) / 127
 *     X < 0 (left):   left = Y * (127 + X) / 127,  right = Y
 * No throttle = no movement. Dead zones, sensitivity and the minimum speed are applied on the PC,
 * so tuning never needs a re-flash.
 * Horn byte: 1 while the gamepad A button is held, else 0.
 *
 * Everything runs in interrupt callbacks (bottom of this file):
 *   USART1 RX  -> decode packet, set motor PWM (TIM3), reverse lights + buzzer
 *   TIM4 (300 ms) -> trigger the HC-SR04 distance sensor, blink while reversing
 *   TIM5 (10 us)  -> end the sensor trigger pulse
 *   TIM2 capture  -> measure the echo pulse width = distance
 * The main while(1) loop only sends debug/status text and sleeps (__WFI).
 */

/* Values from the last Bluetooth packet (written in the USART1 interrupt) */
volatile int y = 0;          // throttle, -127..127 (forward = +, reverse = -)
volatile int x = 0;          // steering, -127..127 (right = +)
volatile int h = 0;          // horn: 1 = A button held -> buzzer on
uint8_t bt_byte[64];         // receive buffer for the Bluetooth UART

/* Flags: an interrupt sets them to 1, the main loop handles them and clears them */
volatile uint8_t new_dist_ready = 0;   // a new distance measurement is ready
volatile uint8_t new_bt_ready = 0;     // a new Bluetooth packet was received

/* HC-SR04 ultrasonic distance sensor (echo measured with TIM2 input capture, 1 tick = 1 us) */
volatile uint32_t t_start = 0;         // timer value at the echo rising edge
volatile uint32_t t_stop = 0;          // timer value at the echo falling edge
volatile uint8_t  edge_flag = 0;       // 0 = waiting for rising edge, 1 = waiting for falling edge
volatile uint32_t echo_width = 0;      // echo pulse length in us
volatile uint32_t distance = 999;      // distance in cm (echo_width / 58)

/* Reversing: lights (PA9) and buzzer blink while both motors run backwards */
volatile uint8_t is_reversing = 0;     // 1 = both motor speeds < 0
volatile uint8_t rev_blink_state = 0;  // 1 = lights/buzzer currently on, toggles every 0.9 s
volatile uint8_t rev_tick_counter = 0; // counts TIM4 ticks (300 ms each) between toggles
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */
void SetSpeedRight(int speed);  // right wheel, -127..127 (H-bridge IN1/IN2, PWM on TIM3 CH1)
void SetSpeedLeft(int speed);   // left wheel,  -127..127 (H-bridge IN3/IN4, PWM on TIM3 CH2)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Sets the reverse lights and the buzzer from the current state (h, is_reversing,
 * rev_blink_state). Called after every Bluetooth packet and on every blink toggle. */
static inline void Update_Reverse_And_Buzzer(void)
{
  // 1. Reverse lights (REVERSE_Pin = PA9): blink ~1 s on / ~1 s off, only while both motors < 0
  if (is_reversing && rev_blink_state)
  {
    HAL_GPIO_WritePin(REVERSE_GPIO_Port, REVERSE_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(REVERSE_GPIO_Port, REVERSE_Pin, GPIO_PIN_RESET);
  }

  // 2. Passive buzzer on PB8 (TIM10 CH1 PWM, 1 MHz / 400 = 2.5 kHz tone):
  //    - A button held (h != 0) -> continuous tone
  //    - reversing              -> tone blinks together with the lights
  //    NOTE: a passive buzzer needs a square wave. Compare 400 with Period 399 = 100% duty
  //    (pin always high), which is probably silent. 200 would give 50% duty. Test on the car.
  if (h != 0 || (is_reversing && rev_blink_state))
  {
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 400);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
  }
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  // Motor PWM: TIM3, 1 MHz tick, Period 126 -> 127 steps, so speed 0..127 maps to 0..100% duty
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   // right motor
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);   // left motor

  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); // distance sensor echo measurement
  HAL_TIM_Base_Start_IT(&htim4);              // 300 ms tick: sensor trigger + reverse blinking

  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);  // buzzer
  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);   // buzzer off at start

  // Debug console on the PC (USART2 = Nucleo USB cable)
  char start_msg[] = "STM32 ready! Waiting for Bluetooth data...\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)start_msg, strlen(start_msg), 100);

  // Start receiving Bluetooth packets; HAL_UARTEx_RxEventCallback runs when a packet has arrived
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, bt_byte, sizeof(bt_byte));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // No control logic here: interrupts do the work and set flags, this loop only sends text.

	  // New Bluetooth packet -> print it on the PC debug console (USART2)
	  if (new_bt_ready)
		  {
			new_bt_ready = 0;
			static char debug_msg[96];
			if (huart2.gState == HAL_UART_STATE_READY)
			{
			  int len = snprintf(debug_msg, sizeof(debug_msg),
				  "BT RX: [%02X %02X %02X %02X %02X %02X] -> Y=%d, X=%d\r\n",
				  bt_byte[0], bt_byte[1], bt_byte[2], bt_byte[3], bt_byte[4], bt_byte[5], y, x);
			  HAL_UART_Transmit_IT(&huart2, (uint8_t*)debug_msg, len);
			}
		  }

		  // New distance -> tell the PC over Bluetooth (fin.py rumbles the gamepad on CLOSE)
		  if (new_dist_ready)
		  {
			new_dist_ready = 0;
			if (huart1.gState == HAL_UART_STATE_READY)
			{
			  static uint8_t is_close = 0;   // hysteresis: CLOSE below 20 cm, CLEAR only above 25 cm
			  if (!is_close && distance > 0 && distance < 20) is_close = 1;
			  else if (is_close && distance > 25)             is_close = 0;
			  HAL_UART_Transmit_IT(&huart1, (uint8_t*)(is_close ? "CLOSE\n" : "CLEAR\n"), 6);
			}
		  }

		  __WFI(); // Sleep CPU until next interrupt
  /* USER CODE END 3 */
    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 31;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 10;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 31;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 126;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 31999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 299;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 31;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 31;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 399;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

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
  huart1.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TRIG_Pin|LD2_Pin|IN1_Pin|REVERSE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN2_Pin|IN4_Pin|IN3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIG_Pin LD2_Pin IN1_Pin REVERSE_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin|LD2_Pin|IN1_Pin|REVERSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IN2_Pin IN4_Pin IN3_Pin */
  GPIO_InitStruct.Pin = IN2_Pin|IN4_Pin|IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4)
  {
    // 1. Start the HC-SR04 trigger pulse: PA1 high now, TIM5 sets it low again after 10 us
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 10);
    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_CC2);
    HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_2);

    // 2. Reverse blinking: toggle every 3 ticks (3 x 300 ms = 0.9 s).
    //    For exactly 1 s: set TIM4 Period = 249 (250 ms) and count to 4.
    if (is_reversing)
    {
      rev_tick_counter++;
      if (rev_tick_counter >= 3) // change to 4 if TIM4 Period = 249
      {
        rev_tick_counter = 0;
        rev_blink_state = !rev_blink_state;
        Update_Reverse_And_Buzzer();
      }
    }
  }
}

/* TIM5 compare, 10 us after the trigger started: end the HC-SR04 trigger pulse */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM5 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_TIM_OC_Stop_IT(htim, TIM_CHANNEL_2);
  }
}

/* TIM2 input capture on the HC-SR04 echo pin (both edges, 1 tick = 1 us).
 * The pin level tells which edge it was, so a noise spike can never swap rising/falling.
 * Pulse width in us / 58 = distance in cm. */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    uint32_t t = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)  // rising edge: echo starts
    {
      t_start = t;
      edge_flag = 1;
    }
    else if (edge_flag)                                        // falling edge: echo ends
    {
      t_stop = t;
      edge_flag = 0;
      echo_width = (t_stop >= t_start) ? (t_stop - t_start)
                                       : (htim->Instance->ARR + 1 - t_start + t_stop);
      distance = echo_width / 58;
      new_dist_ready = 1;
    }
  }
}

/* Clamp a motor speed to the PWM range -127..127 (TIM3 Period = 126 -> CCR 127 = 100%). */
static int Clamp_Speed(int speed)
{
	if (speed > 127)  return 127;
	if (speed < -127) return -127;
	return speed;
}

/* Right wheel. speed > 0 forward, < 0 backward, 0 stop (coast).
 * IN1/IN2 (PA8/PB3) set the H-bridge direction, |speed| is the PWM on TIM3 CH1 (PA6). */
void SetSpeedRight(int speed)
{
	speed = Clamp_Speed(speed);
	if (speed > 0)
	{
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
	}
	else if (speed < 0)
	{
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
	}

	else
	{
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
	}
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, abs(speed));
}

/* Left wheel. Same as SetSpeedRight, but IN3/IN4 (PB5/PB4) and TIM3 CH2 (PA7). */
void SetSpeedLeft(int speed)
{
	speed = Clamp_Speed(speed);
	if (speed > 0)
	{
		HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
	}
	else if (speed < 0)
	{
		HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
	}

	else
	{
		HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
	}
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, abs(speed));
}

/* USART1 (Bluetooth): a packet has arrived (line went idle). Size = number of bytes received.
 * Decodes it, drives the motors, updates lights/buzzer, flags the main loop. */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART1)
	{

		// Decode the 6-byte packet  'y' throttle 'x' steering 'h' horn  (see the overview at the top)
		// Bytes 128..254 are negative speeds: -(value - 127)
		if (Size >= 6 && bt_byte[0] == 'y')
		{
			y = bt_byte[1];
			if (y >= 128)
			{
				y = -1 * (y - 127);
			}
		}

		if (Size >= 6 && bt_byte[2] == 'x')
		{
			x = bt_byte[3];
			if (x >= 128)
			{
				x = -1 * (x - 127);
			}
		}

		// Horn: 1 while the A button is held
		if (Size >= 6 && bt_byte[4] == 'h')
		{
			h = bt_byte[5];
		}

		// Car-style drive: Y = throttle, X = steering. The outer wheel keeps the throttle, the
		// inner wheel slows down in proportion to X (full lock = it stops, it never reverses).
		// No dead zone here: the PC (fin.py) already sends exactly 0 while the stick is centred.
		int left  = Clamp_Speed(y);
		int right = Clamp_Speed(y);
		if(y==0){
			right = (6*x)/10;
			left = -(6*x)/10;

		}
		else if (x > 0)          // steer right: slow the right wheel
		{
		    right = y * (127 - x) / 127;
		}
		else if (x < 0)     // steer left: slow the left wheel
		{
		    left = y * (127 + x) / 127;
		}


		SetSpeedRight(right);
		SetSpeedLeft(left);

		// Reversing = negative throttle (the inner wheel can be 0 at full lock, so check Y, not the wheels)
		uint8_t new_reversing = (-y< 0) ? 1 : 0;
		if (new_reversing && !is_reversing)
		{
			// Reversing just started: lights and buzzer on immediately for the first period
			is_reversing = 1;
			rev_blink_state = 1;
			rev_tick_counter = 0;
		}
		else if (!new_reversing)
		{
			is_reversing = 0;
			rev_blink_state = 0;
			rev_tick_counter = 0;
		}

		Update_Reverse_And_Buzzer();

		new_bt_ready = 1;

		// Clear UART error flags (overrun, noise, framing) and start receiving the next packet
		__HAL_UART_CLEAR_OREFLAG(&huart1);
		__HAL_UART_CLEAR_NEFLAG(&huart1);
		__HAL_UART_CLEAR_FEFLAG(&huart1);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, bt_byte, sizeof(bt_byte));


	}
}
/* On a UART error, reception stops - restart it so the car keeps listening */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, bt_byte, sizeof(bt_byte));
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
