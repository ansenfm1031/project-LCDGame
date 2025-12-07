/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Headers.h"
#include "my_lcd_i2c.h"
#include "joystick.h"
#include "buzzer.h"
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

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for JoyStickTask */
osThreadId_t JoyStickTaskHandle;
const osThreadAttr_t JoyStickTask_attributes = {
  .name = "JoyStickTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for BGMTask */
osThreadId_t BGMTaskHandle;
const osThreadAttr_t BGMTask_attributes = {
  .name = "BGMTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
Joystick_HandleTypeDef hjs;
Joystick_Tracker_t tracker = { 0 };

extern TIM_HandleTypeDef htim2;
Buzzer_HandleTypeDef myBuzzer;

uint8_t object_moving_distance = 1;

#define RX_BUFFER_SIZE 128
uint8_t uart1_rx_buf;
uint8_t uart2_rx_buf;
char rx1_buffer[RX_BUFFER_SIZE];
char rx2_buffer[RX_BUFFER_SIZE];
uint8_t rx1_index = 0;
uint8_t rx2_index = 0;

// for SCENE_CLEAR
int8_t minX = 0;
int8_t maxX = 80;
int8_t minY = 0;
int8_t maxY = 16;
OBJECT head;
OBJECT tail;
POS target_pos;

// for SCENE_MENU
char *menus[MENU_COUNT] =
{
	"0. Shooting",
	"1. TBD",
	"2. TBD",
	"3. TBD",
	"4. Settings",
};
uint8_t cur_menu_select = 0;
uint8_t menu_scroll_offset = 0;

// for SCENE_SHOOTING
OBJECT	player;
OBJECT	bullets[BULLET_MAX_COUNT];
uint8_t shooting_game_level = 1;
uint8_t bullet_moving_distance = 2;
uint8_t	bullet_size = 3;
OBJECT	walls[WALL_MAX_COUNT];
uint8_t	wall_delay = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

/* USER CODE BEGIN PFP */
void SetPixelByPos(POS _pos);
void ResetPixelByPos(POS _pos);
void SetPixelByXY(int8_t _x, int8_t _y);
void ResetPixelByXY(int8_t _x, int8_t _y);

void ResetPixel();
void SetPixel();

void MoveByDir(OBJECT *_obj);
void MovePlayer(int8_t _dx, int8_t _dy);
void FireBullet(const OBJECT *_player);
void SummonBullet(uint8_t _cmd);
void CheckHitBullet();
void SummonWall();
void CheckHitWall();

void Init();
void InitSceneClear();
void InitSceneMenu();
void InitSceneWaiting();
void InitSceneShooting();
void InitSceneResult();

void Update();
void UpdateSceneClear();
void UpdateSceneMenu();
void UpdateSceneWaiting();
void UpdateSceneShooting();
void UpdateSceneResult();
void UpdateBullets();
void UpdateWalls();

void RenderPixel();
void Render();
void RenderSceneMenu();
void RenderSceneWaiting();
void RenderSceneResult();

void PlayBGM();

void ChangeScene();
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);
  lcd_init(&hi2c1);
  Joystick_Init(&hjs, &hadc1, ADC_CHANNEL_0, &hadc1, ADC_CHANNEL_1, GPIOC,GPIO_PIN_0);
  buzzer_init(&myBuzzer, &htim2, TIM_CHANNEL_1, GPIOA, GPIO_PIN_15);

  HAL_UART_Receive_IT(&huart1, &uart1_rx_buf, 1);
  HAL_UART_Receive_IT(&huart2, &uart2_rx_buf, 1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of JoyStickTask */
  JoyStickTaskHandle = osThreadNew(StartTask02, NULL, &JoyStickTask_attributes);

  /* creation of BGMTask */
  BGMTaskHandle = osThreadNew(StartTask03, NULL, &BGMTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.BaudRate = 38400;
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
  huart2.Init.BaudRate = 38400;
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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		if (rx1_index < RX_BUFFER_SIZE - 1)
		{
			rx1_buffer[rx1_index++] = (char) uart1_rx_buf;
		}

		if (rx1_buffer[rx1_index - 2] == '\r' && rx1_buffer[rx1_index - 1] == '\n')
		{
			uint8_t temp = (uint8_t)atoi(rx1_buffer);

			if (temp < 0x40)
				SummonBullet(temp);
			else if (temp < 0x80)
			{
				if (g_cur_scene == SCENE_WAITING)
				{
					if ((temp & 0x0F) == cur_menu_select)
						g_flag = temp;
				}
			}
			else if (temp == 0x80)
			{
				g_flag = 0xC0;
				g_tick = 0;
			}

			memset(rx1_buffer, 0, RX_BUFFER_SIZE);
			rx1_index = 0;
		}

		HAL_UART_Receive_IT(&huart1, &uart1_rx_buf, 1);
	}
	else if (huart->Instance == USART2)
	{
		if (rx2_index < RX_BUFFER_SIZE - 1)
		{
			rx2_buffer[rx2_index++] = (char) uart2_rx_buf;
		}

		if (rx2_buffer[rx2_index - 2] == '\r' && rx2_buffer[rx2_index - 1] == '\n')
		{
			uint8_t temp = (uint8_t)atoi(rx2_buffer);

			if (temp < 0x40)
				SummonBullet(temp);
			else if (temp < 0x80)
			{
				if (g_cur_scene == SCENE_WAITING)
				{
					if ((temp & 0x0F) == cur_menu_select)
					{
						if (cur_menu_select == 0)
							g_flag = temp;
					}
				}
			}
			else if (temp == 0x80)
			{
				g_flag = 0xC0;
				g_tick = 0;
			}

			memset(rx2_buffer, 0, RX_BUFFER_SIZE);
			rx2_index = 0;
		}
		HAL_UART_Receive_IT(&huart2, &uart2_rx_buf, 1);
	}
}

void SetPixelByPos(POS _pos)
{
	if (_pos.x < LCD_MIN_X || LCD_MAX_X <= _pos.x || _pos.y < LCD_MIN_Y || LCD_MAX_Y <= _pos.y)
		return;

	map[_pos.x / 5 * 10 + _pos.y / 8 * 5 + _pos.x % 5] |= (0x01 << (7 - (_pos.y % 8)));
}
void ResetPixelByPos(POS _pos)
{
	if (_pos.x < LCD_MIN_X || LCD_MAX_X <= _pos.x || _pos.y < LCD_MIN_Y || LCD_MAX_Y <= _pos.y)
		return;

	map[_pos.x / 5 * 10 + _pos.y / 8 * 5 + _pos.x % 5] &= ~(0x01 << (7 - (_pos.y % 8)));
}
void SetPixelByXY(int8_t _x, int8_t _y)
{
	if (_x < LCD_MIN_X || LCD_MAX_X <= _x || _y < LCD_MIN_Y || LCD_MAX_Y <= _y)
		return;

	map[_x / 5 * 10 + _y / 8 * 5 + _x % 5] |= (0x01 << (7 - (_y % 8)));
}
void ResetPixelByXY(int8_t _x, int8_t _y)
{
	if (_x < LCD_MIN_X || LCD_MAX_X <= _x || _y < LCD_MIN_Y || LCD_MAX_Y <= _y)
		return;

	map[_x / 5 * 10 + _y / 8 * 5 + _x % 5] &= ~(0x01 << (7 - (_y % 8)));
}

void ResetPixel()
{
	switch (g_cur_scene)
	{
		case SCENE_SHOOTING:
		{
			if ((g_flag & 0x80) == 0)
			{
				// player
				for (int8_t i = -2; i <= 2; i++)
				{
					for (int8_t j = -2; j <= 2; j++)
					{
						int8_t x = player.pos.x + i;
						int8_t y = player.pos.y + j;

						if (x < LCD_MIN_X || LCD_MAX_X <= x || y < LCD_MIN_Y || LCD_MAX_Y <= y)
							continue;

						ResetPixelByXY(x, y);
					}
				}

				// bullets
				for (int i = 0; i < BULLET_MAX_COUNT; i++)
				{
					if ((bullets[i].dir & DIR_RUNNING) == 0)
						continue;

					if ((bullets[i].dir & DIR_RIGHT) > 0)
					{
						for (int j = 0; j <= bullet_size; j++)
						{
							ResetPixelByXY(bullets[i].pos.x - bullet_moving_distance - j, bullets[i].pos.y);
						}
					}
					else if ((bullets[i].dir & DIR_LEFT) > 0)
					{
						for (int j = 0; j <= bullet_size; j++)
						{
							ResetPixelByXY(bullets[i].pos.x + bullet_moving_distance + j, bullets[i].pos.y);
						}
					}
				}

				// walls
				for (int i = 0; i < WALL_MAX_COUNT; i++)
				{
					if ((walls[i].dir & DIR_RUNNING) == 0)
						continue;

					if ((walls[i].dir & DIR_RIGHT) > 0)
					{
						for (int j = 0; j < 8; j++)
						{
							ResetPixelByXY(walls[i].pos.x - 1, walls[i].pos.y + j);
						}
					}
					else if ((walls[i].dir & DIR_LEFT) > 0)
					{
						for (int j = 0; j < 8; j++)
						{
							ResetPixelByXY(walls[i].pos.x + 1, walls[i].pos.y + j);
						}
					}
				}
			}

			break;
		}
		default:
		{
			break;
		}
	}
}
void SetPixel()
{
	switch (g_cur_scene)
	{
		case SCENE_SHOOTING:
		{
			if ((g_flag & 0x80) == 0)
			{
				// player
				for (int8_t i = -1; i <= 1; i++)
				{
					for (int8_t j = -1; j <= 1; j++)
					{
						int8_t x = player.pos.x + i;
						int8_t y = player.pos.y + j;

						if (x < LCD_MIN_X || LCD_MAX_X <= x || y < LCD_MIN_Y || LCD_MAX_Y <= y)
							continue;

						if (((player.dir & DIR_RIGHT) > 0 && i == 1 && j != 0)
								|| ((player.dir & DIR_LEFT) > 0 && i == -1 && j != 0))
							continue;

						SetPixelByXY(x, y);
					}
				}

				// bullets
				for (int i = 0; i < BULLET_MAX_COUNT; i++)
				{
					if ((bullets[i].dir & DIR_RUNNING) == 0)
						continue;

					if ((bullets[i].dir & DIR_RIGHT) > 0)
					{
						for (int j = 0; j <= bullet_size; j++)
						{
							SetPixelByXY(bullets[i].pos.x - j, bullets[i].pos.y);
						}
					}
					else if ((bullets[i].dir & DIR_LEFT) > 0)
					{
						for (int j = 0; j <= bullet_size; j++)
						{
							SetPixelByXY(bullets[i].pos.x + j, bullets[i].pos.y);
						}
					}
				}

				// walls
				for (int i = 0; i < WALL_MAX_COUNT; i++)
				{
					if ((walls[i].dir & DIR_RUNNING) == 0)
						continue;

					for (int j = 0; j < 8; j++)
					{
						SetPixelByXY(walls[i].pos.x, walls[i].pos.y + j);
					}
				}
			}

			break;
		}
		default:
		{
			break;
		}
	}
}

void MoveByDir(OBJECT *_obj)
{
	if ((_obj->dir & 0x03) == DIR_UP)			_obj->pos.y -= object_moving_distance;
	else if ((_obj->dir & 0x03) == DIR_RIGHT)	_obj->pos.x += object_moving_distance;
	else if ((_obj->dir & 0x03) == DIR_DOWN)	_obj->pos.y += object_moving_distance;
	else if ((_obj->dir & 0x03) == DIR_LEFT)	_obj->pos.x -= object_moving_distance;
}
void MovePlayer(int8_t _dx, int8_t _dy)
{
	player.pos.x = max(PLAYABLE_MIN_X, min(PLAYABLE_MAX_X, player.pos.x + _dx));
	player.pos.y = max(PLAYABLE_MIN_Y, min(PLAYABLE_MAX_Y, player.pos.y + _dy));

	if (_dx > 0)
	{
		// look right
		player.dir &= ~DIR_LEFT;
		player.dir |= DIR_RIGHT;
	}
	else if (_dx < 0)
	{
		// look left
		player.dir &= ~DIR_RIGHT;
		player.dir |= DIR_LEFT;
	}
}
void FireBullet(const OBJECT *_player)
{
	for (int i = 0; i < BULLET_MAX_COUNT / 2; i++)
	{
		if ((bullets[i].dir & DIR_RUNNING) == 0)
		{
			bullets[i].dir |= DIR_RUNNING | _player->dir;

			bullets[i].pos = _player->pos;
			if ((bullets[i].dir & DIR_RIGHT) > 0)
			{
				bullets[i].pos.x += 3;
			}
			else if ((bullets[i].dir & DIR_LEFT) > 0)
			{
				bullets[i].pos.x -= 3;
			}

			break;
		}
	}
}
void SummonBullet(uint8_t _cmd)
{
	for (int i = BULLET_MAX_COUNT / 2; i < BULLET_MAX_COUNT; i++)
	{
		if ((bullets[i].dir & DIR_RUNNING) == 0)
		{
			bullets[i].dir |= DIR_RUNNING;

			if ((((_cmd & 0x30) >> 4) & DIR_RIGHT) > 0)
			{
				bullets[i].pos.x = LCD_MAX_X - 1;
				bullets[i].pos.y = (_cmd & 0x0F);
				bullets[i].dir |= DIR_LEFT;
			}
			else if ((((_cmd & 0x30) >> 4) & DIR_LEFT) > 0)
			{
				bullets[i].pos.x = 0;
				bullets[i].pos.y = (_cmd & 0x0F);
				bullets[i].dir |= DIR_RIGHT;
			}

			break;
		}
	}
}
void CheckHitBullet()
{
	for (int i = BULLET_MAX_COUNT / 2; i < BULLET_MAX_COUNT; i++)
	{
		if ((bullets[i].dir & DIR_RUNNING) == 0)
			continue;

		if (abs(bullets[i].pos.x - player.pos.x) < 3
				&& abs(bullets[i].pos.y - player.pos.y) < 2)
		{
			bullets[i].dir = 0;

			g_flag = 0x80;
			g_tick = 0;

			// UART Transmit
			char buf[8];
			sprintf(buf, "%d\r\n", g_flag);
			HAL_UART_Transmit(&huart1, (uint8_t*) buf, strlen(buf), HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart2, (uint8_t*) buf, strlen(buf), HAL_MAX_DELAY);

			break;
		}
	}
}
void SummonWall()
{
	if (shooting_game_level != 3)
		return;

	if (wall_delay < g_tick)
	{
		wall_delay = rand() % 25 + 15;
		g_tick = 0;

		int8_t temp = rand() % 4;

		for (int i = 0; i < WALL_MAX_COUNT; i++)
		{
			if ((walls[i].dir & DIR_RUNNING) == 0)
			{
				walls[i].dir |= DIR_RUNNING;

				if (temp < 2)
				{
					walls[i].dir |= DIR_RIGHT;
					walls[i].pos.x = 0;
					if (temp == 0)
						walls[i].pos.y = 0;
					else
						walls[i].pos.y = 8;
				}
				else
				{
					walls[i].dir |= DIR_LEFT;
					walls[i].pos.x = LCD_MAX_X - 1;
					if (temp == 0)
						walls[i].pos.y = 0;
					else
						walls[i].pos.y = 8;
				}

				break;
			}
		}
	}
}
void CheckHitWall()
{
	for (int i = 0; i < WALL_MAX_COUNT; i++)
	{
		if ((walls[i].dir & DIR_RUNNING) == 0)
			continue;

		if (abs(walls[i].pos.x - player.pos.x) < 2
				&& player.pos.y - walls[i].pos.y >= -1
				&& player.pos.y - walls[i].pos.y < 9)
		{
			walls[i].dir = 0;

			g_flag = 0x80;
			g_tick = 0;

			// UART Transmit
			char buf[8];
			sprintf(buf, "%d\r\n", g_flag);
			HAL_UART_Transmit(&huart1, (uint8_t*) buf, strlen(buf), HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart2, (uint8_t*) buf, strlen(buf), HAL_MAX_DELAY);

			break;
		}
	}
}

void Init()
{
	srand(HAL_GetTick());

	lcd_clear();

	g_flag = 0;
	g_tick = 0;

	InitSceneClear();
	InitSceneShooting();
	InitSceneResult();
}
void InitSceneClear()
{
	head.pos.x = 0;
	head.pos.y = 0;
	head.dir = DIR_RIGHT;

	tail.pos.x = -20;
	tail.pos.y = 0;
	tail.dir = DIR_RIGHT;

	minX = 0;
	maxX = 80;
	minY = 0;
	maxY = 16;

	target_pos.x = 0;
	target_pos.y = 1;
}
void InitSceneMenu()
{
	g_flag = 0;
}
void InitSceneWaiting()
{
	g_tick = 0;
}
void InitSceneShooting()
{
	g_tick = 0;

	player.pos.x = 38;
	player.pos.y = 6;
	player.dir = DIR_LEFT;

	memset(map, 0, sizeof(map));

	shooting_game_level = 1;
	bullet_moving_distance = 2;
	bullet_size = 3;

	memset(bullets, 0, sizeof(bullets));
	memset(walls, 0, sizeof(walls));

	wall_delay = rand() % 25 + 15;
}
void InitSceneResult()
{
	g_tick = 0;
}

void Update()
{
	switch (g_cur_scene)
	{
		case SCENE_CLEAR:
		{
			UpdateSceneClear();
			break;
		}
		case SCENE_MENU:
		{
			UpdateSceneMenu();
			break;
		}
		case SCENE_WAITING:
		{
			UpdateSceneWaiting();
			break;
		}
		case SCENE_SHOOTING:
		{
			UpdateSceneShooting();
			break;
		}
		case SCENE_RESULT:
		{
			UpdateSceneResult();
			break;
		}
		default:
		{
			break;
		}
	}
}
void UpdateSceneClear()
{
	if ((head.dir & 0x03) == DIR_UP)
	{
		if (head.pos.y - object_moving_distance < minY)
		{
			head.dir = DIR_RIGHT;
		}
	}
	else if ((head.dir & 0x03) == DIR_RIGHT)
	{
		if (head.pos.x + object_moving_distance >= maxX)
		{
			head.dir = DIR_DOWN;
		}
	}
	else if ((head.dir & 0x03) == DIR_DOWN)
	{
		if (head.pos.y + object_moving_distance >= maxY)
		{
			head.dir = DIR_LEFT;
		}
	}
	else if ((head.dir & 0x03) == DIR_LEFT)
	{
		if (head.pos.x - object_moving_distance < minX)
		{
			head.dir = DIR_UP;
		}
	}

	if ((tail.dir & 0x03) == DIR_UP)
	{
		if (tail.pos.y - object_moving_distance < minY)
		{
			tail.dir = DIR_RIGHT;
			minX++;
		}
	}
	else if ((tail.dir & 0x03) == DIR_RIGHT)
	{
		if (tail.pos.x + object_moving_distance >= maxX)
		{
			tail.dir = DIR_DOWN;
			minY++;
		}
	}
	else if ((tail.dir & 0x03) == DIR_DOWN)
	{
		if (tail.pos.y + object_moving_distance >= maxY)
		{
			tail.dir = DIR_LEFT;
			maxX--;
		}
	}
	else if ((tail.dir & 0x03) == DIR_LEFT)
	{
		if (tail.pos.x - object_moving_distance < minX)
		{
			tail.dir = DIR_UP;
			maxY--;
		}
	}

	if ((head.pos.x != target_pos.x) || (head.pos.y != target_pos.y))
	{
		MoveByDir(&head);
	}
	MoveByDir(&tail);

	if ((head.pos.x != target_pos.x) || (head.pos.y != target_pos.y))
	{
		SetPixelByPos(head.pos);
	}
	ResetPixelByPos(tail.pos);

	if ((tail.pos.x == target_pos.x) && (tail.pos.y == target_pos.y))
	{
		lcd_clear();
		memset(map, 0, sizeof(map));

		InitSceneClear();
		g_nxt_scene = SCENE_MENU;
	}
}
void UpdateSceneMenu()
{
	if (g_input_sw > 0)
	{
		if (cur_menu_select == 0)
		{
			g_nxt_scene = SCENE_WAITING;
			return;
		}
	}

	if (g_input_y > 0)
	{
		if (cur_menu_select < MENU_COUNT - 1)
		{
			cur_menu_select++;
			if (menu_scroll_offset < cur_menu_select - 1)
			{
				menu_scroll_offset = cur_menu_select - 1;
				lcd_clear();
			}
		}
		else
		{
			cur_menu_select = 0;
			menu_scroll_offset = 0;
			lcd_clear();
		}
	}
	else if (g_input_y < 0)
	{
		if (cur_menu_select > 0)
		{
			cur_menu_select--;
			if (menu_scroll_offset > cur_menu_select)
			{
				menu_scroll_offset = cur_menu_select;
				lcd_clear();
			}
		}
		else
		{
			cur_menu_select = MENU_COUNT - 1;
			menu_scroll_offset = max(0, cur_menu_select - 1);
			lcd_clear();
		}
	}
}
void UpdateSceneWaiting()
{
	if (g_tick % 10 == 0)
	{
		// UART Transmit
		char buf[8];
		sprintf(buf, "%d\r\n", (0x40 | cur_menu_select));
		HAL_UART_Transmit(&huart1, (uint8_t*) buf, strlen(buf), HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2, (uint8_t*) buf, strlen(buf), HAL_MAX_DELAY);
	}

	if ((g_flag & 0x40) > 0)
	{
		if ((g_flag & 0x0F) == cur_menu_select)
		{
			g_nxt_scene = SCENE_SHOOTING;
			// UART Transmit
			char buf[8];
			sprintf(buf, "%d\r\n", (0x40 | cur_menu_select));
			HAL_UART_Transmit(&huart1, (uint8_t*) buf, strlen(buf), HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart2, (uint8_t*) buf, strlen(buf), HAL_MAX_DELAY);
		}
	}

	g_tick++;
}
void UpdateSceneShooting()
{
	g_tick++;

	if ((g_flag & 0x80) > 0)
	{
		if (g_tick > 60)
		{
			g_nxt_scene = SCENE_RESULT;
		}
		else if (g_tick % 6 == 0)
		{
			for (int i = 70; i < 90; i++)
			{
				map[i] = ~map[i];
			}
		}
	}
	else
	{
		if (shooting_game_level == 1 && g_tick > 100)
		{
			shooting_game_level = 2;
			bullet_moving_distance = 3;
			bullet_size = 5;

			g_tick = 0;
		}
		else if (shooting_game_level == 2 && g_tick > 100)
		{
			shooting_game_level = 3;
			bullet_moving_distance = 5;
			bullet_size = 7;

			g_tick = 0;
		}

		MovePlayer(g_input_x, g_input_y);

		if (g_input_sw > 0)
			FireBullet(&player);

		UpdateBullets();
		UpdateWalls();

		CheckHitBullet();
		CheckHitWall();
	}
}
void UpdateSceneResult()
{
	g_tick++;

	if (g_tick > 60)
	{
		g_nxt_scene = SCENE_MENU;
	}
}
void UpdateBullets()
{
	for (int i = 0; i < BULLET_MAX_COUNT; i++)
	{
		if ((bullets[i].dir & DIR_RUNNING) == 0)
			continue;

		if ((bullets[i].dir & DIR_RIGHT) > 0)
		{
			bullets[i].pos.x += bullet_moving_distance;

			if (i < BULLET_MAX_COUNT / 2 && LCD_MAX_X <= bullets[i].pos.x)
			{
				bullets[i].dir = 0;

				for (int j = 0; j <= bullet_size + bullet_moving_distance - 1; j++)
				{
					ResetPixelByXY(LCD_MAX_X - 1 - j, bullets[i].pos.y);
				}

				// UART Transmit
				uint8_t temp = 0;
				temp |= (DIR_RIGHT << 4);
				temp |= (bullets[i].pos.y & 0x0F);
				char buf[8];
				sprintf(buf, "%d\r\n", temp);
				HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
			}
			else if (i >= BULLET_MAX_COUNT / 2 && PLAYABLE_MAX_X + 10 < bullets[i].pos.x)
			{
				bullets[i].dir = 0;

				for (int j = 0; j <= bullet_size; j++)
				{
					ResetPixelByXY(bullets[i].pos.x - bullet_moving_distance - j, bullets[i].pos.y);
				}
			}
		}
		else if ((bullets[i].dir & DIR_LEFT) > 0)
		{
			bullets[i].pos.x -= bullet_moving_distance;

			if (i < BULLET_MAX_COUNT / 2 && bullets[i].pos.x < LCD_MIN_X)
			{
				bullets[i].dir = 0;

				for (int j = 0; j <= bullet_size + bullet_moving_distance - 1; j++)
				{
					ResetPixelByXY(LCD_MIN_X + j, bullets[i].pos.y);
				}

				// USART Transmit
				uint8_t temp = 0;
				temp |= (DIR_LEFT << 4);
				temp |= (bullets[i].pos.y & 0x0F);
				char buf[8];
				sprintf(buf, "%d\r\n", temp);
				HAL_UART_Transmit(&huart1, (uint8_t*) buf, strlen(buf), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, (uint8_t*) buf, strlen(buf), HAL_MAX_DELAY);
			}
			else if (i >= BULLET_MAX_COUNT / 2 && bullets[i].pos.x < PLAYABLE_MIN_X - 10)
			{
				bullets[i].dir = 0;

				for (int j = 0; j <= bullet_size; j++)
				{
					ResetPixelByXY(bullets[i].pos.x + bullet_moving_distance + j, bullets[i].pos.y);
				}
			}
		}
	}
}
void UpdateWalls()
{
	SummonWall();

	for (int i = 0; i < WALL_MAX_COUNT; i++)
	{
		if ((walls[i].dir & DIR_RUNNING) == 0)
			continue;

		if ((walls[i].dir & DIR_RIGHT) > 0)
		{
			walls[i].pos.x += 1;

			if (PLAYABLE_MAX_X + 10 <= walls[i].pos.x)
			{
				walls[i].dir = 0;

				for (int j = 0; j < 8; j++)
				{
					ResetPixelByXY(walls[i].pos.x - 1, walls[i].pos.y + j);
				}
			}
		}
		else if ((walls[i].dir & DIR_LEFT) > 0)
		{
			walls[i].pos.x -= 1;

			if (walls[i].pos.x < PLAYABLE_MIN_X - 10)
			{
				walls[i].dir = 0;

				for (int j = 0; j < 8; j++)
				{
					ResetPixelByXY(walls[i].pos.x + 1, walls[i].pos.y + j);
				}
			}
		}
	}
}

void RenderPixel()
{
	uint8_t display_count = 0;

	for (int i = 70; i >= 0; i -= 10)
	{
		uint8_t flag = 0;

		for (int j = 0; j < 5; j++)
		{
			if ((flag & 0x0F) == 0x0F)
			{
				break;
			}

			if (((flag & 0x01) == 0) && ((map[i + j] & 0xFF) > 0))
			{
				if (display_count < 8)
				{
					display_index[i / 5] = (display_count << 1);
					display_index[i / 5] |= 0x01;
					display_count++;
				}

				flag |= 0x01;
			}

			if (((flag & 0x02) == 0) && ((map[i + j + 5] & 0xFF) > 0))
			{
				if (display_count < 8)
				{
					display_index[i / 5 + 1] = (display_count << 1);
					display_index[i / 5 + 1] |= 0x01;
					display_count++;
				}

				flag |= 0x02;
			}

			if (((flag & 0x04) == 0) && ((map[MAP_SIZE - 10 - i + j] & 0xFF) > 0))
			{
				if (display_count < 8)
				{
					display_index[(MAP_SIZE - 10 - i) / 5] =
							(display_count << 1);
					display_index[(MAP_SIZE - 10 - i) / 5] |= 0x01;
					display_count++;
				}

				flag |= 0x04;
			}

			if (((flag & 0x08) == 0) && ((map[MAP_SIZE - 10 - i + j + 5] & 0xFF) > 0))
			{
				if (display_count < 8)
				{
					display_index[(MAP_SIZE - 10 - i) / 5 + 1] = (display_count
							<< 1);
					display_index[(MAP_SIZE - 10 - i) / 5 + 1] |= 0x01;
					display_count++;
				}

				flag |= 0x08;
			}
		}
	}

	for (int i = 0; i < 32; i++)
	{
		if (display_index[i] == 0x00)
		{
			if (((last_display_index >> i) & 0x00000001) > 0)
			{
				lcd_set_cursor(i % 2, i / 2);
				lcd_send_data(0x20);

				last_display_index &= ~(0x00000001 << i);
			}

			continue;
		}

		uint8_t temp[8] = { 0, };
		//temp[0] |= ( ((map[i * 5 + 0] >> 3) & 0x1F) );
		//temp[1] |= ( ((map[i * 5 + 0] << 2) & 0x1C) | ( (map[i * 5 + 1] >> 6) & 0x03) );
		//temp[2] |= ( ((map[i * 5 + 1] >> 1) & 0x1F) );
		//temp[3] |= ( ((map[i * 5 + 1] << 4) & 0x10) | ( (map[i * 5 + 2] >> 4) & 0x0F) );
		//temp[4] |= ( ((map[i * 5 + 2] << 1) & 0x1E) | ( (map[i * 5 + 3] >> 7) & 0x01) );
		//temp[5] |= ( ((map[i * 5 + 3] >> 2) & 0x1F) );
		//temp[6] |= ( ((map[i * 5 + 3] << 3) & 0x18) | ( (map[i * 5 + 4] >> 5) & 0x07) );
		//temp[7] |= ( ((map[i * 5 + 4] >> 0) & 0x1F) );
		temp[0] |= ((map[i * 5 + 0] >> 3) & 0x10) | ((map[i * 5 + 1] >> 4) & 0x08) | ((map[i * 5 + 2] >> 5) & 0x04) | ((map[i * 5 + 3] >> 6) & 0x02) | ((map[i * 5 + 4] >> 7) & 0x01);
		temp[1] |= ((map[i * 5 + 0] >> 2) & 0x10) | ((map[i * 5 + 1] >> 3) & 0x08) | ((map[i * 5 + 2] >> 4) & 0x04) | ((map[i * 5 + 3] >> 5) & 0x02) | ((map[i * 5 + 4] >> 6) & 0x01);
		temp[2] |= ((map[i * 5 + 0] >> 1) & 0x10) | ((map[i * 5 + 1] >> 2) & 0x08) | ((map[i * 5 + 2] >> 3) & 0x04) | ((map[i * 5 + 3] >> 4) & 0x02) | ((map[i * 5 + 4] >> 5) & 0x01);
		temp[3] |= ((map[i * 5 + 0] >> 0) & 0x10) | ((map[i * 5 + 1] >> 1) & 0x08) | ((map[i * 5 + 2] >> 2) & 0x04) | ((map[i * 5 + 3] >> 3) & 0x02) | ((map[i * 5 + 4] >> 4) & 0x01);
		temp[4] |= ((map[i * 5 + 0] << 1) & 0x10) | ((map[i * 5 + 1] >> 0) & 0x08) | ((map[i * 5 + 2] >> 1) & 0x04) | ((map[i * 5 + 3] >> 2) & 0x02) | ((map[i * 5 + 4] >> 3) & 0x01);
		temp[5] |= ((map[i * 5 + 0] << 2) & 0x10) | ((map[i * 5 + 1] << 1) & 0x08) | ((map[i * 5 + 2] >> 0) & 0x04) | ((map[i * 5 + 3] >> 1) & 0x02) | ((map[i * 5 + 4] >> 2) & 0x01);
		temp[6] |= ((map[i * 5 + 0] << 3) & 0x10) | ((map[i * 5 + 1] << 2) & 0x08) | ((map[i * 5 + 2] << 1) & 0x04) | ((map[i * 5 + 3] >> 0) & 0x02) | ((map[i * 5 + 4] >> 1) & 0x01);
		temp[7] |= ((map[i * 5 + 0] << 4) & 0x10) | ((map[i * 5 + 1] << 3) & 0x08) | ((map[i * 5 + 2] << 2) & 0x04) | ((map[i * 5 + 3] << 1) & 0x02) | ((map[i * 5 + 4] >> 0) & 0x01);

		lcd_create_char(display_index[i] >> 1, temp);
		lcd_set_cursor(i % 2, i / 2);
		lcd_send_data(display_index[i] >> 1);

		display_index[i] &= 0x00;
		last_display_index |= (0x00000001 << i);
	}
}
void Render()
{
	switch (g_cur_scene)
	{
		case SCENE_CLEAR:
		case SCENE_SHOOTING:
		{
			ResetPixel();
			SetPixel();

			RenderPixel();
			break;
		}
		case SCENE_WAITING:
		{
			RenderSceneWaiting();
			break;
		}
		case SCENE_MENU:
		{
			RenderSceneMenu();
			break;
		}
		case SCENE_RESULT:
		{
			RenderSceneResult();
			break;
		}
		default:
		{
			break;
		}
	}
}
void RenderSceneMenu()
{
	for (int i = 0; i < 2; i++)
	{
		uint8_t menu_index = menu_scroll_offset + i;
		lcd_set_cursor(i, 0);

		if (menu_index == cur_menu_select)
		{
			lcd_send_string(">");
		}
		else
		{
			lcd_send_string(" ");
		}

		lcd_send_string(menus[menu_index]);
	}
}
void RenderSceneWaiting()
{
	lcd_set_cursor(0, 4);
	lcd_send_string("WAITING");
	lcd_set_cursor(1, 2);
	lcd_send_string("FOR OPPONENT");
}
void RenderSceneResult()
{
	lcd_set_cursor(0, 4);
	lcd_send_string((g_flag & 0xC0) == 0xC0 ? "YOU WIN" : "YOU LOSE");
}

void PlayBGM()
{
	switch (g_cur_scene)
	{
		case SCENE_MENU:
		{
			playMainTheme(&myBuzzer, 20);
			break;
		}
		case SCENE_SHOOTING:
		{
			playTankBGM(&myBuzzer, 20);
			break;
		}
		default:
		{
			break;
		}
	}
}

void ChangeScene()
{
	if (g_cur_scene != g_nxt_scene)
	{
		g_prv_scene = g_cur_scene;
		g_cur_scene = g_nxt_scene;

		switch(g_cur_scene)
		{
			case SCENE_CLEAR:
			{
				InitSceneClear();
				break;
			}
			case SCENE_MENU:
			{
				InitSceneMenu();
				break;
			}
			case SCENE_WAITING:
			{
				InitSceneWaiting();
				break;
			}
			case SCENE_SHOOTING:
			{
				InitSceneShooting();
				break;
			}
			case SCENE_RESULT:
			{
				InitSceneResult();
				break;
			}
			default:
			{
				break;
			}
		}

		lcd_clear();
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	Init();
	/* Infinite loop */
	for (;;)
	{
		Update();

		Render();

		ChangeScene();

		g_input_x = 0;
		g_input_y = 0;
		g_input_sw = 0;

		osDelay(g_cur_scene == SCENE_CLEAR ? 1 : 100);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the JoyStickTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
	/* Infinite loop */
	for (;;)
	{
		Joystick_Data_t data = Joystick_Read(&hjs, 10);
		Joystick_Track(&tracker, &data, 3000, 1000);

		if (g_cur_scene == SCENE_SHOOTING)
		{
			if (g_input_x == 0)
			{
				if ((tracker.x_plus_history & 0x01) == 0x01)
					g_input_x = 1;
				else if ((tracker.x_minus_history & 0x01) == 0x01)
					g_input_x = -1;

				//g_input_x = tracker.x_pos;
			}
			if (g_input_y == 0)
			{
				if ((tracker.y_plus_history & 0x01) == 0x01)
					g_input_y = 1;
				else if ((tracker.y_minus_history & 0x01) == 0x01)
					g_input_y = -1;

				//g_input_y = tracker.y_pos;
			}
		}
		else
		{
			if (g_input_x == 0)
			{
				if ((tracker.x_plus_history & 0x03) == 0x01)
					g_input_x = 1;
				else if ((tracker.x_minus_history & 0x03) == 0x01)
					g_input_x = -1;

				//g_input_x = tracker.x_pos;
			}
			if (g_input_y == 0)
			{
				if ((tracker.y_plus_history & 0x03) == 0x01)
					g_input_y = 1;
				else if ((tracker.y_minus_history & 0x03) == 0x01)
					g_input_y = -1;

				//g_input_y = tracker.y_pos;
			}
		}
		if (g_input_sw == 0) g_input_sw = (data.button == GPIO_PIN_SET);

#if SERIAL_DEBUG_MODE
		char buf[8];
		sprintf(buf, "%d\r\n", data.button);
		if (data.button == GPIO_PIN_SET)
			HAL_UART_Transmit(&huart2, (uint8_t*) buf, strlen(buf), HAL_MAX_DELAY);
#endif

		osDelay(40);
	}
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the BGMTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
	  PlayBGM();

	  osDelay(20);
  }
  /* USER CODE END StartTask03 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
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
#ifdef USE_FULL_ASSERT
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
