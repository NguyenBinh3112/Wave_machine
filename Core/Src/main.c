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
#include "LyquidCrystal_I2C.h"
#include "Button.h"
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
Button_handle button1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
LiquidCrystal_I2C hlcd;

uint8_t cnt =0 ;
uint8_t cusor = 0;
int8_t cw =0;
uint8_t place =0;
int8_t number[8] ={0,0,0,0,0,0,0,0};

int8_t so = 0;
uint8_t count =0;

uint8_t a;
uint8_t in =0;

uint8_t a_state;
uint8_t a_state_bandau;

uint32_t freq = 16000000;
uint8_t duty = 0;

int32_t frequency;
int8_t duty_cycle;

uint8_t F[8] = {0,0,0,0,0,0,0,0};
uint8_t D[2] = {0,0};

uint32_t time;

uint8_t num_xoay()
{
		a_state =  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
		if(a_state != a_state_bandau)
		{
			if(a_state != HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4))
			{
				cw++;
				if(cw >=1 ) cw = 1;
				if(a == 0)
				{
					if(in == 0)
					{
					place ++;
					}
					else
					{
						switch(place)
						{
							case 0:
							number[0] ++;
							if(number[0] > 1)
							{
								number[0] = 0;
							}
							break;
							case 1:
							number[1] ++;
							if(number[1] > 6)
							{
								number[1] = 0;
							}					
							break;	
							case 2:
							number[2] ++;
							if(number[2] > 9)
							{
								number[2] = 0;
							}
							break;
							case 3:
							number[3] ++;
							if(number[3] > 9)
							{
								number[3] = 0;
							}
							break;
							case 4:
							number[4] ++;
							if(number[4] > 9)
							{
								number[4] = 0;
							}
							break;
							case 5:
							number[5] ++;
							if(number[5] > 9)
							{
								number[5] = 0;
							}
							break;
							case 6:
							number[6] ++;
							if(number[6] > 9)
							{
								number[6] = 0;
							}
							break;
							case 7:
							number[7] ++;
							if(number[7] > 9)
							{
								number[7] = 0;
							}
							break;						
						}
					}
				}
				else
				{
					so++;
					if(so > 100)
					{
						so = 0;
					}
				}

			}
			else
			{
				cw --;
				if(cw < 1) cw = 0;
				if(a == 0)
				{
					if(in == 0)
					{
					place --;
					}
					else
					{
						switch(place)
						{
							case 0:
								number[0] --;
								if(number[0] < 0)
								{
									number[0] = 1;
								}
								break;
							case 1:
								number[1] --;
								if(number[1] < 0)
								{
									number[1] = 6;
								}							
								break;
							case 2:
								number[2] --;
								if(number[2] < 0)
								{
									number[2] = 9;
								}
								break;		
							case 3:
								number[3] --;
								if(number[3] < 0)
								{
									number[3] = 9;
								}
								break;
							case 4:
								number[4] --;
								if(number[4] < 0)
								{
									number[4] = 9;
								}
								break;
							case 5:
								number[5] --;
								if(number[5] < 0)
								{
									number[5] = 9;
								}
								break;
							case 6:
								number[6] --;
								if(number[6] < 0)
								{
									number[6] = 9;
								}
								break;
							case 7:
								number[7] --;
								if(number[7] < 0)
								{
									number[7] = 9;
								}
								break;							
						}
					}
				}
				else
				{
					so --;
					if(so < 0)
					{
						so = 100;
					}
				}

			}
		}
		a_state_bandau = a_state;
		return cw;
}


void button_pressing_callback()
{
	switch(cnt)
	{
		case 0:
			cnt ++;
			count ++;
			break;
		case 1:
			cnt ++;
			lcd_clear_display(&hlcd);
			break;
		case 2:
			cnt ++;
			lcd_clear_display(&hlcd);
			break;
		case 3:
			if(in == 0)
			{
				in ++;
			}
			else if(in ==1)
			{
				in --;
			}
			break;
	}
}

void button_press_long_callback()
{
	lcd_clear_display(&hlcd);
	lcd_set_cursor(&hlcd, 0, 0);
	lcd_printf(&hlcd, "complete");
	lcd_set_cursor_off(&hlcd);
	if(cnt !=0)
	{
		cnt = 0;
	}
	frequency = F[7]+10*F[6]+100*F[5]+1000*F[4]+10000*F[3]+100000*F[2]+1000000*F[1]+10000000*F[0];
	TIM1->ARR = 64000000/frequency;
}

void lcd_screen(uint8_t cnt)
{

	switch(cnt)
	{
		case 1:
			if(count == 1)
			{
			lcd_clear_display(&hlcd);		
			count ++;
			}				
			lcd_set_cursor(&hlcd, 0, 0);
			lcd_printf(&hlcd, "F: %d%d%d%d%d%d%d%d", F[0],F[1],F[2],F[3],F[4],F[5],F[6],F[7]);
			lcd_set_cursor(&hlcd, 1, 0);
			lcd_printf(&hlcd, "Duty: %02d %c", duty,'%');
		break;
		case 2:
			num_xoay();
			if(cw == 0)
			{
				lcd_set_cursor(&hlcd, 0, 0);
				lcd_printf(&hlcd, "F:");
				lcd_set_cursor(&hlcd, 1, 0);
				lcd_printf(&hlcd, "Duty:");
				lcd_set_cursor(&hlcd, 0, 9);
				lcd_printf(&hlcd, "*");
				lcd_set_cursor(&hlcd, 1, 9);
				lcd_printf(&hlcd, " ");
				time = HAL_GetTick();
				while((HAL_GetTick() - time) < 5);	
			}
			else if(cw == 1)
			{
				lcd_set_cursor(&hlcd, 0, 0);
				lcd_printf(&hlcd, "F:");
				lcd_set_cursor(&hlcd, 1, 0);
				lcd_printf(&hlcd, "Duty:");
				lcd_set_cursor(&hlcd, 0, 9);
				lcd_printf(&hlcd, " ");
				lcd_set_cursor(&hlcd, 1, 9);
				lcd_printf(&hlcd, "*");
				time = HAL_GetTick();
				while((HAL_GetTick() - time) < 5);	

			}
			a = cw;
		break;
		case 3:
				if(a == 0)
				{
						lcd_clear_display(&hlcd);
						lcd_set_cursor(&hlcd, 0, 0);
						lcd_printf(&hlcd, "Freq: %d%d%d%d%d%d%d%d", F[0], F[1], F[2], F[3], F[4], F[5], F[6],F[7]);
						num_xoay();
						lcd_set_cursor(&hlcd, 1, place + 6);
						lcd_set_cursor_on(&hlcd);
						if(place >7)
						{
							place = 0;
						}
							switch(place)
							{
								case 0:
									F[0] = number[0];
									break;
								case 1:
									F[1] = number[1];
									break;
								case 2:
									F[2] = number[2];
									break;
								case 3:
									F[3] = number[3];
									break;
								case 4:
									F[4] = number[4];
									break;
								case 5:
									F[5] = number[5];
									break;
								case 6:
									F[6] = number[6];
									break;
								case 7:
									F[7] = number[7];
									break;			
						}		
						time = HAL_GetTick();
						while((HAL_GetTick() - time) < 15);
				}
				else if(a == 1)
				{
						lcd_set_cursor(&hlcd, 0, 0);
						lcd_printf(&hlcd, "Duty : %03d %c", duty, '%');
						num_xoay();
						duty = so;
						TIM1->CCR1 = (TIM1->ARR * duty)/100;					
						time = HAL_GetTick();
						while((HAL_GetTick() - time) < 5);					
				}
			break;
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	lcd_init(&hlcd, &hi2c1, LCD_ADDR_DEFAULT);
	Button_Init(&button1, GPIOA, GPIO_PIN_0);
	
	lcd_set_cursor(&hlcd, 0,0);
	lcd_printf(&hlcd, "Hello");
	lcd_set_cursor(&hlcd, 1,0);
	lcd_printf(&hlcd, "Wave_machine");		
	HAL_Delay(1500);
	lcd_clear_display(&hlcd);	
	a_state_bandau = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
	a = 0;
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		button_handle(&button1);
		lcd_screen(cnt);
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
