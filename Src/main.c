/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void readChannel(uint8_t chWhat);
void trigger(void);
void buzz(uint32_t rounds);
void delayUS(uint32_t us);
/* Variables Start */
uint32_t dipSW 		= 0xFFFFFFFF;
uint32_t sensors 	=	0xFFFFFFFF;
_Bool inverted;
_Bool test;
_Bool CH1Rly, CH2Rly, CH3Rly, CH4Rly, CH5Rly, CH6Rly = 0x0;
/* Variables Ends*/
 void dipSWR(void);
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
	buzz(5000000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		CH1Rly = 0x0; CH2Rly = 0x0; CH3Rly = 0x0; CH4Rly = 0x0; CH5Rly = 0x0;  CH6Rly = 0x0;
		sensors 	=	0xFFFFFFFF;
		dipSWR();
		if (((dipSW >> 0) & 1))
		{
			inverted = 1;
		}
		else
		{
			inverted = 0;
		}
		
		if (((dipSW >> 1) & 1)) //CH6 BYP switch check to see to read the value or no
		{
			readChannel(6U);
		}
		else 
		{
		if (inverted)
		{
		sensors &= ~((0) << 3);
		sensors &= ~((0) << 2);
		sensors &= ~((0) << 1);
		sensors &= ~((0) << 0);		
		}
		else		
		{
		sensors &= ~((1) << 3);
		sensors &= ~((1) << 2);
		sensors &= ~((1) << 1);
		sensors &= ~((1) << 0);	
		}
		
		}
		
		if (((dipSW >> 2) & 1)) //CH5 BYP switch check to see to read the value or no
		{
			readChannel(5U);
		}
		else 
		{
		if (inverted)
		{
		sensors &= ~((0) << 7);
		sensors &= ~((0) << 6);
		sensors &= ~((0) << 5);
		sensors &= ~((0) << 4);	
		}
		else
		{
		sensors &= ~((1) << 7);
		sensors &= ~((1) << 6);
		sensors &= ~((1) << 5);
		sensors &= ~((1) << 4);	
		}
		}
		
		
		if (((dipSW >> 3) & 1)) //CH4 BYP switch check to see to read the value or no
		{
			readChannel(4U);
		}
		else 
		{
			if (inverted)
			{
		sensors &= ~((0) << 11);
		sensors &= ~((0) << 10);
		sensors &= ~((0) << 9);
		sensors &= ~((0) << 8);
			}
			else
			{
		sensors &= ~((1) << 11);
		sensors &= ~((1) << 10);
		sensors &= ~((1) << 9);
		sensors &= ~((1) << 8);
			}
				
		
		}
		
		if (((dipSW >> 4) & 1)) //CH3 BYP switch check to see to read the value or no
		{
			readChannel(3U);
		}
		else 
		{
				if (inverted)
				{
		sensors &= ~((0) << 15);
		sensors &= ~((0) << 14);
		sensors &= ~((0) << 13);
		sensors &= ~((0) << 12);
				}
				else 
				{
		sensors &= ~((1) << 15);
		sensors &= ~((1) << 14);
		sensors &= ~((1) << 13);
		sensors &= ~((1) << 12);
				}
		}
		
		if (((dipSW >> 5) & 1)) //CH2 BYP switch check to see to read the value or no
		{
			readChannel(2U);
		}
		else 
		{
		if (inverted)
		{			
		sensors &= ~((0) << 19);
		sensors &= ~((0) << 18);
		sensors &= ~((0) << 17);
		sensors &= ~((0) << 16);
		}
		else
		{
		sensors &= ~((1) << 19);
		sensors &= ~((1) << 18);
		sensors &= ~((1) << 17);
		sensors &= ~((1) << 16);
		}
		}
		
		if (((dipSW >> 6) & 1)) //CH1 BYP switch check to see to read the value or no
		{
			readChannel(1U);
		}
		else 
		{
		if (inverted)
		{
		sensors &= ~((0) << 23);
		sensors &= ~((0) << 22);
		sensors &= ~((0) << 21);
		sensors &= ~((0) << 20);
		}
		else
		{
		sensors &= ~((1) << 23);
		sensors &= ~((1) << 22);
		sensors &= ~((1) << 21);
		sensors &= ~((1) << 20);
		}
	}
		
		if (inverted)
		{
			sensors = (~sensors);
		}
		
		
		trigger();
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CH6R_2_Pin|CH6R_3_Pin|CH6R_4_Pin|CH1R_2_Pin 
                          |CH1R_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RTS_Pin|CH1G_1_Pin|CONN_FAIL_Pin|CH6R_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, PWR_SYSF_Pin|ASYS_F_Pin|PWR_LED_Pin|RLY1_Pin 
                          |Beep_Pin|TESTpb_Pin|MUTEpb_Pin|CH6G_6_Pin 
                          |CH5R_1_Pin|CH4R_4_Pin|CH4R_3_Pin|CH4R_2_Pin 
                          |CH4R_1_Pin|CH3R_4_Pin|CH3R_3_Pin|CH3R_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CH5G_5_Pin|CH4G_4_Pin|CH3G_3_Pin|CH2G_2_Pin 
                          |CH5R_4_Pin|CH5R_3_Pin|CH5R_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CH3R_1_Pin|CH2R_4_Pin|CH2R_3_Pin|CH2R_2_Pin 
                          |CH2R_1_Pin|CH1R_4_Pin|CH1R_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CH6R_2_Pin CH6R_3_Pin CH6R_4_Pin CH1R_2_Pin 
                           CH1R_1_Pin */
  GPIO_InitStruct.Pin = CH6R_2_Pin|CH6R_3_Pin|CH6R_4_Pin|CH1R_2_Pin 
                          |CH1R_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : ALLGBYP_7_Pin CH6GBYP_6_Pin CH3_2_Pin CH3_3_Pin 
                           CH3_4_Pin CH4_1_Pin CH4_2_Pin CH4_3_Pin 
                           CH4_4_Pin CH5_1_Pin CH5_2_Pin */
  GPIO_InitStruct.Pin = ALLGBYP_7_Pin|CH6GBYP_6_Pin|CH3_2_Pin|CH3_3_Pin 
                          |CH3_4_Pin|CH4_1_Pin|CH4_2_Pin|CH4_3_Pin 
                          |CH4_4_Pin|CH5_1_Pin|CH5_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CH5GBYP_5_Pin CH4GBYP_4_Pin CH3GBYP_3_Pin CH2GBYP_2_Pin 
                           CH1GBYP_1_Pin CH2_1_Pin CH2_2_Pin */
  GPIO_InitStruct.Pin = CH5GBYP_5_Pin|CH4GBYP_4_Pin|CH3GBYP_3_Pin|CH2GBYP_2_Pin 
                          |CH1GBYP_1_Pin|CH2_1_Pin|CH2_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RSV_Pin CH1_1_Pin CH1_2_Pin CH1_3_Pin 
                           CH1_4_Pin PMUTEPB_Pin BATT_FAIL_Pin AC_FAIL_Pin */
  GPIO_InitStruct.Pin = RSV_Pin|CH1_1_Pin|CH1_2_Pin|CH1_3_Pin 
                          |CH1_4_Pin|PMUTEPB_Pin|BATT_FAIL_Pin|AC_FAIL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RTS_Pin CH1G_1_Pin CONN_FAIL_Pin CH6R_1_Pin */
  GPIO_InitStruct.Pin = RTS_Pin|CH1G_1_Pin|CONN_FAIL_Pin|CH6R_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CH2_3_Pin CH2_4_Pin CH3_1_Pin CH5_3_Pin 
                           CH5_4_Pin CH6_1_Pin CH6_2_Pin CH6_3_Pin 
                           CH6_4_Pin */
  GPIO_InitStruct.Pin = CH2_3_Pin|CH2_4_Pin|CH3_1_Pin|CH5_3_Pin 
                          |CH5_4_Pin|CH6_1_Pin|CH6_2_Pin|CH6_3_Pin 
                          |CH6_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PWR_SYSF_Pin ASYS_F_Pin PWR_LED_Pin RLY1_Pin 
                           Beep_Pin TESTpb_Pin MUTEpb_Pin CH6G_6_Pin 
                           CH5R_1_Pin CH4R_4_Pin CH4R_3_Pin CH4R_2_Pin 
                           CH4R_1_Pin CH3R_4_Pin CH3R_3_Pin CH3R_2_Pin */
  GPIO_InitStruct.Pin = PWR_SYSF_Pin|ASYS_F_Pin|PWR_LED_Pin|RLY1_Pin 
                          |Beep_Pin|TESTpb_Pin|MUTEpb_Pin|CH6G_6_Pin 
                          |CH5R_1_Pin|CH4R_4_Pin|CH4R_3_Pin|CH4R_2_Pin 
                          |CH4R_1_Pin|CH3R_4_Pin|CH3R_3_Pin|CH3R_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : CH5G_5_Pin CH4G_4_Pin CH3G_3_Pin CH2G_2_Pin 
                           CH5R_4_Pin CH5R_3_Pin CH5R_2_Pin */
  GPIO_InitStruct.Pin = CH5G_5_Pin|CH4G_4_Pin|CH3G_3_Pin|CH2G_2_Pin 
                          |CH5R_4_Pin|CH5R_3_Pin|CH5R_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CH3R_1_Pin CH2R_4_Pin CH2R_3_Pin CH2R_2_Pin 
                           CH2R_1_Pin CH1R_4_Pin CH1R_3_Pin */
  GPIO_InitStruct.Pin = CH3R_1_Pin|CH2R_4_Pin|CH2R_3_Pin|CH2R_2_Pin 
                          |CH2R_1_Pin|CH1R_4_Pin|CH1R_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void dipSWR(void)
{
	dipSW = 0xFFFF;
//	dipSW &= ~~(1 << 6);
//	dipSW &= ~~(1 << 5);
//	dipSW &= ~(1 << 4);
//	dipSW &= ~(1 << 3);
//	dipSW &= ~(1 << 2);
//	dipSW &= ~~(1 << 1);
//	dipSW &= ~(1 << 0);
		//When SW on, Signal HIGH
		dipSW  &= ~(HAL_GPIO_ReadPin(CH1GBYP_1_GPIO_Port, CH1GBYP_1_Pin) << 6);// 7th bith 
		dipSW  &= ~(HAL_GPIO_ReadPin(CH2GBYP_2_GPIO_Port, CH2GBYP_2_Pin) << 5);
		dipSW  &= ~(HAL_GPIO_ReadPin(CH3GBYP_3_GPIO_Port, CH3GBYP_3_Pin) << 4);
		dipSW  &= ~(HAL_GPIO_ReadPin(CH4GBYP_4_GPIO_Port, CH4GBYP_4_Pin) << 3);
		dipSW  &= ~(HAL_GPIO_ReadPin(CH5GBYP_5_GPIO_Port, CH5GBYP_5_Pin) << 2);
		dipSW  &= ~(HAL_GPIO_ReadPin(CH6GBYP_6_GPIO_Port, CH6GBYP_6_Pin) << 1);
		dipSW  &= ~(HAL_GPIO_ReadPin(ALLGBYP_7_GPIO_Port, ALLGBYP_7_Pin) << 0);
}

void readChannel(uint8_t chWhat)
{
	switch (chWhat)
  {
  	case 1U: 
		sensors &= ~(	HAL_GPIO_ReadPin(CH1_1_GPIO_Port, CH1_1_Pin) << 23);
		sensors &= ~(	HAL_GPIO_ReadPin(CH1_2_GPIO_Port, CH1_2_Pin) << 22);
		sensors &= ~(	HAL_GPIO_ReadPin(CH1_3_GPIO_Port, CH1_3_Pin) << 21);
		sensors &= ~(	HAL_GPIO_ReadPin(CH1_4_GPIO_Port, CH1_4_Pin) << 20);
  			break;
  	case 2U:
		sensors &= ~(	HAL_GPIO_ReadPin(CH2_1_GPIO_Port, CH2_1_Pin) << 19);
		sensors &= ~(	HAL_GPIO_ReadPin(CH2_2_GPIO_Port, CH2_2_Pin) << 18);
		sensors &= ~(	HAL_GPIO_ReadPin(CH2_3_GPIO_Port, CH2_3_Pin) << 17);
		sensors &= ~(	HAL_GPIO_ReadPin(CH2_4_GPIO_Port, CH2_4_Pin) << 16);
			
  			break;
	case 3U:
		sensors &= ~(	HAL_GPIO_ReadPin(CH3_1_GPIO_Port, CH3_1_Pin) << 15);
		sensors &= ~(	HAL_GPIO_ReadPin(CH3_2_GPIO_Port, CH3_2_Pin) << 14);
		sensors &= ~(	HAL_GPIO_ReadPin(CH3_3_GPIO_Port, CH3_3_Pin) << 13);
		sensors &= ~(	HAL_GPIO_ReadPin(CH3_4_GPIO_Port, CH3_4_Pin) << 12);
			
			break;
	case 4U:
		sensors &= ~(	HAL_GPIO_ReadPin(CH4_1_GPIO_Port, CH4_1_Pin) << 11);
		sensors &= ~(	HAL_GPIO_ReadPin(CH4_2_GPIO_Port, CH4_2_Pin) << 10);
		sensors &= ~(	HAL_GPIO_ReadPin(CH4_3_GPIO_Port, CH4_3_Pin) << 9);
		sensors &= ~(	HAL_GPIO_ReadPin(CH4_4_GPIO_Port, CH4_4_Pin) << 8);
		
			break;
	case 5U:
		sensors &= ~(	HAL_GPIO_ReadPin(CH5_1_GPIO_Port, CH5_1_Pin) << 7);
		sensors &= ~(	HAL_GPIO_ReadPin(CH5_2_GPIO_Port, CH5_2_Pin) << 6);
		sensors &= ~(	HAL_GPIO_ReadPin(CH5_3_GPIO_Port, CH5_3_Pin) << 5);
		sensors &= ~(	HAL_GPIO_ReadPin(CH5_4_GPIO_Port, CH5_4_Pin) << 4);
		
			break;
		
	case 6U:
		sensors &= ~(	HAL_GPIO_ReadPin(CH6_1_GPIO_Port, CH6_1_Pin) << 3);
		sensors &= ~(	HAL_GPIO_ReadPin(CH6_2_GPIO_Port, CH6_2_Pin) << 2);
		sensors &= ~(	HAL_GPIO_ReadPin(CH6_3_GPIO_Port, CH6_3_Pin) << 1);
		sensors &= ~(	HAL_GPIO_ReadPin(CH6_4_GPIO_Port, CH6_4_Pin) << 0);
			
			break;

  	default:
  			break;
  }
}

void trigger(void)
{
	//HAL_GPIO_WritePin(RLY1_GPIO_Port, RLY1_Pin, GPIO_PIN_RESET);
	test = ((sensors) ^ (1)) & 1; 
	//test = ((sensors >> 0 ^ 1) && (sensors >> 1 ^ 1 ) && ((sensors >> 2) ^ 1) && (sensors >> 3 ^ 1)) ; 
	//CH6, Green ligth // All Trigger 0,  ()
	if ((((sensors >> 0) ^ 1 )&1 ) & (((sensors >> 1) ^ 1 )&1) & (((sensors >> 2) ^ 1) &1) & (((sensors >> 3) ^ 1)&1))
	{
		//Turn ON Green LED for CH6
		HAL_GPIO_WritePin(CH6G_6_GPIO_Port, CH6G_6_Pin, GPIO_PIN_SET);
		//Turn Of Any Red LEDS, System is good.
		HAL_GPIO_WritePin(CH6R_1_GPIO_Port, CH6R_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CH6R_2_GPIO_Port, CH6R_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CH6R_3_GPIO_Port, CH6R_3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CH6R_4_GPIO_Port, CH6R_4_Pin, GPIO_PIN_RESET);
		CH6Rly = 0x00;
	}
	else
	{
		CH6Rly = 0x01;
		HAL_GPIO_WritePin(CH6G_6_GPIO_Port, CH6G_6_Pin, GPIO_PIN_RESET);
		//Each individual red led trigger CH6-1 -> CH1-1
		if((sensors >> 0) & 1)
		{
			HAL_GPIO_WritePin(CH6R_4_GPIO_Port, CH6R_4_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(CH6R_4_GPIO_Port, CH6R_4_Pin, GPIO_PIN_RESET);
		}
		if((sensors >> 1) & 1)
		{
			HAL_GPIO_WritePin(CH6R_3_GPIO_Port, CH6R_3_Pin, GPIO_PIN_SET);
		}
		else 
		{
			HAL_GPIO_WritePin(CH6R_3_GPIO_Port, CH6R_3_Pin, GPIO_PIN_RESET);
		}
		if((sensors >> 2) & 1)
		{
			HAL_GPIO_WritePin(CH6R_2_GPIO_Port, CH6R_2_Pin, GPIO_PIN_SET);
		}
		else 
		{
			HAL_GPIO_WritePin(CH6R_2_GPIO_Port, CH6R_2_Pin, GPIO_PIN_RESET);
		}
		if((sensors >> 3) & 1)
		{
			HAL_GPIO_WritePin(CH6R_1_GPIO_Port, CH6R_1_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(CH6R_1_GPIO_Port, CH6R_1_Pin, GPIO_PIN_RESET);
		}
			
		//HAL_GPIO_WritePin(RLY1_GPIO_Port, RLY1_Pin, GPIO_PIN_SET);
	}
		if ((((sensors >> 4) ^ 1) &1) && (((sensors >> 5) ^ 1 )&1) && (((sensors >> 6) ^ 1)&1) && (((sensors >> 7) ^ 1)&1))
	{
		//Turn ON Green LED for CH5
		HAL_GPIO_WritePin(CH5G_5_GPIO_Port, CH5G_5_Pin, GPIO_PIN_SET);
		//Turn Of Any Red LEDS, System is good.
		HAL_GPIO_WritePin(CH5R_1_GPIO_Port, CH5R_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CH5R_2_GPIO_Port, CH5R_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CH5R_3_GPIO_Port, CH5R_3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CH5R_4_GPIO_Port, CH5R_4_Pin, GPIO_PIN_RESET);
		CH5Rly = 0x00;
	}
	else
	{
		CH5Rly = 0x01;
		HAL_GPIO_WritePin(CH5G_5_GPIO_Port, CH5G_5_Pin, GPIO_PIN_RESET);
		//Each individual red led trigger CH5-1 -> CH1-1
		if((sensors >> 4) & 1)
		{
			HAL_GPIO_WritePin(CH5R_4_GPIO_Port, CH5R_4_Pin, GPIO_PIN_SET);
		}
		else 
		{
			HAL_GPIO_WritePin(CH5R_4_GPIO_Port, CH5R_4_Pin, GPIO_PIN_RESET);
		}
			
		if((sensors >> 5) & 1)
		{
			HAL_GPIO_WritePin(CH5R_3_GPIO_Port, CH5R_3_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(CH5R_3_GPIO_Port, CH5R_3_Pin, GPIO_PIN_RESET);
		}
		if((sensors >> 6) & 1)
		{
			HAL_GPIO_WritePin(CH5R_2_GPIO_Port, CH5R_2_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(CH5R_2_GPIO_Port, CH5R_2_Pin, GPIO_PIN_RESET);
		}
			
		if((sensors >> 7) & 1)
		{
			HAL_GPIO_WritePin(CH5R_1_GPIO_Port, CH5R_1_Pin, GPIO_PIN_SET);
		}
		else 
		{
			HAL_GPIO_WritePin(CH5R_1_GPIO_Port, CH5R_1_Pin, GPIO_PIN_RESET);
		}
		//HAL_GPIO_WritePin(RLY1_GPIO_Port, RLY1_Pin, GPIO_PIN_SET);
	}
			if ((((sensors >> 8) ^ 1) &1) && (((sensors >> 9) ^ 1 )&1) && (((sensors >> 10) ^ 1)&1) && (((sensors >> 11) ^ 1)&1))
	{
		//Turn ON Green LED for CH4
		HAL_GPIO_WritePin(CH4G_4_GPIO_Port, CH4G_4_Pin, GPIO_PIN_SET);
		//Turn Of Any Red LEDS, System is good.
		HAL_GPIO_WritePin(CH4R_1_GPIO_Port, CH4R_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CH4R_2_GPIO_Port, CH4R_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CH4R_3_GPIO_Port, CH4R_3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CH4R_4_GPIO_Port, CH4R_4_Pin, GPIO_PIN_RESET);
		CH4Rly = 0x00;
	}
	else
	{
		CH4Rly = 0x01;
		HAL_GPIO_WritePin(CH4G_4_GPIO_Port, CH4G_4_Pin, GPIO_PIN_RESET);
		//Each individual red led trigger CH4-1 -> CH1-1
		if((sensors >> 8) & 1)
		{
			HAL_GPIO_WritePin(CH4R_4_GPIO_Port, CH4R_4_Pin, GPIO_PIN_SET);
		}
		else 
		{
			HAL_GPIO_WritePin(CH4R_4_GPIO_Port, CH4R_4_Pin, GPIO_PIN_RESET);
		}
			
		if((sensors >> 9) & 1)
		{
			HAL_GPIO_WritePin(CH4R_3_GPIO_Port, CH4R_3_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(CH4R_3_GPIO_Port, CH4R_3_Pin, GPIO_PIN_RESET);
		}
		if((sensors >> 10) & 1)
		{
			HAL_GPIO_WritePin(CH4R_2_GPIO_Port, CH4R_2_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(CH4R_2_GPIO_Port, CH4R_2_Pin, GPIO_PIN_RESET);
		}
			
		if((sensors >> 11) & 1)
		{
			HAL_GPIO_WritePin(CH4R_1_GPIO_Port, CH4R_1_Pin, GPIO_PIN_SET);
		}
		else 
		{
			HAL_GPIO_WritePin(CH4R_1_GPIO_Port, CH4R_1_Pin, GPIO_PIN_RESET);
		}
		//HAL_GPIO_WritePin(RLY1_GPIO_Port, RLY1_Pin, GPIO_PIN_SET);
	}
	
	if ((((sensors >> 12) ^ 1)&1) && (((sensors >> 13) ^ 1 )&1) && (((sensors >> 14) ^ 1)&1) && (((sensors >> 15) ^ 1)&1))
	{
		//Turn ON Green LED for CH3
		HAL_GPIO_WritePin(CH3G_3_GPIO_Port, CH3G_3_Pin, GPIO_PIN_SET);
		//Turn Of Any Red LEDS, System is good.
		HAL_GPIO_WritePin(CH3R_1_GPIO_Port, CH3R_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CH3R_2_GPIO_Port, CH3R_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CH3R_3_GPIO_Port, CH3R_3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CH3R_4_GPIO_Port, CH3R_4_Pin, GPIO_PIN_RESET);
		CH3Rly = 0x00;
	}
	else
	{
		CH3Rly = 0x01;
		HAL_GPIO_WritePin(CH3G_3_GPIO_Port, CH3G_3_Pin, GPIO_PIN_RESET);
		//Each individual red led trigger CH3-1 -> CH1-1
		if((sensors >> 12) & 1)
		{
			HAL_GPIO_WritePin(CH3R_4_GPIO_Port, CH3R_4_Pin, GPIO_PIN_SET);
		}
		else 
		{
			HAL_GPIO_WritePin(CH3R_4_GPIO_Port, CH3R_4_Pin, GPIO_PIN_RESET);
		}
			
		if((sensors >> 13) & 1)
		{
			HAL_GPIO_WritePin(CH3R_3_GPIO_Port, CH3R_3_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(CH3R_3_GPIO_Port, CH3R_3_Pin, GPIO_PIN_RESET);
		}
		if((sensors >> 14) & 1)
		{
			HAL_GPIO_WritePin(CH3R_2_GPIO_Port, CH3R_2_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(CH3R_2_GPIO_Port, CH3R_2_Pin, GPIO_PIN_RESET);
		}
			
		if((sensors >> 15) & 1)
		{
			HAL_GPIO_WritePin(CH3R_1_GPIO_Port, CH3R_1_Pin, GPIO_PIN_SET);
		}
		else 
		{
			HAL_GPIO_WritePin(CH3R_1_GPIO_Port, CH3R_1_Pin, GPIO_PIN_RESET);
		}
		//HAL_GPIO_WritePin(RLY1_GPIO_Port, RLY1_Pin, GPIO_PIN_SET);
	}
	if ((((sensors >> 16) ^ 1)&1) && (((sensors >> 17) ^1) & 1 ) && (((sensors >> 18) ^ 1)&1) && (((sensors >> 19) ^ 1 )&1))
	{
		//Turn ON Green LED for CH2
		HAL_GPIO_WritePin(CH2G_2_GPIO_Port, CH2G_2_Pin, GPIO_PIN_SET);
		//Turn Of Any Red LEDS, System is good.
		HAL_GPIO_WritePin(CH2R_1_GPIO_Port, CH2R_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CH2R_2_GPIO_Port, CH2R_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CH2R_3_GPIO_Port, CH2R_3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CH2R_4_GPIO_Port, CH2R_4_Pin, GPIO_PIN_RESET);
		CH2Rly = 0x00;
	}
	else
	{
		CH2Rly = 0x01;
		HAL_GPIO_WritePin(CH2G_2_GPIO_Port, CH2G_2_Pin, GPIO_PIN_RESET);
		//Each individual red led trigger CH2-1 -> CH1-1
		if((sensors >> 16) & 1)
		{
			HAL_GPIO_WritePin(CH2R_4_GPIO_Port, CH2R_4_Pin, GPIO_PIN_SET);
		}
		else 
		{
			HAL_GPIO_WritePin(CH2R_4_GPIO_Port, CH2R_4_Pin, GPIO_PIN_RESET);
		}
			
		if((sensors >> 17) & 1)
		{
			HAL_GPIO_WritePin(CH2R_3_GPIO_Port, CH2R_3_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(CH2R_3_GPIO_Port, CH2R_3_Pin, GPIO_PIN_RESET);
		}
		if((sensors >> 18) & 1)
		{
			HAL_GPIO_WritePin(CH2R_2_GPIO_Port, CH2R_2_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(CH2R_2_GPIO_Port, CH2R_2_Pin, GPIO_PIN_RESET);
		}
			
		if((sensors >> 19) & 1)
		{
			HAL_GPIO_WritePin(CH2R_1_GPIO_Port, CH2R_1_Pin, GPIO_PIN_SET);
		}
		else 
		{
			HAL_GPIO_WritePin(CH2R_1_GPIO_Port, CH2R_1_Pin, GPIO_PIN_RESET);
		}
		//HAL_GPIO_WritePin(RLY1_GPIO_Port, RLY1_Pin, GPIO_PIN_SET);
	}
	if ((((sensors >> 20) ^ 1)&1) && (((sensors >> 21) ^1  )&1) && (((sensors >> 22) ^ 1)&1) && (((sensors >> 23) ^1) &1))
	{
		//Turn ON Green LED for CH1
		HAL_GPIO_WritePin(CH1G_1_GPIO_Port, CH1G_1_Pin, GPIO_PIN_SET);
		//Turn Of Any Red LEDS, System is good.
		HAL_GPIO_WritePin(CH1R_1_GPIO_Port, CH1R_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CH1R_2_GPIO_Port, CH1R_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CH1R_3_GPIO_Port, CH1R_3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CH1R_4_GPIO_Port, CH1R_4_Pin, GPIO_PIN_RESET);
		CH1Rly = 0x00;
	}
	else
	{
		CH1Rly = 0x01;
		HAL_GPIO_WritePin(CH1G_1_GPIO_Port, CH1G_1_Pin, GPIO_PIN_RESET);
		//Each individual red led trigger CH1-1 -> CH1-1
		if((sensors >> 20) & 1)
		{
			HAL_GPIO_WritePin(CH1R_4_GPIO_Port, CH1R_4_Pin, GPIO_PIN_SET);
		}
		else 
		{
			HAL_GPIO_WritePin(CH1R_4_GPIO_Port, CH1R_4_Pin, GPIO_PIN_RESET);
		}
			
		if((sensors >> 21) & 1)
		{
			HAL_GPIO_WritePin(CH1R_3_GPIO_Port, CH1R_3_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(CH1R_3_GPIO_Port, CH1R_3_Pin, GPIO_PIN_RESET);
		}
		if((sensors >> 22) & 1)
		{
			HAL_GPIO_WritePin(CH1R_2_GPIO_Port, CH1R_2_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(CH1R_2_GPIO_Port, CH1R_2_Pin, GPIO_PIN_RESET);
		}
			
		if((sensors >> 23) & 1)
		{
			HAL_GPIO_WritePin(CH1R_1_GPIO_Port, CH1R_1_Pin, GPIO_PIN_SET);
		}
		else 
		{
			HAL_GPIO_WritePin(CH1R_1_GPIO_Port, CH1R_1_Pin, GPIO_PIN_RESET);
		}
		//HAL_GPIO_WritePin(RLY1_GPIO_Port, RLY1_Pin, GPIO_PIN_SET);
	}
	
	if((CH1Rly | CH2Rly | CH3Rly | CH4Rly | CH5Rly | CH6Rly))
	{
		HAL_GPIO_WritePin(RLY1_GPIO_Port, RLY1_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(RLY1_GPIO_Port, RLY1_Pin, GPIO_PIN_RESET);
	}
}

void buzz(uint32_t rounds)
{
	for (uint16_t x = 0; x< rounds; x++)
	{
		HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, GPIO_PIN_SET);
		//HAL_Delay(125);
		delayUS(150);
		HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, GPIO_PIN_RESET);
		//HAL_Delay(125);	
		delayUS(150);		
	}
}

void delayUS(uint32_t us)
{
	volatile uint32_t counter = 7*us;
	while(counter--);
}
	
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
