/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define CH6_2R_Pin GPIO_PIN_2
#define CH6_2R_GPIO_Port GPIOE
#define CH6_3R_Pin GPIO_PIN_3
#define CH6_3R_GPIO_Port GPIOE
#define CH6_4R_Pin GPIO_PIN_4
#define CH6_4R_GPIO_Port GPIOE
#define CH1_1_Pin GPIO_PIN_4
#define CH1_1_GPIO_Port GPIOA
#define CH1_2_Pin GPIO_PIN_5
#define CH1_2_GPIO_Port GPIOA
#define CH1_3_Pin GPIO_PIN_6
#define CH1_3_GPIO_Port GPIOA
#define CH1_4_Pin GPIO_PIN_7
#define CH1_4_GPIO_Port GPIOA
#define CH2_1_Pin GPIO_PIN_4
#define CH2_1_GPIO_Port GPIOC
#define CH2_2_Pin GPIO_PIN_5
#define CH2_2_GPIO_Port GPIOC
#define CH2_3_Pin GPIO_PIN_0
#define CH2_3_GPIO_Port GPIOB
#define CH2_4_Pin GPIO_PIN_1
#define CH2_4_GPIO_Port GPIOB
#define CH3_1_Pin GPIO_PIN_2
#define CH3_1_GPIO_Port GPIOB
#define CH3_2_Pin GPIO_PIN_7
#define CH3_2_GPIO_Port GPIOE
#define CH3_3_Pin GPIO_PIN_8
#define CH3_3_GPIO_Port GPIOE
#define CH3_4_Pin GPIO_PIN_9
#define CH3_4_GPIO_Port GPIOE
#define CH4_1_Pin GPIO_PIN_10
#define CH4_1_GPIO_Port GPIOE
#define CH4_2_Pin GPIO_PIN_11
#define CH4_2_GPIO_Port GPIOE
#define CH4_3_Pin GPIO_PIN_12
#define CH4_3_GPIO_Port GPIOE
#define CH4_4_Pin GPIO_PIN_13
#define CH4_4_GPIO_Port GPIOE
#define CH5_1_Pin GPIO_PIN_14
#define CH5_1_GPIO_Port GPIOE
#define CH5_2_Pin GPIO_PIN_15
#define CH5_2_GPIO_Port GPIOE
#define CH5_3_Pin GPIO_PIN_10
#define CH5_3_GPIO_Port GPIOB
#define CH5_4_Pin GPIO_PIN_11
#define CH5_4_GPIO_Port GPIOB
#define CH6_1_Pin GPIO_PIN_12
#define CH6_1_GPIO_Port GPIOB
#define CH6_2_Pin GPIO_PIN_13
#define CH6_2_GPIO_Port GPIOB
#define CH6_3_Pin GPIO_PIN_14
#define CH6_3_GPIO_Port GPIOB
#define CH6_4_Pin GPIO_PIN_15
#define CH6_4_GPIO_Port GPIOB
#define PWR_SYSF_Pin GPIO_PIN_8
#define PWR_SYSF_GPIO_Port GPIOD
#define ASYS_F_Pin GPIO_PIN_9
#define ASYS_F_GPIO_Port GPIOD
#define PWR_LED_Pin GPIO_PIN_10
#define PWR_LED_GPIO_Port GPIOD
#define RLY1_Pin GPIO_PIN_11
#define RLY1_GPIO_Port GPIOD
#define Beep_Pin GPIO_PIN_12
#define Beep_GPIO_Port GPIOD
#define TESTpb_Pin GPIO_PIN_13
#define TESTpb_GPIO_Port GPIOD
#define MUTEpb_Pin GPIO_PIN_14
#define MUTEpb_GPIO_Port GPIOD
#define CH6_G_Pin GPIO_PIN_15
#define CH6_G_GPIO_Port GPIOD
#define CH5_G_Pin GPIO_PIN_6
#define CH5_G_GPIO_Port GPIOC
#define CH4_G_Pin GPIO_PIN_7
#define CH4_G_GPIO_Port GPIOC
#define CH3_G_Pin GPIO_PIN_8
#define CH3_G_GPIO_Port GPIOC
#define CH2_G_Pin GPIO_PIN_9
#define CH2_G_GPIO_Port GPIOC
#define CH1_G_Pin GPIO_PIN_8
#define CH1_G_GPIO_Port GPIOA
#define CH6_1R_Pin GPIO_PIN_15
#define CH6_1R_GPIO_Port GPIOA
#define CH5_4R_Pin GPIO_PIN_10
#define CH5_4R_GPIO_Port GPIOC
#define CH5_3R_Pin GPIO_PIN_11
#define CH5_3R_GPIO_Port GPIOC
#define CH5_2R_Pin GPIO_PIN_12
#define CH5_2R_GPIO_Port GPIOC
#define CH5_1R_Pin GPIO_PIN_0
#define CH5_1R_GPIO_Port GPIOD
#define CH4_4R_Pin GPIO_PIN_1
#define CH4_4R_GPIO_Port GPIOD
#define CH4_3R_Pin GPIO_PIN_2
#define CH4_3R_GPIO_Port GPIOD
#define CH4_2R_Pin GPIO_PIN_3
#define CH4_2R_GPIO_Port GPIOD
#define CH4_1R_Pin GPIO_PIN_4
#define CH4_1R_GPIO_Port GPIOD
#define CH3_4R_Pin GPIO_PIN_5
#define CH3_4R_GPIO_Port GPIOD
#define CH3_3R_Pin GPIO_PIN_6
#define CH3_3R_GPIO_Port GPIOD
#define CH3_2R_Pin GPIO_PIN_7
#define CH3_2R_GPIO_Port GPIOD
#define CH3_1R_Pin GPIO_PIN_3
#define CH3_1R_GPIO_Port GPIOB
#define CH2_4R_Pin GPIO_PIN_4
#define CH2_4R_GPIO_Port GPIOB
#define CH2_3R_Pin GPIO_PIN_5
#define CH2_3R_GPIO_Port GPIOB
#define CH2_2R_Pin GPIO_PIN_6
#define CH2_2R_GPIO_Port GPIOB
#define CH2_1R_Pin GPIO_PIN_7
#define CH2_1R_GPIO_Port GPIOB
#define CH1_4R_Pin GPIO_PIN_8
#define CH1_4R_GPIO_Port GPIOB
#define CH1_3R_Pin GPIO_PIN_9
#define CH1_3R_GPIO_Port GPIOB
#define CH1_2R_Pin GPIO_PIN_0
#define CH1_2R_GPIO_Port GPIOE
#define CH1_1R_Pin GPIO_PIN_1
#define CH1_1R_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
