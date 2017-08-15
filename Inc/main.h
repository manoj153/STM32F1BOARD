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

#define LR22_Pin GPIO_PIN_2
#define LR22_GPIO_Port GPIOE
#define LR23_Pin GPIO_PIN_3
#define LR23_GPIO_Port GPIOE
#define LR24_Pin GPIO_PIN_4
#define LR24_GPIO_Port GPIOE
#define S1_Pin GPIO_PIN_4
#define S1_GPIO_Port GPIOA
#define S2_Pin GPIO_PIN_5
#define S2_GPIO_Port GPIOA
#define S3_Pin GPIO_PIN_6
#define S3_GPIO_Port GPIOA
#define S4_Pin GPIO_PIN_7
#define S4_GPIO_Port GPIOA
#define S5_Pin GPIO_PIN_4
#define S5_GPIO_Port GPIOC
#define S6_Pin GPIO_PIN_5
#define S6_GPIO_Port GPIOC
#define S7_Pin GPIO_PIN_0
#define S7_GPIO_Port GPIOB
#define S8_Pin GPIO_PIN_1
#define S8_GPIO_Port GPIOB
#define S9_Pin GPIO_PIN_2
#define S9_GPIO_Port GPIOB
#define S10_Pin GPIO_PIN_7
#define S10_GPIO_Port GPIOE
#define S11_Pin GPIO_PIN_8
#define S11_GPIO_Port GPIOE
#define S12_Pin GPIO_PIN_9
#define S12_GPIO_Port GPIOE
#define S13_Pin GPIO_PIN_10
#define S13_GPIO_Port GPIOE
#define S14_Pin GPIO_PIN_11
#define S14_GPIO_Port GPIOE
#define S15_Pin GPIO_PIN_12
#define S15_GPIO_Port GPIOE
#define S16_Pin GPIO_PIN_13
#define S16_GPIO_Port GPIOE
#define S17_Pin GPIO_PIN_14
#define S17_GPIO_Port GPIOE
#define S18_Pin GPIO_PIN_15
#define S18_GPIO_Port GPIOE
#define S19_Pin GPIO_PIN_10
#define S19_GPIO_Port GPIOB
#define S20_Pin GPIO_PIN_11
#define S20_GPIO_Port GPIOB
#define S21_Pin GPIO_PIN_12
#define S21_GPIO_Port GPIOB
#define S22_Pin GPIO_PIN_13
#define S22_GPIO_Port GPIOB
#define S23_Pin GPIO_PIN_14
#define S23_GPIO_Port GPIOB
#define S24_Pin GPIO_PIN_15
#define S24_GPIO_Port GPIOB
#define RLY1_Pin GPIO_PIN_11
#define RLY1_GPIO_Port GPIOD
#define Beep_Pin GPIO_PIN_12
#define Beep_GPIO_Port GPIOD
#define TESTpb_Pin GPIO_PIN_13
#define TESTpb_GPIO_Port GPIOD
#define MUTEpb_Pin GPIO_PIN_14
#define MUTEpb_GPIO_Port GPIOD
#define LG6_Pin GPIO_PIN_15
#define LG6_GPIO_Port GPIOD
#define LG5_Pin GPIO_PIN_6
#define LG5_GPIO_Port GPIOC
#define LG4_Pin GPIO_PIN_7
#define LG4_GPIO_Port GPIOC
#define LG3_Pin GPIO_PIN_8
#define LG3_GPIO_Port GPIOC
#define LG2_Pin GPIO_PIN_9
#define LG2_GPIO_Port GPIOC
#define LG1_Pin GPIO_PIN_8
#define LG1_GPIO_Port GPIOA
#define LR21_Pin GPIO_PIN_15
#define LR21_GPIO_Port GPIOA
#define LR20_Pin GPIO_PIN_10
#define LR20_GPIO_Port GPIOC
#define LR19_Pin GPIO_PIN_11
#define LR19_GPIO_Port GPIOC
#define LR18_Pin GPIO_PIN_12
#define LR18_GPIO_Port GPIOC
#define LR17_Pin GPIO_PIN_0
#define LR17_GPIO_Port GPIOD
#define LR16_Pin GPIO_PIN_1
#define LR16_GPIO_Port GPIOD
#define LR15_Pin GPIO_PIN_2
#define LR15_GPIO_Port GPIOD
#define LR14_Pin GPIO_PIN_3
#define LR14_GPIO_Port GPIOD
#define LR13_Pin GPIO_PIN_4
#define LR13_GPIO_Port GPIOD
#define LR12_Pin GPIO_PIN_5
#define LR12_GPIO_Port GPIOD
#define LR11_Pin GPIO_PIN_6
#define LR11_GPIO_Port GPIOD
#define LR10_Pin GPIO_PIN_7
#define LR10_GPIO_Port GPIOD
#define LR9_Pin GPIO_PIN_3
#define LR9_GPIO_Port GPIOB
#define LR8_Pin GPIO_PIN_4
#define LR8_GPIO_Port GPIOB
#define LR7_Pin GPIO_PIN_5
#define LR7_GPIO_Port GPIOB
#define LR6_Pin GPIO_PIN_6
#define LR6_GPIO_Port GPIOB
#define LR5_Pin GPIO_PIN_7
#define LR5_GPIO_Port GPIOB
#define LR4_Pin GPIO_PIN_8
#define LR4_GPIO_Port GPIOB
#define LR3_Pin GPIO_PIN_9
#define LR3_GPIO_Port GPIOB
#define LR2_Pin GPIO_PIN_0
#define LR2_GPIO_Port GPIOE
#define LR1_Pin GPIO_PIN_1
#define LR1_GPIO_Port GPIOE

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
