/**
  ******************************************************************************
  * File Name          : main.hpp
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

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define CH6R_2_Pin GPIO_PIN_2
#define CH6R_2_GPIO_Port GPIOE
#define CH6R_3_Pin GPIO_PIN_3
#define CH6R_3_GPIO_Port GPIOE
#define CH6R_4_Pin GPIO_PIN_4
#define CH6R_4_GPIO_Port GPIOE
#define ALLGBYP_7_Pin GPIO_PIN_5
#define ALLGBYP_7_GPIO_Port GPIOE
#define CH6GBYP_6_Pin GPIO_PIN_6
#define CH6GBYP_6_GPIO_Port GPIOE
#define CH5GBYP_5_Pin GPIO_PIN_13
#define CH5GBYP_5_GPIO_Port GPIOC
#define CH4GBYP_4_Pin GPIO_PIN_0
#define CH4GBYP_4_GPIO_Port GPIOC
#define CH3GBYP_3_Pin GPIO_PIN_1
#define CH3GBYP_3_GPIO_Port GPIOC
#define CH2GBYP_2_Pin GPIO_PIN_2
#define CH2GBYP_2_GPIO_Port GPIOC
#define CH1GBYP_1_Pin GPIO_PIN_3
#define CH1GBYP_1_GPIO_Port GPIOC
#define RSV_Pin GPIO_PIN_0
#define RSV_GPIO_Port GPIOA
#define RTS_Pin GPIO_PIN_1
#define RTS_GPIO_Port GPIOA
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
#define CH6G_6_Pin GPIO_PIN_15
#define CH6G_6_GPIO_Port GPIOD
#define CH5G_5_Pin GPIO_PIN_6
#define CH5G_5_GPIO_Port GPIOC
#define CH4G_4_Pin GPIO_PIN_7
#define CH4G_4_GPIO_Port GPIOC
#define CH3G_3_Pin GPIO_PIN_8
#define CH3G_3_GPIO_Port GPIOC
#define CH2G_2_Pin GPIO_PIN_9
#define CH2G_2_GPIO_Port GPIOC
#define CH1G_1_Pin GPIO_PIN_8
#define CH1G_1_GPIO_Port GPIOA
#define PMUTEPB_Pin GPIO_PIN_9
#define PMUTEPB_GPIO_Port GPIOA
#define CONN_FAIL_Pin GPIO_PIN_10
#define CONN_FAIL_GPIO_Port GPIOA
#define BATT_FAIL_Pin GPIO_PIN_11
#define BATT_FAIL_GPIO_Port GPIOA
#define AC_FAIL_Pin GPIO_PIN_12
#define AC_FAIL_GPIO_Port GPIOA
#define CH6R_1_Pin GPIO_PIN_15
#define CH6R_1_GPIO_Port GPIOA
#define CH5R_4_Pin GPIO_PIN_10
#define CH5R_4_GPIO_Port GPIOC
#define CH5R_3_Pin GPIO_PIN_11
#define CH5R_3_GPIO_Port GPIOC
#define CH5R_2_Pin GPIO_PIN_12
#define CH5R_2_GPIO_Port GPIOC
#define CH5R_1_Pin GPIO_PIN_0
#define CH5R_1_GPIO_Port GPIOD
#define CH4R_4_Pin GPIO_PIN_1
#define CH4R_4_GPIO_Port GPIOD
#define CH4R_3_Pin GPIO_PIN_2
#define CH4R_3_GPIO_Port GPIOD
#define CH4R_2_Pin GPIO_PIN_3
#define CH4R_2_GPIO_Port GPIOD
#define CH4R_1_Pin GPIO_PIN_4
#define CH4R_1_GPIO_Port GPIOD
#define CH3R_4_Pin GPIO_PIN_5
#define CH3R_4_GPIO_Port GPIOD
#define CH3R_3_Pin GPIO_PIN_6
#define CH3R_3_GPIO_Port GPIOD
#define CH3R_2_Pin GPIO_PIN_7
#define CH3R_2_GPIO_Port GPIOD
#define CH3R_1_Pin GPIO_PIN_3
#define CH3R_1_GPIO_Port GPIOB
#define CH2R_4_Pin GPIO_PIN_4
#define CH2R_4_GPIO_Port GPIOB
#define CH2R_3_Pin GPIO_PIN_5
#define CH2R_3_GPIO_Port GPIOB
#define CH2R_2_Pin GPIO_PIN_6
#define CH2R_2_GPIO_Port GPIOB
#define CH2R_1_Pin GPIO_PIN_7
#define CH2R_1_GPIO_Port GPIOB
#define CH1R_4_Pin GPIO_PIN_8
#define CH1R_4_GPIO_Port GPIOB
#define CH1R_3_Pin GPIO_PIN_9
#define CH1R_3_GPIO_Port GPIOB
#define CH1R_2_Pin GPIO_PIN_0
#define CH1R_2_GPIO_Port GPIOE
#define CH1R_1_Pin GPIO_PIN_1
#define CH1R_1_GPIO_Port GPIOE

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
