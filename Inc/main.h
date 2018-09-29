/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define GPIO_EXTI3_Pin GPIO_PIN_3
#define GPIO_EXTI3_GPIO_Port GPIOE
#define GPIO_EXTI3_EXTI_IRQn EXTI3_IRQn
#define GPIO_EXTI4_Pin GPIO_PIN_4
#define GPIO_EXTI4_GPIO_Port GPIOE
#define GPIO_EXTI4_EXTI_IRQn EXTI4_IRQn
#define GPIO_EXTI5_Pin GPIO_PIN_5
#define GPIO_EXTI5_GPIO_Port GPIOE
#define GPIO_EXTI5_EXTI_IRQn EXTI9_5_IRQn
#define GPIO_EXTI6_Pin GPIO_PIN_6
#define GPIO_EXTI6_GPIO_Port GPIOE
#define GPIO_EXTI6_EXTI_IRQn EXTI9_5_IRQn
#define User_Button_Pin GPIO_PIN_13
#define User_Button_GPIO_Port GPIOC
#define GPIO_EXTI7_Pin GPIO_PIN_7
#define GPIO_EXTI7_GPIO_Port GPIOF
#define GPIO_EXTI7_EXTI_IRQn EXTI9_5_IRQn
#define GPIO_EXTI8_Pin GPIO_PIN_8
#define GPIO_EXTI8_GPIO_Port GPIOF
#define GPIO_EXTI8_EXTI_IRQn EXTI9_5_IRQn
#define GPIO_EXTI9_Pin GPIO_PIN_9
#define GPIO_EXTI9_GPIO_Port GPIOF
#define GPIO_EXTI9_EXTI_IRQn EXTI9_5_IRQn
#define Red_LED_Pin GPIO_PIN_14
#define Red_LED_GPIO_Port GPIOB
#define Blue_LED_Pin GPIO_PIN_7
#define Blue_LED_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define TOGGLE_CH   6
#define LIMIT_CH    5
/* ä�ΰ�����
 * PE3 : 1ch    PE4 : 2ch    PE5 : 3ch      PE6 : 4ch
 * PF7 : 5ch    PF8 : 6ch    PF9 : 7ch
 * */
/*  ���� �⵿ PWM
 *  [GPIO��]  [Ÿ�̸���] [����]
 *  PC8       PE9       WhEEL_R_U
 *  PC9       PE11      WhEEL_L_U
 *  PC10      PE13      WhEEL_R_M
 *  PC11      PE14      WhEEL_L_M
 *  PC12      PA5       WhEEL_R_D
 *  PD2       PB10      WhEEL_L_D
 *
 *  ���� ���� PWM
 * [Ÿ�̸���]  [����]
 *  PD12    WhEEL_R_U
 *  PD13    WhEEL_L_U
 *  PD14    WhEEL_R_M
 *  PD15    WhEEL_L_M
 *  PA0     WhEEL_R_D
 *  PA1     WhEEL_L_D
 */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
