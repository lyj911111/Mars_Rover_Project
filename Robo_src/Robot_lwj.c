/*
 * Robot_lwj.c
 *
 *  Created on: 2018. 9. 20.
 *      Author: Wonjae
 *
 *
 */
#include "Robot_lwj.h"

//////////////////////////////////////////////////////////////
/*
 * 	바퀴 컨트롤 함수.
 * 	함수 사용법 : Wheel_Control(바퀴 선택, 바퀴 방향/멈춤 , 바퀴 속도)
 *
 * 	위 3개의 인자를 받아서 바퀴를 선택하고, 그에 따른 앞 뒤 방향선택 또는 멈춤선택, 바퀴속도를 제어 0~180범위
 * 	바퀴 선 연결시 참고.
 * 	[GPIO핀]  [타이머핀]	[바퀴]
 * 	PC8      PE9        WhEEL_R_U
 * 	PC9      PE11       WhEEL_L_U
 * 	PC10     PE13       WhEEL_R_M
 * 	PC11     PE14       WhEEL_L_M
 *	PC12     PA5        WhEEL_R_D
 *	PD2      PB10       WhEEL_L_D
 *
 *	함수 Parameter.
 *		첫번째 인자 : WHEEL_R_U , WHEEL_L_U, WHEEL_R_M, WHEEL_L_M, WHEEL_R_D, WHEEL_L_D 중에 하나.
 *		두번째 인자 : WHEEL_STOP, WHEEL_FORWARD , WHEEL_BACKWARD
 *		세번째 인자 : Wheel_Speed
 */
void Wheel_Allbreak()
{
    Wheel_Break(WHEEL_R_U, BREAK_MAX);
    Wheel_Break(WHEEL_L_U, BREAK_MAX);
    Wheel_Break(WHEEL_R_M, BREAK_MAX);
    Wheel_Break(WHEEL_L_M, BREAK_MAX);
    Wheel_Break(WHEEL_R_D, BREAK_MAX);
    Wheel_Break(WHEEL_L_D, BREAK_MAX);
}

void Wheel_Contorl(const uint8_t Wheel_select, const uint8_t Wheel_direction, uint32_t PWM_Pulse) // 타이머 Pulse 최대값: 180 (최고속도)
{

	switch (Wheel_select) // 바퀴 선택
	{
	case WHEEL_R_U:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, Wheel_direction );  //	앞뒤 방향 선택.
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_Pulse); //	속도 제어.
		break;
	case WHEEL_L_U:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, Wheel_direction );
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_Pulse);
		break;
	case WHEEL_R_M:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, Wheel_direction );
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_Pulse);
		break;
	case WHEEL_L_M:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, Wheel_direction );
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_Pulse);
		break;
	case WHEEL_R_D:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, Wheel_direction );
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_Pulse);
		break;
	case WHEEL_L_D:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, Wheel_direction );
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_Pulse);
		break;
	}

}

//////////////////////////////////////////////////////////////
/*
 * 	바퀴 브레이크 함수.
 * 	함수 사용법 : Wheel_Break(바퀴 선택, PWM파형)
 *
 * 	위 2개의 인자를 받음. 바퀴를 선택하고, 바퀴속도 Break를 하는 PWM파형을 출력.
 * 	바퀴 선 연결시 참고.
 * 	[타이머핀]	[바퀴]
 * 	PD12    WhEEL_R_U
 * 	PD13    WhEEL_L_U
 * 	PD14    WhEEL_R_M
 * 	PD15    WhEEL_L_M
 *	PA0     WhEEL_R_D
 *	PA2     WhEEL_L_D
 *
 *	함수 Parameter.
 *		첫번째 인자 : WHEEL_R_U , WHEEL_L_U, WHEEL_R_M, WHEEL_L_M, WHEEL_R_D, WHEEL_L_D 중에 하나.
 *		두번째 인자 : Break_PWM
 */

void Wheel_Break(const uint8_t Wheel_select, uint32_t Break_PWM)
{
	switch (Wheel_select) // 바퀴 선택
	{
	case WHEEL_R_U:
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, Break_PWM);
		break;
	case WHEEL_L_U:
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, Break_PWM);
		break;
	case WHEEL_R_M:
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, Break_PWM);
		break;
	case WHEEL_L_M:
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, Break_PWM);
		break;
	case WHEEL_R_D:
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, Break_PWM);
		break;
	case WHEEL_L_D:
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, Break_PWM);
		break;
	}
}





