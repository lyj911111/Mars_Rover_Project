/*
 * Robot_lwj.c
 *
 *  Created on: 2018. 9. 20.
 *      Author: Wonjae
 *
 *
 */
#include "Robot_lwj.h"

/*
 * 	바퀴 컨트롤 함수.
 * 	함수 사용법 : Wheel_Control(바퀴 선택, 바퀴 방향 , 바퀴 속도)
 *
 * 	위 3개의 인자를 받아서 바퀴를 선택하고, 그에 따른 앞 뒤 방향선택, 움직일 바퀴를 선택.
 * 	앞 뒤 방향 GPIO핀 설정. 바퀴 선 연결시 참고.
 * 	PC8		WhEEL_R_U
 * 	PC9		WhEEL_L_U
 * 	PC10 	WhEEL_R_M
 * 	PC11 	WhEEL_L_M
 *	PC12	WhEEL_R_D
 *	PD2 	WhEEL_L_D
 *
 *	함수 Parameter.
 *		첫번째 인자 : WHEEL_R_U , WHEEL_L_U, WHEEL_R_M, WHEEL_L_M, WHEEL_R_D, WHEEL_L_D 중에 하나.
 *		두번째 인자 : WHEEL_FORWARD , WHEEL_BACKWARD
 *		세번째 인자 : WHEEL_SPEED
 */

void Wheel_Contorl(const uint8_t Wheel_select, const uint8_t Wheel_direction, uint32_t PWM_Pulse)
{

	switch (Wheel_select) // 바퀴 선택
	{
	case WHEEL_R_U:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, Wheel_direction );	 //	앞뒤 방향 선택.
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
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, Wheel_direction );
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_Pulse);
		break;
	}

}

