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
 * 	���� ��Ʈ�� �Լ�.
 * 	�Լ� ���� : Wheel_Control(���� ����, ���� ���� , ���� �ӵ�)
 *
 * 	�� 3���� ���ڸ� �޾Ƽ� ������ �����ϰ�, �׿� ���� �� �� ���⼱��, ������ ������ ����.
 * 	�� �� ���� GPIO�� ����. ���� �� ����� ����.
 * 	PC8		WhEEL_R_U
 * 	PC9		WhEEL_L_U
 * 	PC10 	WhEEL_R_M
 * 	PC11 	WhEEL_L_M
 *	PC12	WhEEL_R_D
 *	PD2 	WhEEL_L_D
 *
 *	�Լ� Parameter.
 *		ù��° ���� : WHEEL_R_U , WHEEL_L_U, WHEEL_R_M, WHEEL_L_M, WHEEL_R_D, WHEEL_L_D �߿� �ϳ�.
 *		�ι�° ���� : WHEEL_FORWARD , WHEEL_BACKWARD
 *		����° ���� : WHEEL_SPEED
 */

void Wheel_Contorl(const uint8_t Wheel_select, const uint8_t Wheel_direction, uint32_t PWM_Pulse)
{

	switch (Wheel_select) // ���� ����
	{
	case WHEEL_R_U:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, Wheel_direction );	 //	�յ� ���� ����.
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_Pulse); //	�ӵ� ����.
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

