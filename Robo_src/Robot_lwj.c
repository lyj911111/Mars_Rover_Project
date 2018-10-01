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
 * 	���� ��Ʈ�� �Լ�.
 * 	�Լ� ���� : Wheel_Control(���� ����, ���� ����/���� , ���� �ӵ�)
 *
 * 	�� 3���� ���ڸ� �޾Ƽ� ������ �����ϰ�, �׿� ���� �� �� ���⼱�� �Ǵ� ���㼱��, �����ӵ��� ���� 0~180����
 * 	���� �� ����� ����.
 * 	[GPIO��]  [Ÿ�̸���]	[����]
 * 	PC8      PE9        WhEEL_R_U
 * 	PC9      PE11       WhEEL_L_U
 * 	PC10     PE13       WhEEL_R_M
 * 	PC11     PE14       WhEEL_L_M
 *	PC12     PA5        WhEEL_R_D
 *	PD2      PB10       WhEEL_L_D
 *
 *	�Լ� Parameter.
 *		ù��° ���� : WHEEL_R_U , WHEEL_L_U, WHEEL_R_M, WHEEL_L_M, WHEEL_R_D, WHEEL_L_D �߿� �ϳ�.
 *		�ι�° ���� : WHEEL_STOP, WHEEL_FORWARD , WHEEL_BACKWARD
 *		����° ���� : Wheel_Speed
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

void Wheel_Contorl(const uint8_t Wheel_select, const uint8_t Wheel_direction, uint32_t PWM_Pulse) // Ÿ�̸� Pulse �ִ밪: 180 (�ְ�ӵ�)
{

	switch (Wheel_select) // ���� ����
	{
	case WHEEL_R_U:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, Wheel_direction );  //	�յ� ���� ����.
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
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, Wheel_direction );
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_Pulse);
		break;
	}

}

//////////////////////////////////////////////////////////////
/*
 * 	���� �극��ũ �Լ�.
 * 	�Լ� ���� : Wheel_Break(���� ����, PWM����)
 *
 * 	�� 2���� ���ڸ� ����. ������ �����ϰ�, �����ӵ� Break�� �ϴ� PWM������ ���.
 * 	���� �� ����� ����.
 * 	[Ÿ�̸���]	[����]
 * 	PD12    WhEEL_R_U
 * 	PD13    WhEEL_L_U
 * 	PD14    WhEEL_R_M
 * 	PD15    WhEEL_L_M
 *	PA0     WhEEL_R_D
 *	PA2     WhEEL_L_D
 *
 *	�Լ� Parameter.
 *		ù��° ���� : WHEEL_R_U , WHEEL_L_U, WHEEL_R_M, WHEEL_L_M, WHEEL_R_D, WHEEL_L_D �߿� �ϳ�.
 *		�ι�° ���� : Break_PWM
 */

void Wheel_Break(const uint8_t Wheel_select, uint32_t Break_PWM)
{
	switch (Wheel_select) // ���� ����
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





