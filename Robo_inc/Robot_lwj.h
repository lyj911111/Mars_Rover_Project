#ifndef _ROBOT_LWJ_H
#define _ROBOT_LWJ_H

#include "stm32f4xx_hal.h"
#include "Robot_kjy.h"

#define WHEEL_FORWARD   1	//	��������
#define WHEEL_BACKWARD  0	//	��������


TIM_HandleTypeDef htim1;	//	�չ���, �߰����� Pulse���� (�ִ밪 180)
TIM_HandleTypeDef htim2;	//	�޹��� Pulse���� (�ִ밪 180)
TIM_HandleTypeDef htim4;	//	�չ���, �߰����� �극��ũ Pulse���� (�ִ밪 180) 
TIM_HandleTypeDef htim5;	//	�극��ũ �޹��� Pulse���� (�ִ밪 180)


// ��� ��. R: Right , L: Left , M: Middle , U: Up , D: Down
#define WHEEL_R_U 0
#define WHEEL_L_U 1
#define WHEEL_R_M 2
#define WHEEL_L_M 3
#define WHEEL_R_D 4
#define WHEEL_L_D 5

#define PULSE_MAX 180
#define PULSE_MIN 0
void Wheel_Contorl(const uint8_t Wheel_select, const uint8_t Wheel_direction, uint32_t PWM_Pulse);
void Wheel_Break(const uint8_t Wheel_select, uint32_t Break_PWM);
void Wheel_Allbreak();
void Wheel_AllbreakX(uint32_t break_pulse);
void Wheel_AllSpeedX(uint32_t speed_pulse,uint32_t direction);
void Watch_Dog(void);

#endif
