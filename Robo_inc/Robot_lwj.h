#ifndef _ROBOT_LWJ_H
#define _ROBOT_LWJ_H

#include "stm32f4xx_hal.h"

#define WHEEL_STOP		0	//	정지
#define WHEEL_FORWARD 	1	//	전진방향
#define WHEEL_BACKWARD 	2	//	후진방향

// 약어 뜻. R: Right , L: Left , M: Middle , U: Up , D: Down
enum{
	WHEEL_R_U , WHEEL_L_U, WHEEL_R_M, WHEEL_L_M, WHEEL_R_D, WHEEL_L_D
};

TIM_HandleTypeDef htim1;	//	앞바퀴, 중간바퀴 Pulse제어 (최대값 180)
TIM_HandleTypeDef htim2;	//	뒷바퀴 Pulse제어 (최대값 180)

void Wheel_Contorl(const uint8_t Wheel_select, const uint8_t Wheel_direction, uint32_t PWM_Pulse);

#endif
