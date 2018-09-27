/*
 * Robot_jhm.c
 *
 *  Created on: 2018. 9. 22.
 *      Author: jhm
 */


#include "Robot_jhm.h"

//int rc_dutycycle_1;
//int rc_dutycycle_2;
//int rc_dutycycle_3;
//int rc_dutycycle_4;
//int rc_dutycycle_5;
//int rc_dutycycle_6;
//int rc_dutycycle_7;

void RC_Return_dutycycle(int rc_dutycycle){
	static uint8_t flag = 0;
	static int high_width, low_width;

	if(flag){
		flag = 0, high_width = TIM3->CNT;
	}
	else{
		flag = 1, low_width = TIM3->CNT;
	}

	TIM3->CNT = 0;

	if(high_width < low_width) 	rc_dutycycle = high_width;
	else						rc_dutycycle = low_width;

//	if(high_width < low_width) rc_dutycycle = high_width;
//	else 					   rc_dutycycle = low_width;
//
//	return rc_dutycycle;
}
