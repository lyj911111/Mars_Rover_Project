/*
 * Robot_jhm.c
 *
 *  Created on: 2018. 9. 22.
 *      Author: jhm
 */


#include "Robot_jhm.h"

int RC_Return_dutycycle(int rc_dutycycle){
	static uint32_t t3_tick = 0;			//t3_tick: 1.00us
	static uint8_t flag = 0;
	static int cycle1, cycle2;

	t3_tick = TIM3->CNT;					//tim3 count register

	if(flag){
		flag = 0, cycle1 = t3_tick;
	}
	else{
		flag = 1, cycle2 = t3_tick;
	}

	TIM3->CNT = 0;

	if(cycle1 < cycle2) rc_dutycycle = cycle1;
	else 				rc_dutycycle = cycle2;

	return rc_dutycycle;
}
