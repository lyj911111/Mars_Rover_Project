/*
 * Robot_jhm.h
 *
 *  Created on: 2018. 9. 22.
 *      Author: jhm
 */

#ifndef ROBOT_JHM_H_
#define ROBOT_JHM_H_

#include "stm32f4xx_hal.h"
#include "Robot_kjy.h"
#include "Robot_lwj.h"
#include "Robot_khy.h"

#define Calibration1 129
#define Calibration2 130
#define MAX_CHANNEL_NUM 7

#define RC_MAX          3800UL
#define RC_TOGGLE_MAX   3300UL
#define RC_MID1         3100UL
#define RC_MID2         2900UL
#define RC_TOGGLE_MIN   2700UL
#define RC_MIN          2200UL

#define SAFE_ROOF       50


void RC_Return_dutycycle(uint32_t GPIO_Pin);
uint32_t RC_Read(uint32_t interrupt);

#endif /* ROBOT_JHM_H_ */
/*          MAX             MIDDLE          MIN
 * CH1      3865~3871       3032~3077       2190~2196
 * CH2      3854~3859       3029~3078       2180~2186
 * CH3      3840~3850       3000~3040       2180~2200
 * CH4      3830~3850       3000~3040       2150~2180
 * CH5      3830~3850       3000~3040       2150~2180
 * CH6      3830~3850       3000~3040       2150~2180
 * CH7      3830~3850       3000~3040       2150~2180
 * */
/*
 * PE3 : 1ch    PE4 : 2ch    PE5 : 3ch      PE6 : 4ch
 * PF7 : 5ch    PF8 : 6ch    PF9 : 7ch
 * */















