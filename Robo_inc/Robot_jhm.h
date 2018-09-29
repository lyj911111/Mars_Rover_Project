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

#define Calibration1 129
#define Calibration2 130

void RC_Return_dutycycle(uint32_t GPIO_Pin);
uint32_t RC_Read(uint32_t interrupt);

#endif /* ROBOT_JHM_H_ */

