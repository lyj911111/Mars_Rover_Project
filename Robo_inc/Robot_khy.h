/*
 * Robot_khy.h
 *
 *  Created on: 2018. 9. 28.
 *      Author: khy
 */

#ifndef ROBOT_KHY_H_
#define ROBOT_KHY_H_

#include "stm32f4xx_hal.h"


/*
 * from_min�� from_max ������ value��  from_min�� from_max ������ ������ �����Ͽ� ����
 * */
uint32_t math_Map(uint32_t val, uint32_t from_min, uint32_t from_max, uint32_t to_min, uint32_t to_max);




#endif /* ROBOT_KHY_H_ */
