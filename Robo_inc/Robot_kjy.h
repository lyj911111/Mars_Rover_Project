#ifndef _ROBOT_KJY_H
#define _ROBOT_KJY_H

#include "stm32f4xx_hal.h"
#include "Robot_lwj.h"
#include "Robot_jhm.h"
#include "Robot_khy.h"

#define STEP_PUL_PORT       GPIOB
#define STEP_PUL_PINNUM     GPIO_PIN_11
#define STEP_ENA_PORT       GPIOB
#define STEP_ENA_PINNUM     GPIO_PIN_13
#define STEP_DIR_PORT       GPIOB
#define STEP_DIR_PINNUM     GPIO_PIN_12

#define STEP_FORWARD 1
#define STEP_BACKWARD 0

#define STEP_FORWARD_RC     3200
#define STEP_BACKWARD_RC    2800
void ARM_Move_motor(TIM_HandleTypeDef *htim,uint32_t RC_instance);
void ARM_Generation_pulse(TIM_HandleTypeDef* htim);
#endif
