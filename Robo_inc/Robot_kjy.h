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

/*********************************************************/

#define BREAK_MAX PULSE_MAX
#define BREAK_MIN PULSE_MIN

#define GAIN_R_U    0
#define GAIN_L_U    0
#define GAIN_R_M    0
#define GAIN_L_M    0
#define GAIN_R_D    0
#define GAIN_L_D    0

#define FORWARD_BACKWARD    2
#define MOVE_SIDE           1
#define LIMIT_SPEED         5
#define CHANGE_GEAR         6

#define CH1             1
#define CH2             2
#define CH3             3
#define CH4             4
#define CH5             5
#define CH6             6
#define CH7             7

void BUGI_DriveMode(uint32_t mode);
void Wheel_Allbreak();
#endif

















