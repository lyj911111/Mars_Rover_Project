#ifndef _ROBOT_KJY_H
#define _ROBOT_KJY_H

#include "stm32f4xx_hal.h"
#include "Robot_lwj.h"
#include "Robot_jhm.h"
#include "Robot_khy.h"
#include "robotics_arm.h"

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

/*********************************************************/

#define BREAK_MAX PULSE_MAX
#define BREAK_MIN PULSE_MIN

#if WHEEL_FORWARD == 1
#define WHEEL_LEFT    0
#define WHEEL_RIGHT   1
#elif WHEEL_FORWARD == 0
#define WHEEL_LEFT    1
#define WHEEL_RIGHT   0
#endif

#define GAIN_R_U    0
#define GAIN_L_U    0
#define GAIN_R_M    0
#define GAIN_L_M    0
#define GAIN_R_D    0
#define GAIN_L_D    0

#define MOVE_LINE       2
#define MOVE_SIDE       4
#define LIMIT_SPEED     5
#define CHANGE_GEAR     6
#define EMERGENCY       7

#define CH1             1
#define CH2             2
#define CH3             3
#define CH4             4
#define CH5             5
#define CH6             6
#define CH7             7

#define MAX_PWM_ARM     250
#define MIN_PWM_ARM     50
/*  손 RC 범위 제한      */
#define MAX_GRAB        90
#define MIN_GRAB        55
#define INIT_GRAB       58
/*  손목 RC 범위 제한    */
#define MAX_WRIST       190
#define MIN_WRIST       50
#define INIT_WRIST      50
/*  카메라 RC 범위 제한  */
#define MAX_CAMERA      100
#define MIN_CAMERA      50
#define INIT_CAMERA     75

#define SHOULDER_ID           0x01
#define ELBOW_ID              0x02


#define MAX_SERVO_ROBOTICS    4095
#define MIN_SERVO_ROBOTICS    0

/*  어깨 속도   */
#define MAX_SHOULDER_SPEED          7
#define MID_SHOULDER_SPEED          0
#define MIN_SHOULDER_SPEED         -7
/*  팔꿈치 속도  */
#define MAX_ELBOW_SPEED             7
#define MID_ELBOW_SPEED             0
#define MIN_ELBOW_SPEED            -7


#define GRAB_MOVE             1
#define WRIST_MOVE            4
#define ELBOW_POSTION         0x0A
#define SHOULDER_POSTION      0x0A

void BUGI_DriveMode(uint32_t mode);
uint32_t Mode_select();

void ARM_init();
void ARM_enable(uint8_t ID);


#endif

















