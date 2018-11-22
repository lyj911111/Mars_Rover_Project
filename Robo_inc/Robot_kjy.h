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
void ARM_Move_motor(TIM_HandleTypeDef *htim,uint32_t RC_instance);
void ARM_Generation_pulse(TIM_HandleTypeDef* htim);
void servo_enable(uint8_t id,uint8_t en_ok);

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

#define MAX_PWM_ARM     80
#define MIN_PWM_ARM     50

void BUGI_DriveMode(uint32_t mode);
uint32_t Mode_select();

#define PING            0x01  //Ping  Packet ID와 동일한 ID를 갖은 장치에 Packet이 도달했는지 여부 확인을 위한 Instruction
#define READ            0x02  //Read  장치로부터 데이터를 읽어오기 위한 Instruction
#define WRITE           0x03  //Write 장치에 데이터를 쓰기 위한 Instruction
#define REG             0x04  //Reg Write Instruction Packet을 대기 상태로 등록하는 Instruction, Action 명령에 의해 실행됨
#define ACTION          0x05  //Action  Reg Write 로 미리 등록한 Packet을 실행하는 Instruction
#define FACTORY_RESET   0x06  //Factory Reset 컨트롤테이블을 공장 출하 상태의 초기값으로 되돌리는 Instruction
#define REBOOT          0x08  //Reboot  장치를 재부팅 시키는 Instruction
#define CLEAR           0x10  //Clear 장치의 특정 상태를 해제하는 Instruction
#define STATUS          0x55  //Status(Return)  Instruction Packet 에 대한 Return Instruction
#define SYNC_READ       0x82  //Sync Read 다수의 장치에 대해서, 동일한 Address에서 동일한 길이의 데이터를 한 번에 읽기 위한 Instruction
#define SYNC_WRITE      0x83  //Sync Write  다수의 장치에 대해서, 동일한 Address에 동일한 길이의 데이터를 한 번에 쓰기 위한 Instruction
#define BULK_READ       0x92  //Bulk Read 다수의 장치에 대해서, 서로 다른 Address에서 서로 다른 길이의 데이터를 한 번에 읽기 위한 Instruction
#define BULK_WRITE      0x93  //Bulk Write  다수의 장치에 대해서, 서로 다른 Address에 서로 다른 길이의 데이터를 한번에 쓰기 위한 Instruction


#define TORQUE_EN       64
#define GOAL_PWM        100
#define GOAL_SPEED      104
#define GOAL_DEGREE     116   // 0~4095  1당 0.088도

#define SERVO_MAX_ROBOTICS    4095
#define SERVO_MIN_ROBOTICS    0

#define SERVO1_ID       0x01
#define SERVO2_ID       0x01

#endif

















