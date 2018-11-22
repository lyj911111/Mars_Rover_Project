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

#define PING            0x01  //Ping  Packet ID�� ������ ID�� ���� ��ġ�� Packet�� �����ߴ��� ���� Ȯ���� ���� Instruction
#define READ            0x02  //Read  ��ġ�κ��� �����͸� �о���� ���� Instruction
#define WRITE           0x03  //Write ��ġ�� �����͸� ���� ���� Instruction
#define REG             0x04  //Reg Write Instruction Packet�� ��� ���·� ����ϴ� Instruction, Action ��ɿ� ���� �����
#define ACTION          0x05  //Action  Reg Write �� �̸� ����� Packet�� �����ϴ� Instruction
#define FACTORY_RESET   0x06  //Factory Reset ��Ʈ�����̺��� ���� ���� ������ �ʱⰪ���� �ǵ����� Instruction
#define REBOOT          0x08  //Reboot  ��ġ�� ����� ��Ű�� Instruction
#define CLEAR           0x10  //Clear ��ġ�� Ư�� ���¸� �����ϴ� Instruction
#define STATUS          0x55  //Status(Return)  Instruction Packet �� ���� Return Instruction
#define SYNC_READ       0x82  //Sync Read �ټ��� ��ġ�� ���ؼ�, ������ Address���� ������ ������ �����͸� �� ���� �б� ���� Instruction
#define SYNC_WRITE      0x83  //Sync Write  �ټ��� ��ġ�� ���ؼ�, ������ Address�� ������ ������ �����͸� �� ���� ���� ���� Instruction
#define BULK_READ       0x92  //Bulk Read �ټ��� ��ġ�� ���ؼ�, ���� �ٸ� Address���� ���� �ٸ� ������ �����͸� �� ���� �б� ���� Instruction
#define BULK_WRITE      0x93  //Bulk Write  �ټ��� ��ġ�� ���ؼ�, ���� �ٸ� Address�� ���� �ٸ� ������ �����͸� �ѹ��� ���� ���� Instruction


#define TORQUE_EN       64
#define GOAL_PWM        100
#define GOAL_SPEED      104
#define GOAL_DEGREE     116   // 0~4095  1�� 0.088��

#define SERVO_MAX_ROBOTICS    4095
#define SERVO_MIN_ROBOTICS    0

#define SERVO1_ID       0x01
#define SERVO2_ID       0x01

#endif

















