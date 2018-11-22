/*
 * robotics_arm.h
 *
 *  Created on: 2018. 11. 21.
 *      Author: JuYeong
 */

#include "stm32f4xx_hal.h"

typedef unsigned char byte;

#ifndef ROBOTICS_ARM_H_
#define ROBOTICS_ARM_H_


/*    �������� 2.0 ��Ŷ ����
Header1 Header2 Header3 Reserved  Packet_ID Length1 Length2 Instruction Param Param Param CRC1  CRC2
  0xFF   0xFF    0xFD     0x00       ID      Len_L   Len_H  Instruction Param 1 �� Param N CRC_L CRC_H
*/

/*  �κ�Ƽ�� ��������2.0 command ����    */
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


/*  �������� �Ѱ� �� ID  */
#define GOAL_DEGREE     116   // 0~4095  1�� 0.088��

#define MAX_SERVO_ROBOTICS    4095
#define MIN_SERVO_ROBOTICS    0

#define MAX_SHOULDER      3000
#define MIN_SHOULDER      1000

#define MAX_KNEE          2500
#define MIN_KNEE          1500

#define MASTER_ID       0xFE
#define SERVO1_ID       0x01
#define SERVO2_ID       0x02

void arm_write_ram(uint8_t id,uint8_t reg,uint32_t data);
void arm_write_eeprom(uint8_t id,uint8_t reg,uint32_t data);

void arm_factory_reset(uint8_t id);
void arm_reboot(uint8_t id);


/*  ROBOTICS MX28-�ø��� �������� ��������*/
enum{
  MODEL_NUMBER,
  MODEL_INFORMATION,
  FIRMWARE_VERSION,
  TRANSMISSION_ID,
  BAUD_RATE,
  RETURN_DELAY,
  DRIVE_MODE,
  OPERATING_MODE,
  SECONDARY_ID,
  PROTOCOL_VERSION,
  HOMING_OFFSET,
  MOVING_THRESHOLD,
  TEMPERATURE_LIMIT,
  MAX_VOLTAGE_LIMIT,
  MIN_VOLTAGE_LIMIT,
  PWM_LIMIT,
  ACCELERATION_LIMIT,
  VELOCITY_LIMIT,
  MAX_POSITION_LIMIT,
  MIN_POSITION_LIMIT,
  SHUTDOWN,
};
/*  EEPROM ����   */
#define MODEL_NUMBER_REG          0   //2 Model Number  �� ��ȣ R 30
#define MODEL_INFORMATION_REG     2   //4 Model Information �� ���� R -
#define FIRMWARE_VERSION_REG      6   //1 Firmware Version  �߿��� ���� ���� R -
#define TRANSMISSION_ID_REG       7   //1 ID  ��� ID RW  1
#define BAUD_RATE_REG             8   //1 Baud Rate ��� �ӵ� RW  1
#define RETURN_DELAY_REG          9   //1 Return Delay Time ���� ���� �ð�  RW  250
#define DRIVE_MODE_REG            10  //1 Drive Mode  ����̺� ��� RW  0
#define OPERATING_MODE_REG        11  //1 Operating Mode  ���� ��� RW  3
#define SECONDARY_ID_REG          12  //1 Secondary(Shadow) ID  ���� ID RW  255
#define PROTOCOL_VERSION_REG      13  //1 Protocol Version  �������� ���� RW  2
#define HOMING_OFFSET_REG         20  //4 Homing Offset ��0���� ��ġ ������ RW  0
#define MOVING_THRESHOLD_REG      24  //4 Moving Threshold  ������ ������ �����ϴ� �ӵ� ���ذ� RW  10
#define TEMPERATURE_LIMIT_REG     31  //1 Temperature Limit ���� �Ѱ� �µ�  RW  80
#define MAX_VOLTAGE_LIMIT_REG     32  //2 Max Voltage Limit �ְ� �Ѱ� ����  RW  160
#define MIN_VOLTAGE_LIMIT_REG     34  //2 Min Voltage Limit ���� �Ѱ� ����  RW  95
#define PWM_LIMIT_REG             36  //2 PWM Limit PWM �Ѱ谪 RW  885
#define ACCELERATION_LIMIT_REG    40  //4 Acceleration Limit  ���ӵ� �Ѱ谪 RW  32767
#define VELOCITY_LIMIT_REG        44  //4 Velocity Limit  �ӵ� �Ѱ谪  RW  230
#define MAX_POSITION_LIMIT_REG    48  //4 Max Position Limit  �ִ� ��ġ ���Ѱ� RW  4,095
#define MIN_POSITION_LIMIT_REG    52  //4 Min Position Limit  �ּ� ��ġ ���Ѱ� RW  0
#define SHUTDOWN_REG              63  //1 Shutdown  �˴ٿ� RW  52

/*   EEPROM ���� ũ��   */
#define MODEL_NUMBER_REG_SIZE          2   //2 Model Number  �� ��ȣ R 30
#define MODEL_INFORMATION_REG_SIZE     4   //4 Model Information �� ���� R -
#define FIRMWARE_VERSION_REG_SIZE      1   //1 Firmware Version  �߿��� ���� ���� R -
#define TRANSMISSION_ID_REG_SIZE       1   //1 ID  ��� ID RW  1
#define BAUD_RATE_REG_SIZE             1   //1 Baud Rate ��� �ӵ� RW  1
#define RETURN_DELAY_REG_SIZE          1   //1 Return Delay Time ���� ���� �ð�  RW  250
#define DRIVE_MODE_REG_SIZE            1   //1 Drive Mode  ����̺� ��� RW  0
#define OPERATING_MODE_REG_SIZE        1   //1 Operating Mode  ���� ��� RW  3
#define SECONDARY_ID_REG_SIZE          1   //1 Secondary(Shadow) ID  ���� ID RW  255
#define PROTOCOL_VERSION_REG_SIZE      1   //1 Protocol Version  �������� ���� RW  2
#define HOMING_OFFSET_REG_SIZE         4   //4 Homing Offset ��0���� ��ġ ������ RW  0
#define MOVING_THRESHOLD_REG_SIZE      4   //4 Moving Threshold  ������ ������ �����ϴ� �ӵ� ���ذ� RW  10
#define TEMPERATURE_LIMIT_REG_SIZE     1   //1 Temperature Limit ���� �Ѱ� �µ�  RW  80
#define MAX_VOLTAGE_LIMIT_REG_SIZE     2   //2 Max Voltage Limit �ְ� �Ѱ� ����  RW  160
#define MIN_VOLTAGE_LIMIT_REG_SIZE     2   //2 Min Voltage Limit ���� �Ѱ� ����  RW  95
#define PWM_LIMIT_REG_SIZE             2   //2 PWM Limit PWM �Ѱ谪 RW  885
#define ACCELERATION_LIMIT_REG_SIZE    4   //4 Acceleration Limit  ���ӵ� �Ѱ谪 RW  32767
#define VELOCITY_LIMIT_REG_SIZE        4   //4 Velocity Limit  �ӵ� �Ѱ谪  RW  230
#define MAX_POSITION_LIMIT_REG_SIZE    4   //4 Max Position Limit  �ִ� ��ġ ���Ѱ� RW  4,095
#define MIN_POSITION_LIMIT_REG_SIZE    4   //4 Min Position Limit  �ּ� ��ġ ���Ѱ� RW  0
#define SHUTDOWN_REG_SIZE              1   //1 Shutdown  �˴ٿ� RW  52

enum{
  TORQUE_ENABLE,
  LED_ON_OFF,
  STATUS_RETURN_LEVEL,
  REGISTERED_INSTRUCTION,
  HARDWARE_ERROR_STATUS,
  VELOCITY_I_GAIN,
  VELOCITY_P_GAIN,
  POSITION_D_GAIN,
  POSITION_I_GAIN,
  POSITION_P_GAIN,
  FEEDFORWARD_2ND_GAIN,
  FEEDFORWARD_1ST_GAIN,
  BUS_WATCHDOG,
  GOAL_PWM,
  GOAL_VELOCITY,
  PROFILE_ACCELERATION,
  PROFILE_VELOCITY,
  GOAL_POSTION,
};
/*  RAM ����    */
#define TORQUE_ENABLE_REG             64  //1 Torque Enable ��ũ On/Off RW  0
#define LED_ON_OFF_REG                65  //1 LED LED On/Off  RW  0
#define STATUS_RETURN_LEVEL_REG       68  //1 Status Return Level ���䷹��  RW  2
#define REGISTERED_INSTRUCTION_REG    69  //1 Registered Instruction  Instruction�� ��� ����  R 0
#define HARDWARE_ERROR_STATUS_REG     70  //1 Hardware Error Status �ϵ���� ���� ����  R 0
#define VELOCITY_I_GAIN_REG           76  //2 Velocity I Gain �ӵ� I Gain RW  1920
#define VELOCITY_P_GAIN_REG           78  //2 Velocity P Gain �ӵ� P Gain RW  100
#define POSITION_D_GAIN_REG           80  //2 Position D Gain ��ġ D Gain RW  0
#define POSITION_I_GAIN_REG           82  //2 Position I Gain ��ġ I Gain RW  0
#define POSITION_P_GAIN_REG           84  //2 Position P Gain ��ġ P Gain RW  850
#define FEEDFORWARD_2ND_GAIN_REG      88  //2 Feedforward 2nd Gain  �ǵ������� 2nd Gain  RW  0
#define FEEDFORWARD_1ST_GAIN_REG      90  //2 Feedforward 1st Gain  �ǵ������� 1st Gain  RW  0
#define BUS_WATCHDOG_REG              98  //1 BUS Watchdog  ��� ���� ��ġ�� RW  0
#define GOAL_PWM_REG                  100 //2 Goal PWM  ��ǥ PWM ��  RW  -
#define GOAL_VELOCITY_REG             104 //4 Goal Velocity ��ǥ �ӵ� �� RW  -
#define PROFILE_ACCELERATION_REG      108 //4 Profile Acceleration  �������� ���ӵ� ��  RW  0
#define PROFILE_VELOCITY_REG          112 //4 Profile Velocity  �������� �ӵ� �� RW  0
#define GOAL_POSTION_REG              116 //4 Goal Position ��ǥ ��ġ �� RW  -

/*  RAM ���� ũ��   */
#define TORQUE_ENABLE_REG_SIZE             1  //1 Torque Enable ��ũ On/Off RW  0
#define LED_ON_OFF_REG_SIZE                1  //1 LED LED On/Off  RW  0
#define STATUS_RETURN_LEVEL_REG_SIZE       1  //1 Status Return Level ���䷹��  RW  2
#define REGISTERED_INSTRUCTION_REG_SIZE    1  //1 Registered Instruction  Instruction�� ��� ����  R 0
#define HARDWARE_ERROR_STATUS_REG_SIZE     1  //1 Hardware Error Status �ϵ���� ���� ����  R 0
#define VELOCITY_I_GAIN_REG_SIZE           2  //2 Velocity I Gain �ӵ� I Gain RW  1920
#define VELOCITY_P_GAIN_REG_SIZE           2  //2 Velocity P Gain �ӵ� P Gain RW  100
#define POSITION_D_GAIN_REG_SIZE           2  //2 Position D Gain ��ġ D Gain RW  0
#define POSITION_I_GAIN_REG_SIZE           2  //2 Position I Gain ��ġ I Gain RW  0
#define POSITION_P_GAIN_REG_SIZE           2  //2 Position P Gain ��ġ P Gain RW  850
#define FEEDFORWARD_2ND_GAIN_REG_SIZE      2  //2 Feedforward 2nd Gain  �ǵ������� 2nd Gain  RW  0
#define FEEDFORWARD_1ST_GAIN_REG_SIZE      2  //2 Feedforward 1st Gain  �ǵ������� 1st Gain  RW  0
#define BUS_WATCHDOG_REG_SIZE              1  //1 BUS Watchdog  ��� ���� ��ġ�� RW  0
#define GOAL_PWM_REG_SIZE                  2  //2 Goal PWM  ��ǥ PWM ��  RW  -
#define GOAL_VELOCITY_REG_SIZE             4  //4 Goal Velocity ��ǥ �ӵ� �� RW  -
#define PROFILE_ACCELERATION_REG_SIZE      4  //4 Profile Acceleration  �������� ���ӵ� ��  RW  0
#define PROFILE_VELOCITY_REG_SIZE          4  //4 Profile Velocity  �������� �ӵ� �� RW  0
#define GOAL_POSTION_REG_SIZE              4  //4 Goal Position ��ǥ ��ġ �� RW  -


#endif /* ROBOTICS_ARM_H_ */
