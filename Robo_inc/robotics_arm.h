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


/*    프로토콜 2.0 패킷 형식
Header1 Header2 Header3 Reserved  Packet_ID Length1 Length2 Instruction Param Param Param CRC1  CRC2
  0xFF   0xFF    0xFD     0x00       ID      Len_L   Len_H  Instruction Param 1 … Param N CRC_L CRC_H
*/

/*  로보티즈 프로토콜2.0 command 종류    */
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


/*  서보모터 한계 및 ID  */
#define GOAL_DEGREE     116   // 0~4095  1당 0.088도

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


/*  ROBOTICS MX28-시리즈 서보모터 레지스터*/
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
/*  EEPROM 영역   */
#define MODEL_NUMBER_REG          0   //2 Model Number  모델 번호 R 30
#define MODEL_INFORMATION_REG     2   //4 Model Information 모델 정보 R -
#define FIRMWARE_VERSION_REG      6   //1 Firmware Version  펌웨어 버전 정보 R -
#define TRANSMISSION_ID_REG       7   //1 ID  통신 ID RW  1
#define BAUD_RATE_REG             8   //1 Baud Rate 통신 속도 RW  1
#define RETURN_DELAY_REG          9   //1 Return Delay Time 응답 지연 시간  RW  250
#define DRIVE_MODE_REG            10  //1 Drive Mode  드라이브 모드 RW  0
#define OPERATING_MODE_REG        11  //1 Operating Mode  동작 모드 RW  3
#define SECONDARY_ID_REG          12  //1 Secondary(Shadow) ID  보조 ID RW  255
#define PROTOCOL_VERSION_REG      13  //1 Protocol Version  프로토콜 버전 RW  2
#define HOMING_OFFSET_REG         20  //4 Homing Offset ‘0’점 위치 조정값 RW  0
#define MOVING_THRESHOLD_REG      24  //4 Moving Threshold  움직임 유무를 결정하는 속도 기준값 RW  10
#define TEMPERATURE_LIMIT_REG     31  //1 Temperature Limit 내부 한계 온도  RW  80
#define MAX_VOLTAGE_LIMIT_REG     32  //2 Max Voltage Limit 최고 한계 전압  RW  160
#define MIN_VOLTAGE_LIMIT_REG     34  //2 Min Voltage Limit 최저 한계 전압  RW  95
#define PWM_LIMIT_REG             36  //2 PWM Limit PWM 한계값 RW  885
#define ACCELERATION_LIMIT_REG    40  //4 Acceleration Limit  가속도 한계값 RW  32767
#define VELOCITY_LIMIT_REG        44  //4 Velocity Limit  속도 한계값  RW  230
#define MAX_POSITION_LIMIT_REG    48  //4 Max Position Limit  최대 위치 제한값 RW  4,095
#define MIN_POSITION_LIMIT_REG    52  //4 Min Position Limit  최소 위치 제한값 RW  0
#define SHUTDOWN_REG              63  //1 Shutdown  셧다운 RW  52

/*   EEPROM 영역 크기   */
#define MODEL_NUMBER_REG_SIZE          2   //2 Model Number  모델 번호 R 30
#define MODEL_INFORMATION_REG_SIZE     4   //4 Model Information 모델 정보 R -
#define FIRMWARE_VERSION_REG_SIZE      1   //1 Firmware Version  펌웨어 버전 정보 R -
#define TRANSMISSION_ID_REG_SIZE       1   //1 ID  통신 ID RW  1
#define BAUD_RATE_REG_SIZE             1   //1 Baud Rate 통신 속도 RW  1
#define RETURN_DELAY_REG_SIZE          1   //1 Return Delay Time 응답 지연 시간  RW  250
#define DRIVE_MODE_REG_SIZE            1   //1 Drive Mode  드라이브 모드 RW  0
#define OPERATING_MODE_REG_SIZE        1   //1 Operating Mode  동작 모드 RW  3
#define SECONDARY_ID_REG_SIZE          1   //1 Secondary(Shadow) ID  보조 ID RW  255
#define PROTOCOL_VERSION_REG_SIZE      1   //1 Protocol Version  프로토콜 버전 RW  2
#define HOMING_OFFSET_REG_SIZE         4   //4 Homing Offset ‘0’점 위치 조정값 RW  0
#define MOVING_THRESHOLD_REG_SIZE      4   //4 Moving Threshold  움직임 유무를 결정하는 속도 기준값 RW  10
#define TEMPERATURE_LIMIT_REG_SIZE     1   //1 Temperature Limit 내부 한계 온도  RW  80
#define MAX_VOLTAGE_LIMIT_REG_SIZE     2   //2 Max Voltage Limit 최고 한계 전압  RW  160
#define MIN_VOLTAGE_LIMIT_REG_SIZE     2   //2 Min Voltage Limit 최저 한계 전압  RW  95
#define PWM_LIMIT_REG_SIZE             2   //2 PWM Limit PWM 한계값 RW  885
#define ACCELERATION_LIMIT_REG_SIZE    4   //4 Acceleration Limit  가속도 한계값 RW  32767
#define VELOCITY_LIMIT_REG_SIZE        4   //4 Velocity Limit  속도 한계값  RW  230
#define MAX_POSITION_LIMIT_REG_SIZE    4   //4 Max Position Limit  최대 위치 제한값 RW  4,095
#define MIN_POSITION_LIMIT_REG_SIZE    4   //4 Min Position Limit  최소 위치 제한값 RW  0
#define SHUTDOWN_REG_SIZE              1   //1 Shutdown  셧다운 RW  52

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
/*  RAM 영역    */
#define TORQUE_ENABLE_REG             64  //1 Torque Enable 토크 On/Off RW  0
#define LED_ON_OFF_REG                65  //1 LED LED On/Off  RW  0
#define STATUS_RETURN_LEVEL_REG       68  //1 Status Return Level 응답레벨  RW  2
#define REGISTERED_INSTRUCTION_REG    69  //1 Registered Instruction  Instruction의 등록 여부  R 0
#define HARDWARE_ERROR_STATUS_REG     70  //1 Hardware Error Status 하드웨어 에러 상태  R 0
#define VELOCITY_I_GAIN_REG           76  //2 Velocity I Gain 속도 I Gain RW  1920
#define VELOCITY_P_GAIN_REG           78  //2 Velocity P Gain 속도 P Gain RW  100
#define POSITION_D_GAIN_REG           80  //2 Position D Gain 위치 D Gain RW  0
#define POSITION_I_GAIN_REG           82  //2 Position I Gain 위치 I Gain RW  0
#define POSITION_P_GAIN_REG           84  //2 Position P Gain 위치 P Gain RW  850
#define FEEDFORWARD_2ND_GAIN_REG      88  //2 Feedforward 2nd Gain  피드포워드 2nd Gain  RW  0
#define FEEDFORWARD_1ST_GAIN_REG      90  //2 Feedforward 1st Gain  피드포워드 1st Gain  RW  0
#define BUS_WATCHDOG_REG              98  //1 BUS Watchdog  통신 버스 워치독 RW  0
#define GOAL_PWM_REG                  100 //2 Goal PWM  목표 PWM 값  RW  -
#define GOAL_VELOCITY_REG             104 //4 Goal Velocity 목표 속도 값 RW  -
#define PROFILE_ACCELERATION_REG      108 //4 Profile Acceleration  프로파일 가속도 값  RW  0
#define PROFILE_VELOCITY_REG          112 //4 Profile Velocity  프로파일 속도 값 RW  0
#define GOAL_POSTION_REG              116 //4 Goal Position 목표 위치 값 RW  -

/*  RAM 영역 크기   */
#define TORQUE_ENABLE_REG_SIZE             1  //1 Torque Enable 토크 On/Off RW  0
#define LED_ON_OFF_REG_SIZE                1  //1 LED LED On/Off  RW  0
#define STATUS_RETURN_LEVEL_REG_SIZE       1  //1 Status Return Level 응답레벨  RW  2
#define REGISTERED_INSTRUCTION_REG_SIZE    1  //1 Registered Instruction  Instruction의 등록 여부  R 0
#define HARDWARE_ERROR_STATUS_REG_SIZE     1  //1 Hardware Error Status 하드웨어 에러 상태  R 0
#define VELOCITY_I_GAIN_REG_SIZE           2  //2 Velocity I Gain 속도 I Gain RW  1920
#define VELOCITY_P_GAIN_REG_SIZE           2  //2 Velocity P Gain 속도 P Gain RW  100
#define POSITION_D_GAIN_REG_SIZE           2  //2 Position D Gain 위치 D Gain RW  0
#define POSITION_I_GAIN_REG_SIZE           2  //2 Position I Gain 위치 I Gain RW  0
#define POSITION_P_GAIN_REG_SIZE           2  //2 Position P Gain 위치 P Gain RW  850
#define FEEDFORWARD_2ND_GAIN_REG_SIZE      2  //2 Feedforward 2nd Gain  피드포워드 2nd Gain  RW  0
#define FEEDFORWARD_1ST_GAIN_REG_SIZE      2  //2 Feedforward 1st Gain  피드포워드 1st Gain  RW  0
#define BUS_WATCHDOG_REG_SIZE              1  //1 BUS Watchdog  통신 버스 워치독 RW  0
#define GOAL_PWM_REG_SIZE                  2  //2 Goal PWM  목표 PWM 값  RW  -
#define GOAL_VELOCITY_REG_SIZE             4  //4 Goal Velocity 목표 속도 값 RW  -
#define PROFILE_ACCELERATION_REG_SIZE      4  //4 Profile Acceleration  프로파일 가속도 값  RW  0
#define PROFILE_VELOCITY_REG_SIZE          4  //4 Profile Velocity  프로파일 속도 값 RW  0
#define GOAL_POSTION_REG_SIZE              4  //4 Goal Position 목표 위치 값 RW  -


#endif /* ROBOTICS_ARM_H_ */
