#include "Robot_kjy.h"

/**********소스 내부 함수 *************/
void Mix_Pwm(uint32_t* Pwm_speed);
void Push_speed(uint32_t* Pwm_speed , uint32_t* Direction);
void Push_break(uint32_t* Pwm_break);
void BUGI_Mode1();
void BUGI_Mode2();
void BUGI_Mode3();
void ARM_Mode();
void command_arm_robotics(uint8_t reg,uint8_t id,uint32_t data);
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);

/**************전역변수*************/
uint32_t pwm_speed[6]={0},direction[6]={0},pwm_break[6]={0};
extern UART_HandleTypeDef huart2;

/*
 * 타이머 구조체의 주소와 현재 RC컨트롤 상태를 입력받아 타이머를 키거나 끄는 함수
 * RC값에 따라서 역방향인지 정방향인지 정함
 * ex)
 * Move_motor(&htim10,2300);
 * */
void ARM_Move_motor(TIM_HandleTypeDef *htim,uint32_t RC_instance)
{
    if(RC_instance > STEP_FORWARD_RC){
        HAL_GPIO_WritePin(STEP_DIR_PORT,STEP_DIR_PINNUM,STEP_FORWARD);
        HAL_TIM_Base_Start_IT(htim);
    }
    else if(RC_instance < STEP_BACKWARD_RC){
        HAL_GPIO_WritePin(STEP_DIR_PORT,STEP_DIR_PINNUM,STEP_BACKWARD);
        HAL_TIM_Base_Start_IT(htim);
    }
    else{
        HAL_TIM_Base_Stop_IT(htim);
    }
}
void ARM_Generation_pulse(TIM_HandleTypeDef* htim)
{
    static uint32_t count=0;
    static uint32_t toggle=0;
    if(htim->Instance == TIM10){
        count++;
        toggle = !toggle;
        HAL_GPIO_WritePin(STEP_PUL_PORT,STEP_PUL_PINNUM,toggle);
    }
}
/*********************************************************************/
/*
 * 공용함수
 * */
uint8_t just_one_mode1=1;
uint8_t just_one_mode2=1;
uint8_t just_one_mode3=1;
uint8_t str[50];

void BUGI_DriveMode(uint32_t mode)
{
    switch(mode)
    {
    case 0:
      Wheel_AllbreakX(PULSE_MAX);
      just_one_mode1=1;
      just_one_mode2=1;
      just_one_mode3=1;
      break;
    case 1:
      BUGI_Mode1();
      break;
    case 2:
      BUGI_Mode2();
      break;
    case 3:
      BUGI_Mode3();
      break;
    case 4:
      servo_enable(SERVO1_ID,1);
      ARM_Mode();
      break;
    }
}
uint32_t Mode_select()
{
  uint32_t emergency=RC_Read(EMERGENCY),gear=RC_Read(CHANGE_GEAR);

  if(RC_MID2 < emergency && emergency < RC_MID1)
    return 0;
  else if(emergency < RC_MID2)    //  작아지면 ARM 모드 변환
    return 4;

  if(gear>RC_MID1)
    return 1;
  else if(gear >RC_MID2)
    return 2;
  else
    return 3;
}
//********************************************************//
/*
 * 소스 내부 함수
 * */
void Mix_Pwm(uint32_t* Pwm_speed)
{
  Pwm_speed[WHEEL_R_U] += GAIN_R_U;
  Pwm_speed[WHEEL_L_U] += GAIN_L_U;
  Pwm_speed[WHEEL_R_M] += GAIN_R_M;
  Pwm_speed[WHEEL_L_M] += GAIN_L_M;
  Pwm_speed[WHEEL_R_D] += GAIN_R_D;
  Pwm_speed[WHEEL_L_D] += GAIN_L_D;
}

void Push_speed(uint32_t* Pwm_speed , uint32_t* Direction)
{
  Mix_Pwm(Pwm_speed);
  for(int i=0; i<6;i++){
    Wheel_Contorl(i, Direction[i], Pwm_speed[i]);
  }
}

void Push_break(uint32_t* Pwm_break)
{
  for(int i=0 ;i<6 ;i++){
    Wheel_Break(i,Pwm_break[i]);
  }
}

void BUGI_Mode1()
{
  uint32_t line=RC_Read(MOVE_LINE),side=RC_Read(MOVE_SIDE),limit=RC_Read(LIMIT_SPEED);
  static uint32_t dir=0,wheel_dir=WHEEL_FORWARD;
  if(just_one_mode1){
    wheel_dir = WHEEL_FORWARD;
    for(int i=0;i<6;i++)
      direction[i]=wheel_dir;
    just_one_mode1=0;
    just_one_mode2=1;
    just_one_mode3=1;
    Wheel_AllbreakX(0);
    Wheel_AllSpeedX(0,wheel_dir);
  }
  if(line > RC_MID1){
    constrain(line,RC_MID1,RC_MAX);
    constrain(limit,RC_MIN,RC_MAX);
    limit = RC_MAX - limit + RC_MIN;
    line  = math_Map(line, RC_MID1, RC_MAX, PULSE_MIN, PULSE_MAX);
    limit = math_Map(limit, RC_MIN, RC_MAX, PULSE_MIN, PULSE_MAX);
    constrain(line,PULSE_MIN,limit);

    if(side > RC_MID1){
      /*constrain(side,RC_MID1,RC_MAX);
      side  = math_Map(side, RC_MID1, RC_MAX, PULSE_MIN, PULSE_MAX);
      side  = math_Map(side, PULSE_MIN, PULSE_MAX, PULSE_MIN, line);*/
      side = line;
      if(wheel_dir==WHEEL_FORWARD)
        dir = WHEEL_RIGHT;
      else
        dir = WHEEL_LEFT;
    }
    else if(side > RC_MID2){
      side = 0;
    }
    else{
      /*constrain(side,RC_MIN,RC_MID2);
      side = RC_MID2 - side + RC_MIN;
      side  = math_Map(side, RC_MIN, RC_MID2, PULSE_MIN, PULSE_MAX);
      side  = math_Map(side, PULSE_MIN, PULSE_MAX, PULSE_MIN, line);*/
      side = line;
      if(wheel_dir==WHEEL_BACKWARD)
        dir = WHEEL_LEFT;
      else
        dir = WHEEL_RIGHT;
    }

    for(int i=0;i<6;i++){
      if(i%2==dir){
        pwm_speed[i] = line;
        pwm_break[i] = 0;
        constrain(pwm_speed[i],0,limit);
      }
      else{
        pwm_speed[i] = line-side;
        pwm_break[i] = (side!=0)*PULSE_MAX;
        constrain(pwm_speed[i],0,limit);
      }
    }
    Push_speed(pwm_speed,direction);
    Push_break(pwm_break);
  }
  else if(line > RC_MID2){
    Wheel_AllSpeedX(0,wheel_dir );
    Wheel_AllbreakX(0);
  }
  else {
    constrain(line,RC_MIN,RC_MID2);
    line = math_Map(line, RC_MIN, RC_MID2, PULSE_MIN, PULSE_MAX);
    line = PULSE_MAX - line + PULSE_MIN;
    Wheel_AllbreakX(line);
    Wheel_AllSpeedX(0,wheel_dir);
  }
}

void BUGI_Mode2()
{
  int32_t side=RC_Read(MOVE_SIDE);
  static int32_t dir=0;
  constrain(side,RC_MIN,RC_MAX);

  if(side > RC_MID1){
    dir = WHEEL_LEFT;
    side = math_Map(side, RC_MID1, RC_MAX, PULSE_MIN, PULSE_MAX/2);
    Wheel_AllbreakX(PULSE_MIN);
  }
  else if(side > RC_MID2){
    side =0;
    Wheel_AllbreakX(PULSE_MAX/2);
  }
  else if(side < RC_MID2){
    dir = WHEEL_RIGHT;
    side = RC_MID2 - side +RC_MIN;
    side = math_Map(side, RC_MIN, RC_MID2, PULSE_MIN, PULSE_MAX/2);
    Wheel_AllbreakX(PULSE_MIN);
  }

  if(just_one_mode2){
    just_one_mode1=1;
    just_one_mode2=0;
    just_one_mode3=1;
    Wheel_AllbreakX(0);
    Wheel_AllSpeedX(0, 0);
  }

  for(int i=0 ; i<6; i++){
    if(i%2==0){
      direction[i]=dir;
      pwm_speed[i]=side;
    }
    else{
      direction[i]=!dir;
      pwm_speed[i]=side;
    }
  }
  Push_speed(pwm_speed,direction);
}

void BUGI_Mode3()
{
  uint32_t line=RC_Read(MOVE_LINE),side=RC_Read(MOVE_SIDE),limit=RC_Read(LIMIT_SPEED);
  static uint32_t dir=0,wheel_dir=WHEEL_BACKWARD;
  if(just_one_mode1){
    wheel_dir = WHEEL_BACKWARD;
    for(int i=0;i<6;i++)
      direction[i]=wheel_dir;
    just_one_mode1=0;
    just_one_mode2=1;
    just_one_mode3=1;
    Wheel_AllbreakX(0);
    Wheel_AllSpeedX(0,wheel_dir);
  }
  if(line > RC_MID1){
    constrain(line,RC_MID1,RC_MAX);
    constrain(limit,RC_MIN,RC_MAX);
    limit = RC_MAX - limit + RC_MIN;
    line  = math_Map(line, RC_MID1, RC_MAX, PULSE_MIN, PULSE_MAX);
    limit = math_Map(limit, RC_MIN, RC_MAX, PULSE_MIN, PULSE_MAX);
    constrain(line,PULSE_MIN,limit);

    if(side > RC_MID1){
      /*constrain(side,RC_MID1,RC_MAX);
      side  = math_Map(side, RC_MID1, RC_MAX, PULSE_MIN, PULSE_MAX);
      side  = math_Map(side, PULSE_MIN, PULSE_MAX, PULSE_MIN, line);*/
      side = line;
      if(wheel_dir==WHEEL_BACKWARD)
        dir = WHEEL_RIGHT;
      else
        dir = WHEEL_LEFT;
    }
    else if(side > RC_MID2){
      side = 0;
    }
    else{
      /*constrain(side,RC_MIN,RC_MID2);
      side = RC_MID2 - side + RC_MIN;
      side  = math_Map(side, RC_MIN, RC_MID2, PULSE_MIN, PULSE_MAX);
      side  = math_Map(side, PULSE_MIN, PULSE_MAX, PULSE_MIN, line);*/
      side = line;
      if(wheel_dir==WHEEL_FORWARD)
        dir = WHEEL_LEFT;
      else
        dir = WHEEL_RIGHT;
    }

    for(int i=0;i<6;i++){
      if(i%2==dir){
        pwm_speed[i] = line;
        pwm_break[i] = 0;
        constrain(pwm_speed[i],0,limit);
      }
      else{
        pwm_speed[i] = line-side;
        pwm_break[i] = (side!=0)*PULSE_MAX;
        constrain(pwm_speed[i],0,limit);
      }
    }
    Push_speed(pwm_speed,direction);
    Push_break(pwm_break);
  }
  else if(line > RC_MID2){
    Wheel_AllSpeedX(0,wheel_dir );
    Wheel_AllbreakX(0);
  }
  else {
    constrain(line,RC_MIN,RC_MID2);
    line = math_Map(line, RC_MIN, RC_MID2, PULSE_MIN, PULSE_MAX);
    line = PULSE_MAX - line + PULSE_MIN;
    Wheel_AllbreakX(line);
    Wheel_AllSpeedX(0,wheel_dir);
  }
}
extern TIM_HandleTypeDef htim8;

#define degree_term 2
#define time_interval 100
void ARM_Mode()
{
  uint32_t grab_signal = RC_Read(CH1),wrist_signal = RC_Read(CH2);
  uint32_t knee_signal =  RC_Read(CH3),shoulder_signal = RC_Read(CH4);
  static uint16_t grab,wrist,knee,shoulder;
  static uint32_t time_previous;
  uint32_t time_current = HAL_GetTick();


  if(time_current - time_previous > time_interval){
    if(grab_signal > RC_MID1)
      grab += degree_term;
    else if(grab_signal <RC_MID2)
      grab -= degree_term;
    if(wrist_signal > RC_MID1)
      wrist += degree_term;
    else if(wrist_signal < RC_MID2)
      wrist -= degree_term;
    time_previous = time_current;
    if(knee_signal > RC_MID1)
      knee += 50;
    else if(knee_signal < RC_MID2)
      knee -= 50;
    command_arm_robotics(GOAL_DEGREE , SERVO1_ID, knee);
  }

  constrain(grab,MIN_PWM_ARM,MAX_PWM_ARM);
  constrain(wrist,MIN_PWM_ARM,MAX_PWM_ARM);
  constrain(knee,SERVO_MIN_ROBOTICS,SERVO_MAX_ROBOTICS);
  constrain(shoulder,SERVO_MIN_ROBOTICS,SERVO_MAX_ROBOTICS);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, grab);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, wrist);

}

/*
Header1 Header2 Header3 Reserved  Packet ID Length1 Length2 Instruction Param Param Param CRC1  CRC2
0xFF  0xFF  0xFD  0x00  ID  Len_L Len_H Instruction Param 1 … Param N CRC_L CRC_H
*/
enum{
  HEAD1,HEAD2,HEAD3,RESERVED,PACKET_ID,LENGTH_LOW,LENGTH_HIGH,COMMAND,REG_LOW,REG_HIGH,
  PARA1,PARA2,PARA3,PARA4,CRC1,CRC2
};
void command_arm_robotics(uint8_t reg,uint8_t id,uint32_t data)
{
  uint8_t str[20]={0xFF,0xFF,0xFD,0x00};
  uint8_t str_para_num=0;
  uint8_t str_size=0;
  uint16_t crc;
  str[PACKET_ID] = id;
  if(reg == 116){
    str[LENGTH_LOW] = 9;
    str_para_num = 4;
  }

  str[LENGTH_HIGH] = 0;
  str[COMMAND]   = WRITE;
  str[REG_LOW]   = reg;
  str[REG_HIGH]  = 0;
  for(int i=0 ; i<str_para_num ; i++)
    str[PARA1+i] = (uint8_t)((data>>(str_para_num-i)) & 0x000000FF);

  str_size = CRC2+1;
  crc = update_crc(0, str, str_size-2);
  str[CRC1] = crc&0x00FF;
  str[CRC2] = (crc>>8)&0x00FF;

  HAL_UART_Transmit(&huart2, str, str_size,10);
}

void servo_enable(uint8_t id,uint8_t en_ok)
{
  uint8_t str[20]={0xFF,0xFF,0xFD,0x00};
  uint8_t str_size=8;
  uint16_t crc;
  str[PACKET_ID] = id;
  str[LENGTH_LOW] = 9;
  str[LENGTH_HIGH] = 0;
  str[COMMAND]  = WRITE;
  str[REG_LOW]  = TORQUE_EN;
  str[REG_HIGH] = 0;
  str[PARA1]    = 0;
  str[PARA2]    = 0;
  str[PARA3]    = 0;
  if(en_ok ==0)
    str[PARA4]    = 0;
  else
    str[PARA4]    = 1;
  str_size = CRC2+1;
  crc = update_crc(0, str, str_size-2);
  str[CRC1] = crc&0x00FF;
  str[CRC2] = (crc>>8)&0x00FF;

  HAL_UART_Transmit(&huart2, str, str_size,10);
}



unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}





















