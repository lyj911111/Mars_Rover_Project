#include "Robot_kjy.h"

/**********소스 내부 함수 *************/
void Mix_Pwm(uint32_t* Pwm_speed);
void Push_speed(uint32_t* Pwm_speed , uint32_t* Direction);
void Push_break(uint32_t* Pwm_break);
void BUGI_Mode1();
void BUGI_Mode2();
void BUGI_Mode3();
void ARM_Mode();



/**************전역변수*************/
uint32_t pwm_speed[6]={0},direction[6]={0},pwm_break[6]={0};
extern UART_HandleTypeDef huart2;

extern TIM_HandleTypeDef htim8;

#define degree_term 2
#define time_interval 100
void ARM_Mode()
{
  uint32_t grab_signal = RC_Read(CH1),wrist_signal = RC_Read(CH2);
  uint32_t elbow_signal =  RC_Read(CH3),shoulder_signal = RC_Read(CH4);
  static uint16_t grab,wrist,elbow,shoulder;
  static uint16_t previous_grab,previous_wrist,previous_knee,previous_shoulder;
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
    if(elbow_signal > RC_MID1)
      elbow += ELBOW_POSTION;
    else if(elbow_signal < RC_MID2)
      elbow -= ELBOW_POSTION;
    if(shoulder_signal > RC_MID1)
      shoulder += SHOULDER_POSTION;
    else if(shoulder_signal < RC_MID2)
      shoulder -= SHOULDER_POSTION;

    constrain(grab,MIN_PWM_ARM,MAX_PWM_ARM);
    constrain(wrist,MIN_PWM_ARM,MAX_PWM_ARM);
    constrain(elbow,MIN_ELBOW,MAX_ELBOW);
    constrain(shoulder,MIN_SHOULDER,MAX_SHOULDER);

    if(previous_grab != grab){
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, grab);
      previous_grab = grab;
    }
    if(previous_wrist != wrist){
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, wrist);
      previous_wrist = wrist;
    }
    if(previous_knee != elbow){
      arm_write_ram(ELBOW_ID, GOAL_POSTION, elbow);
      previous_knee = elbow;
    }
    if(previous_shoulder != shoulder){
      arm_write_ram(SHOULDER_ID, GOAL_POSTION, shoulder);
      previous_shoulder = shoulder;
    }
    time_previous = time_current;
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































