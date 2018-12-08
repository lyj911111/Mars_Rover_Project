#include "Robot_kjy.h"

/**********소스 내부 함수 *************/
void Mix_Pwm(uint32_t* Pwm_speed);
void Push_speed(uint32_t* Pwm_speed , uint32_t* Direction);
void Push_break(uint32_t* Pwm_break);
void BUGI_Mode1();
void BUGI_Mode2();
void BUGI_Mode3();
void ARM_Mode();
void CAMERA_Mode();

/**************전역변수*************/
uint32_t pwm_speed[6]={0},direction[6]={0},pwm_break[6]={0};

extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;

#define degree_term 2
#define time_interval 100

void CAMERA_Mode()
{
  uint16_t camera_signal = RC_Read(CH4);
  static uint16_t camera = INIT_CAMERA;
  static uint32_t time_previous;
  uint32_t time_current = HAL_GetTick();

    if(time_current - time_previous > time_interval*2){
      if(camera_signal > RC_MID1)
        camera += 1;
      else if(camera_signal < RC_MID2)
        camera -= 1;
      constrain(camera,MIN_CAMERA,MAX_CAMERA);
      __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, camera);
      time_previous = time_current;
    }
}

void ARM_init()
{
  HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start_IT(&htim9, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, INIT_GRAB);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, INIT_WRIST);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, INIT_CAMERA);

  ARM_enable(SERVO1_ID);
  ARM_enable(SERVO2_ID);
  arm_write_ram(ELBOW_ID, GOAL_VELOCITY, MID_ELBOW_SPEED);
  HAL_Delay(10);
  arm_write_ram(SHOULDER_ID, GOAL_VELOCITY, MID_SHOULDER_SPEED);
}
/*
 * PC6 손        8/CH1
 * PC7 손목      8/CH2
 * PA3 카메라    9/CH2
 * PD5 어깨,팔꿈치
 * */
#define RC_SUB  600
#define AC_SUB  200
void ARM_Mode()
{
  uint32_t grab_signal = RC_Read(CH1),wrist_signal = RC_Read(CH2);
  uint32_t elbow_signal =  RC_Read(CH3),shoulder_signal = RC_Read(CH4);
  static uint16_t grab=INIT_GRAB,wrist=INIT_WRIST,elbow=2,shoulder=2;
  static uint16_t previous_grab,previous_wrist,previous_knee,previous_shoulder;
  static uint32_t time_previous;
  uint32_t time_current = HAL_GetTick();


  if(time_current - time_previous > time_interval){
    if(grab_signal > RC_MID1+RC_SUB)
      grab += GRAB_MOVE;
    else if(grab_signal <RC_MID2-RC_SUB)
      grab -= GRAB_MOVE;
    if(wrist_signal > RC_MID1+RC_SUB)
      wrist += WRIST_MOVE;
    else if(wrist_signal < RC_MID2-RC_SUB)
      wrist -= WRIST_MOVE;

    if(elbow_signal > RC_MID1+AC_SUB)
      elbow = 0;
    else if(elbow_signal < RC_MID2-AC_SUB)
      elbow = 1;
    else
      elbow = 2;

    if(shoulder_signal > RC_MID1+AC_SUB)
      shoulder = 0;
    else if(shoulder_signal < RC_MID2-AC_SUB)
      shoulder = 1;
    else
      shoulder = 2;

    constrain(grab,MIN_GRAB,MAX_GRAB);
    constrain(wrist,MIN_WRIST,MAX_WRIST);

    if(previous_grab != grab){
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, grab);
      previous_grab = grab;
    }
    if(previous_wrist != wrist){
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, wrist);
      previous_wrist = wrist;
    }
    if(previous_knee != elbow){
      if(elbow == 0)
        arm_write_ram(ELBOW_ID, GOAL_VELOCITY, MAX_ELBOW_SPEED);
      else if(elbow ==1)
        arm_write_ram(ELBOW_ID, GOAL_VELOCITY, MIN_ELBOW_SPEED);
      else if(elbow ==2)
        arm_write_ram(ELBOW_ID, GOAL_VELOCITY, MID_ELBOW_SPEED);
      previous_knee = elbow;
      HAL_Delay(4);
    }
    if(previous_shoulder != shoulder){
      if(shoulder == 0)
        arm_write_ram(SHOULDER_ID, GOAL_VELOCITY, MAX_SHOULDER_SPEED);
      else if(shoulder ==1)
        arm_write_ram(SHOULDER_ID, GOAL_VELOCITY, MIN_SHOULDER_SPEED);
      else if(shoulder ==2)
        arm_write_ram(SHOULDER_ID, GOAL_VELOCITY, MID_SHOULDER_SPEED);
      previous_shoulder = shoulder;
      HAL_Delay(4);
    }
    time_previous = time_current;
  }
}
void ARM_enable(uint8_t ID)
{
  arm_write_ram(ID,TORQUE_ENABLE,1);
  HAL_Delay(10);
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
      arm_write_ram(SHOULDER_ID, GOAL_VELOCITY, MID_SHOULDER_SPEED);
      HAL_Delay(5);
      arm_write_ram(ELBOW_ID, GOAL_VELOCITY, MID_ELBOW_SPEED);
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
      ARM_Mode();
      break;
    case 5:
      CAMERA_Mode();
    }
}
uint32_t Mode_select()
{
  uint32_t emergency=RC_Read(EMERGENCY),gear=RC_Read(CHANGE_GEAR);

  if(RC_MID2 < emergency && emergency < RC_MID1)
    return 0;
  else if(emergency < RC_MID2){         //  작아지면 ARM 모드 변환
    if(gear > RC_MID1 || gear < RC_MID2)
      return 4;
    else
      return 5;
  }

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

  if(just_one_mode2){
    just_one_mode1=1;
    just_one_mode2=0;
    just_one_mode3=1;
    Wheel_AllbreakX(0);
    Wheel_AllSpeedX(0, 0);
  }

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
  if(just_one_mode3){
    wheel_dir = WHEEL_BACKWARD;
    for(int i=0;i<6;i++)
      direction[i]=wheel_dir;
    just_one_mode1=1;
    just_one_mode2=1;
    just_one_mode3=0;
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































