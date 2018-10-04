#include "Robot_kjy.h"

/**********소스 내부 함수 *************/
void Mix_Pwm(uint32_t* Pwm_speed);
void Push_speed(uint32_t* Pwm_speed , uint32_t* Direction);
void Push_break(uint32_t* Pwm_break);
void BUGI_Mode1();
void BUGI_Mode2();
void BUGI_Mode3();


/**************전역변수*************/
uint32_t pwm_speed[6]={0},direction[6]={0},pwm_break[6]={0};

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
      Wheel_AllbreakX(0);
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
    }
}
uint32_t Mode_select()
{
  uint32_t emergency=RC_Read(EMERGENCY),gear=RC_Read(CHANGE_GEAR);

  if(RC_MID2 < emergency && emergency < RC_MID1)
    return 0;

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
  static uint32_t dir=0;
  if(just_one_mode1){
    for(int i=0;i<6;i++)
      direction[i]=WHEEL_FORWARD;
    just_one_mode1=0;
    just_one_mode2=1;
    just_one_mode3=1;
    Wheel_AllbreakX(0);
    Wheel_AllSpeedX(0,WHEEL_FORWARD);
  }
  if(line > RC_MID1){
    constrain(line,RC_MID1,RC_MAX);
    constrain(limit,RC_MIN,RC_MAX);
    line  = math_Map(line, RC_MID1, RC_MAX, PULSE_MIN, PULSE_MAX);
    limit = math_Map(limit, RC_MIN, RC_MAX, PULSE_MIN, PULSE_MAX);
    constrain(line,PULSE_MIN,limit);

    if(side > RC_MID1){
      constrain(side,RC_MID1,RC_MAX);
      side  = math_Map(side, RC_MID1, RC_MAX, PULSE_MIN, PULSE_MAX);
      side  = math_Map(side, PULSE_MIN, PULSE_MAX, PULSE_MIN, line);
      dir = WHEEL_RIGHT;
    }
    else if(side > RC_MID2){
      side = 0;
    }
    else{
      constrain(side,RC_MIN,RC_MID2);
      side = RC_MID2 - side + RC_MIN;
      side  = math_Map(side, RC_MIN, RC_MID2, PULSE_MIN, PULSE_MAX);
      side  = math_Map(side, PULSE_MIN, PULSE_MAX, PULSE_MIN, line);
      dir = WHEEL_LEFT;
    }

    for(int i=0;i<6;i++){
      if(i%2==dir){
        pwm_speed[i]=line;
        pwm_break[i] = 0;
        constrain(pwm_speed[i],0,limit);
      }
      else{
        //pwm_speed[i]=line-side;
        pwm_speed[i]=0;
        pwm_break[i] = side;
        constrain(pwm_speed[i],0,limit);
      }

    }
    Push_speed(pwm_speed,direction);
    Push_break(pwm_break);
  }
  else if(line > RC_MID2){
    Wheel_AllSpeedX(0,WHEEL_FORWARD );
    Wheel_AllbreakX(0);
  }
  else {
    constrain(line,RC_MIN,RC_MID2);
    line = math_Map(line, RC_MIN, RC_MID2, PULSE_MIN, PULSE_MAX);
    line = PULSE_MAX - line + PULSE_MIN;
    Wheel_AllbreakX(line);
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
  uint32_t dir=0;
  if(just_one_mode3){
    for(int i=0;i<6;i++)
      direction[i]=WHEEL_BACKWARD;
    just_one_mode1=1;
    just_one_mode2=1;
    just_one_mode3=0;
    Wheel_AllbreakX(0);
  }
  constrain(line,RC_MIN,RC_MAX);
  constrain(side,RC_MIN,RC_MAX);

  if(line > RC_MID1){
    line  = math_Map(line, RC_MID1, RC_MAX, PULSE_MIN, PULSE_MAX);
    limit = math_Map(limit, RC_MIN, RC_MAX, PULSE_MIN, PULSE_MAX);
    constrain(line,PULSE_MIN,limit);

    if(side > RC_MID1){
      side  = math_Map(side, RC_MID1, RC_MAX, PULSE_MIN, PULSE_MAX);
      side  = math_Map(side, PULSE_MIN, PULSE_MAX, PULSE_MIN, line);
      dir = WHEEL_RIGHT;
    }
    else if(side > RC_MID2)
      side = 0;
    else{
      side = RC_MID2 - side + RC_MIN;
      side  = math_Map(side, RC_MIN, RC_MID2, PULSE_MIN, PULSE_MAX);
      side  = math_Map(side, PULSE_MIN, PULSE_MAX, PULSE_MIN, line);
      dir = WHEEL_LEFT;
    }

    for(int i=0;i<6;i++){
      if(i%2==dir){
        pwm_speed[i]=line;
        constrain(pwm_speed[i],0,limit);
      }
      else{
        pwm_speed[i]=line-side;
        constrain(pwm_speed[i],0,limit);
      }
    }
    Push_speed(pwm_speed,direction);
  }
  else if(line > RC_MID2){
    Wheel_AllSpeedX(0,0);
    Wheel_AllbreakX(0);
  }
  else {
    line = math_Map(line, RC_MIN, RC_MID2, PULSE_MIN, PULSE_MAX);
    line = PULSE_MAX - line + PULSE_MIN;
    Wheel_AllbreakX(line);
  }
}

























