#include "Robot_kjy.h"

/**********소스 내부 함수 *************/
void Mix_Pwm(uint32_t* pwm);
void BUGI_Mode1();


/**************전역변수*************/
uint32_t pwm[6]={0};

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

/*
 * 모드에 따라서 조종하는 방법이 달라진다.
 * */
void BUGI_DriveMode(uint32_t mode)
{
    switch(mode)
    {
    case 1:
        break;
    case 2:
        break;
    case 3:
        break;
    }
}

void Mix_Pwm(uint32_t* Pwm)
{
    Pwm[WHEEL_R_U] += GAIN_R_U;
    Pwm[WHEEL_L_U] += GAIN_L_U;
    Pwm[WHEEL_R_M] += GAIN_R_M;
    Pwm[WHEEL_L_M] += GAIN_L_M;
    Pwm[WHEEL_R_D] += GAIN_R_D;
    Pwm[WHEEL_L_D] += GAIN_L_D;
}
void BUGI_Mode1()
{
    uint32_t rc[MAX_CHANNEL_NUM+1]={0};
    for(int i=1 ; i<=MAX_CHANNEL_NUM ; i++){
        rc[i]=RC_Read(i);
    }
    if(!RC_CheckConnect())
        Wheel_Allbreak();
    else{

    }

}


























