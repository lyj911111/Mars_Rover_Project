#include "Robot_kjy.h"

/**********�ҽ� ���� �Լ� *************/
void Mix_Pwm(uint32_t* pwm);
void BUGI_Mode1();


/**************��������*************/
uint32_t pwm[6]={0};

/*
 * Ÿ�̸� ����ü�� �ּҿ� ���� RC��Ʈ�� ���¸� �Է¹޾� Ÿ�̸Ӹ� Ű�ų� ���� �Լ�
 * RC���� ���� ���������� ���������� ����
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
 * ��忡 ���� �����ϴ� ����� �޶�����.
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


























