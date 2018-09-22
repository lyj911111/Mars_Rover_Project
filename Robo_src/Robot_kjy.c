#include "Robot_kjy.h"


/*
 * Ÿ�̸� ����ü�� �ּҿ� ���� RC��Ʈ�� ���¸� �Է¹޾� Ÿ�̸Ӹ� Ű�ų� ���� �Լ�
 * RC���� ���� ���������� ���������� ����
 *
 * ex)
 * Move_motor(&htim10,500);
 *
 *
 *
 *
 * */
void ARM_Move_motor(TIM_HandleTypeDef *htim,uint32_t RC_instance)
{
    if(RC_instance > STEP_FORWARD_RC){
        HAL_GPIO_WritePin(STEP_DIR_PORT,STEP_DIR_PINNUM,STEP_FORWARD);
        HAL_TIM_Base_Start_IT(htim);
    }
    else if(RC_instance > STEP_BACKWARD_RC){
        HAL_TIM_Base_Stop_IT(htim);
    }
    else{
        HAL_GPIO_WritePin(STEP_DIR_PORT,STEP_DIR_PINNUM,STEP_BACKWARD);
        HAL_TIM_Base_Start_IT(htim);
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

