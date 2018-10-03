/*
 * Robot_jhm.c
 *
 *  Created on: 2018. 9. 22.
 *      Author: jhm
 */


#include "Robot_jhm.h"
volatile uint32_t rc_dutycycle[7];
volatile uint16_t ch_rising[7], ch_falling[7];
static GPIO_InitTypeDef my_GPIO_InitStruct[7];
GPIO_TypeDef* gpio[7]={GPIOE,GPIOE,GPIOE,       // 1 2 3
                       GPIOE,GPIOF,GPIOF,       // 4 5 6
                       GPIOF};                  // 7

uint32_t check;
/*			MAX				MIDDLE			MIN
 * CH1  	3865~3871		3032~3077		2190~2196
 * CH2		3854~3859		3029~3078		2180~2186
 * CH3		3840~3850		3000~3040		2180~2200
 * CH4		3830~3850		3000~3040		2150~2180
 * CH5		3830~3850		3000~3040		2150~2180
 * CH6		3830~3850		3000~3040		2150~2180
 * CH7		3830~3850		3000~3040		2150~2180
 * */
/*
 * PE3 : 1ch    PE4 : 2ch    PE5 : 3ch      PE6 : 4ch
 * PF7 : 5ch    PF8 : 6ch    PF9 : 7ch
 * */
void RC_Return_dutycycle(uint32_t GPIO_Pin){
    int i=0;
    for(i=0;i<MAX_CHANNEL_NUM;i++){
        if(GPIO_Pin == ((uint16_t)1<<(i+3))){
            if(my_GPIO_InitStruct[i].Mode == GPIO_MODE_IT_RISING){
                ch_rising[i] = TIM3->CNT;
                HAL_GPIO_DeInit(gpio[i],(uint32_t)(1<<(i+3)));
                my_GPIO_InitStruct[i].Pin = (uint32_t)(1<<(i+3));
                my_GPIO_InitStruct[i].Mode = GPIO_MODE_IT_FALLING;
                my_GPIO_InitStruct[i].Pull = GPIO_PULLUP;
                HAL_GPIO_Init(gpio[i], &my_GPIO_InitStruct[i]);
            }
            else {
                ch_falling[i] = TIM3->CNT;
                rc_dutycycle[i] = (40000 - ch_rising[i] + ch_falling[i]) % 40000;
                HAL_GPIO_DeInit(gpio[i], (uint32_t)(1<<(i+3)));
                my_GPIO_InitStruct[i].Pin = (uint32_t)(1<<(i+3));
                my_GPIO_InitStruct[i].Mode = GPIO_MODE_IT_RISING;
                my_GPIO_InitStruct[i].Pull = GPIO_PULLUP;
                HAL_GPIO_Init(gpio[i], &my_GPIO_InitStruct[i]);
                check = 0;
            }
        }
    }
}
uint32_t RC_Read(uint32_t interrupt)
{
    uint32_t temp=rc_dutycycle[interrupt-1];
    constrain(temp,RC_MIN,RC_MAX);
    temp = RC_MAX-temp+RC_MIN;
    return temp;
}
















