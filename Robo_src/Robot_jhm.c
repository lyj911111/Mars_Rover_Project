/*
 * Robot_jhm.c
 *
 *  Created on: 2018. 9. 22.
 *      Author: jhm
 */


#include "Robot_jhm.h"
volatile int rc_dutycycle_1;
volatile int rc_dutycycle_2;
volatile int rc_dutycycle_3;
volatile int rc_dutycycle_4;
volatile int rc_dutycycle_5;
volatile int rc_dutycycle_6;
volatile int rc_dutycycle_7;

volatile  uint16_t ch1_rising, ch1_falling;
volatile  uint16_t ch2_rising, ch2_falling;
volatile  uint16_t ch3_rising, ch3_falling;
volatile  uint16_t ch4_rising, ch4_falling;
volatile  uint16_t ch5_rising, ch5_falling;
volatile  uint16_t ch6_rising, ch6_falling;
volatile  uint16_t ch7_rising, ch7_falling;
/*			MAX				MIDDLE			MIN
 * CH1  	3865~3871		3032~3077		2190~2196
 * CH2		3854~3859		3029~3078		2180~2186
 * CH3		3840~3850		3000~3040		2180~2200
 * CH4		3830~3850		3000~3040		2150~2180
 * CH5		3830~3850		3000~3040		2150~2180
 * CH6		3830~3850		3000~3040		2150~2180
 * CH7		3830~3850		3000~3040		2150~2180
 * */
void RC_Return_dutycycle(uint32_t GPIO_Pin){
    GPIO_InitTypeDef my1_GPIO_InitStruct;
    GPIO_InitTypeDef my2_GPIO_InitStruct;
    GPIO_InitTypeDef my3_GPIO_InitStruct;
    GPIO_InitTypeDef my4_GPIO_InitStruct;
    GPIO_InitTypeDef my5_GPIO_InitStruct;
    GPIO_InitTypeDef my6_GPIO_InitStruct;
    GPIO_InitTypeDef my7_GPIO_InitStruct;

    if(GPIO_Pin == GPIO_EXTI3_Pin){             //(EXTI PIN)PE3 : 1ch
        if(my1_GPIO_InitStruct.Mode == GPIO_MODE_IT_RISING){
            HAL_GPIO_TogglePin(GPIOB, Red_LED_Pin);     //debug
            ch1_rising = TIM3->CNT;
            HAL_GPIO_DeInit(GPIOE, GPIO_EXTI3_Pin);
            my1_GPIO_InitStruct.Pin = GPIO_EXTI3_Pin;
            my1_GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
            my1_GPIO_InitStruct.Pull = GPIO_PULLUP;
            HAL_GPIO_Init(GPIOE, &my1_GPIO_InitStruct);
        }
        else {
            ch1_falling = TIM3->CNT;
            rc_dutycycle_1 = (40000 - ch1_rising + ch1_falling) % 40000;
            HAL_GPIO_DeInit(GPIOE, GPIO_EXTI3_Pin);
            my1_GPIO_InitStruct.Pin = GPIO_EXTI3_Pin;
            my1_GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
            my1_GPIO_InitStruct.Pull = GPIO_PULLUP;
            HAL_GPIO_Init(GPIOE, &my1_GPIO_InitStruct);
        }
    }

    else if (GPIO_Pin == GPIO_EXTI4_Pin){               //(EXTI PIN)PE4 : 2ch
        if(my2_GPIO_InitStruct.Mode == GPIO_MODE_IT_RISING){
            ch2_rising = TIM3->CNT;
            HAL_GPIO_DeInit(GPIOE, GPIO_EXTI4_Pin);
            my2_GPIO_InitStruct.Pin = GPIO_EXTI4_Pin;
            my2_GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
            my2_GPIO_InitStruct.Pull = GPIO_PULLUP;
            HAL_GPIO_Init(GPIOE, &my2_GPIO_InitStruct);
        }
        else {
            ch2_falling = TIM3->CNT;
            rc_dutycycle_2 = (40000 - ch2_rising + ch2_falling) % 40000;
            HAL_GPIO_DeInit(GPIOE, GPIO_EXTI4_Pin);
            my2_GPIO_InitStruct.Pin = GPIO_EXTI4_Pin;
            my2_GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
            my2_GPIO_InitStruct.Pull = GPIO_PULLUP;
            HAL_GPIO_Init(GPIOE, &my2_GPIO_InitStruct);
        }
    }

    else if (GPIO_Pin == GPIO_EXTI5_Pin){               //(EXTI PIN)PE5 : 3ch
        if(my3_GPIO_InitStruct.Mode == GPIO_MODE_IT_RISING){
            ch3_rising = TIM3->CNT;
            HAL_GPIO_DeInit(GPIOE, GPIO_EXTI5_Pin);
            my3_GPIO_InitStruct.Pin = GPIO_EXTI5_Pin;
            my3_GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
            my3_GPIO_InitStruct.Pull = GPIO_PULLUP;
            HAL_GPIO_Init(GPIOE, &my3_GPIO_InitStruct);
        }
        else {
            ch3_falling = TIM3->CNT;
            rc_dutycycle_3 = (40000 - ch3_rising + ch3_falling) % 40000;
            HAL_GPIO_DeInit(GPIOE, GPIO_EXTI5_Pin);
            my3_GPIO_InitStruct.Pin = GPIO_EXTI5_Pin;
            my3_GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
            my3_GPIO_InitStruct.Pull = GPIO_PULLUP;
            HAL_GPIO_Init(GPIOE, &my3_GPIO_InitStruct);
        }
    }

    else if (GPIO_Pin == GPIO_EXTI6_Pin){               //(EXTI PIN)PE6 : 4ch
        if(my4_GPIO_InitStruct.Mode == GPIO_MODE_IT_RISING){
            ch4_rising = TIM3->CNT;
            HAL_GPIO_DeInit(GPIOE, GPIO_EXTI6_Pin);
            my4_GPIO_InitStruct.Pin = GPIO_EXTI6_Pin;
            my4_GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
            my4_GPIO_InitStruct.Pull = GPIO_PULLUP;
            HAL_GPIO_Init(GPIOE, &my4_GPIO_InitStruct);
        }
        else {
            ch4_falling = TIM3->CNT;
            rc_dutycycle_4 = (40000 - ch4_rising + ch4_falling) % 40000;
            HAL_GPIO_DeInit(GPIOE, GPIO_EXTI6_Pin);
            my4_GPIO_InitStruct.Pin = GPIO_EXTI6_Pin;
            my4_GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
            my4_GPIO_InitStruct.Pull = GPIO_PULLUP;
            HAL_GPIO_Init(GPIOE, &my4_GPIO_InitStruct);
        }
    }

    else if (GPIO_Pin == GPIO_EXTI7_Pin){               //(EXTI PIN)PF7 : 5ch
        if(my5_GPIO_InitStruct.Mode == GPIO_MODE_IT_RISING){
            ch5_rising = TIM3->CNT;
            HAL_GPIO_DeInit(GPIOF, GPIO_EXTI7_Pin);
            my5_GPIO_InitStruct.Pin = GPIO_EXTI7_Pin;
            my5_GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
            my5_GPIO_InitStruct.Pull = GPIO_PULLUP;
            HAL_GPIO_Init(GPIOF, &my5_GPIO_InitStruct);
        }
        else {
            ch5_falling = TIM3->CNT;
            rc_dutycycle_5 = (40000 - ch5_rising + ch5_falling) % 40000;
            HAL_GPIO_DeInit(GPIOF, GPIO_EXTI7_Pin);
            my5_GPIO_InitStruct.Pin = GPIO_EXTI7_Pin;
            my5_GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
            my5_GPIO_InitStruct.Pull = GPIO_PULLUP;
            HAL_GPIO_Init(GPIOF, &my5_GPIO_InitStruct);
        }
    }

    else if (GPIO_Pin == GPIO_EXTI8_Pin){               //(EXTI PIN)PF8 : 6ch
        if(my6_GPIO_InitStruct.Mode == GPIO_MODE_IT_RISING){
            ch6_rising = TIM3->CNT;
            HAL_GPIO_DeInit(GPIOF, GPIO_EXTI8_Pin);
            my6_GPIO_InitStruct.Pin = GPIO_EXTI8_Pin;
            my6_GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
            my6_GPIO_InitStruct.Pull = GPIO_PULLUP;
            HAL_GPIO_Init(GPIOF, &my6_GPIO_InitStruct);
        }
        else {
            ch6_falling = TIM3->CNT;
            rc_dutycycle_6 = (40000 - ch6_rising + ch6_falling) % 40000;
            HAL_GPIO_DeInit(GPIOF, GPIO_EXTI8_Pin);
            my6_GPIO_InitStruct.Pin = GPIO_EXTI8_Pin;
            my6_GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
            my6_GPIO_InitStruct.Pull = GPIO_PULLUP;
            HAL_GPIO_Init(GPIOF, &my6_GPIO_InitStruct);
        }
    }

    else if (GPIO_Pin == GPIO_EXTI9_Pin){               //(EXTI PIN)PF8 : 6ch
        if(my7_GPIO_InitStruct.Mode == GPIO_MODE_IT_RISING){
            ch7_rising = TIM3->CNT;
            HAL_GPIO_DeInit(GPIOF, GPIO_EXTI9_Pin);
            my7_GPIO_InitStruct.Pin = GPIO_EXTI9_Pin;
            my7_GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
            my7_GPIO_InitStruct.Pull = GPIO_PULLUP;
            HAL_GPIO_Init(GPIOF, &my7_GPIO_InitStruct);
        }
        else {
            ch7_falling = TIM3->CNT;
            rc_dutycycle_7 = (40000 - ch7_rising + ch7_falling) % 40000;
            HAL_GPIO_DeInit(GPIOF, GPIO_EXTI9_Pin);
            my7_GPIO_InitStruct.Pin = GPIO_EXTI9_Pin;
            my7_GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
            my7_GPIO_InitStruct.Pull = GPIO_PULLUP;
            HAL_GPIO_Init(GPIOF, &my7_GPIO_InitStruct);
        }
    }
}
int RC_Read(uint32_t interrupt)
{
    if(interrupt==1)
        return rc_dutycycle_1;
    else if(interrupt==2)
        return rc_dutycycle_2;
    else if(interrupt==3)
        return rc_dutycycle_3;
    else if(interrupt==4)
        return rc_dutycycle_4;
    else if(interrupt==5)
        return rc_dutycycle_5;
    else if(interrupt==6)
        return rc_dutycycle_6;
    else if(interrupt==7)
        return rc_dutycycle_7;
    else
        return 0;
}
















