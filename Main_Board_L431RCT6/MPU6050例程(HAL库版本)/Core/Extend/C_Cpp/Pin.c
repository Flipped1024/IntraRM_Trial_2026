/*
 * Pin.c
 *
 *  Created on: Sep 20, 2024
 *      Author: benjamin
 */

#include "Pin.h"


void Pin_Blink(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,uint32_t time){
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 0);
	HAL_Delay(time);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 1);
	HAL_Delay(time);
}


void Pin_debounce(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,void (*debounce_function)()){
	if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)==0){
		HAL_Delay(10);
		if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)==1){
			HAL_Delay(10);
			debounce_function();
		}
	}


}

