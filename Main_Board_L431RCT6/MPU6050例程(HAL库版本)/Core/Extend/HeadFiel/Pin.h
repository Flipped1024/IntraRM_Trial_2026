/*
 * Pin.h
 *
 *  Created on: Sep 20, 2024
 *      Author: benjamin
 */

#ifndef EXTEND_HEADFIEL_PIN_H_
#define EXTEND_HEADFIEL_PIN_H_
#include "main.h"

#define Fast_config_GPIO 0 //Enable it ,it can configure fast GPIO

#if Fast_config_GPIO == 1
#define GPIO_A1(x) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, x)
#define GPIO_A2(x) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, x)
#define GPIO_A3(x) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, x)
#define GPIO_A4(x) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, x)
#define GPIO_A10(x) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, x)
#define GPIO_B1(x) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, x)
#define GPIO_B2(x) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, x)
#define GPIO_B3(x) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, x)
#define GPIO_B4(x) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, x)
#define GPIO_C0(x) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, x)
#define GPIO_C1(x) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, x)
#define GPIO_C2(x) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, x)
#define GPIO_C3(x) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, x)
#define GPIO_C4(x) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, x)

#endif

void Pin_Blink(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,uint32_t time);//Flip voltage by count
void Pin_debounce(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,void (*function)());

#endif /* EXTEND_HEADFIEL_PIN_H_ */
