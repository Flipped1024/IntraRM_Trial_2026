/*
 * HAL_Add.h
 *
 *  Created on: Sep 25, 2024
 *      Author: benjamin
 */

#ifndef EXTEND__SUNDRIES_HAL_ADD_H_
#define EXTEND__SUNDRIES_HAL_ADD_H_
#include "main.h"
#include "stdio.h"
//告诉编译器我们的的确确有这个值在，它是存在的，让编译器自己去找

///////////////////////USART////////////////////////////
#ifdef HAL_UART_MODULE_ENABLED
	#include "usart.h"
	int __io_putchar(int ch);//可以使用printf
#endif

///////////////////////IIC////////////////////////////
#ifdef HAL_I2C_MODULE_ENABLED
	#include "i2c.h"
	void HI2C_Init();

#endif

///////////////////////TIM////////////////////////////
#ifdef HAL_TIM_MODULE_ENABLED
	#include "tim.h"
	extern uint32_t timer_over_float;
	void Delay_us(uint16_t us);//默认使用htim6
	void TIM_Reset(TIM_HandleTypeDef* Timer,uint32_t ms);
	void TIM_Speed_Check_Start(TIM_HandleTypeDef* Timer);
	void TIM_Speed_Check_Stop(TIM_HandleTypeDef* Timer,uint32_t* MCU_Speed);
	void PWM_Init(TIM_HandleTypeDef* Timer,uint32_t Channel,uint32_t hz);
	void PWM_complementary(TIM_HandleTypeDef* Timer,uint32_t Channel,uint32_t hz);
	void PWM_Dut(TIM_HandleTypeDef* Timer,uint32_t Channel,uint8_t dut);
#endif

///////////////////////ADC////////////////////////////
#ifdef HAL_ADC_MODULE_ENABLED
	#include "adc.h"
	uint32_t ADC_Get(ADC_HandleTypeDef* Adc,uint32_t Channel);
	float ADC_Get_Voltage(ADC_HandleTypeDef* Adc,uint32_t Channel);
	uint16_t ADC_DMA_GET(ADC_HandleTypeDef* Adc,uint16_t* Buffer,uint8_t num);
	float ADC_Chip_temp_Get();
#endif

///////////////////////DAC////////////////////////////
#ifdef HAL_DAC_MODULE_ENABLED
	#include "dac.h"
	void DAC_Set_voltage(uint32_t DAC_CHANNEL_x,float voltage);
#endif

#endif /* EXTEND__SUNDRIES_HAL_ADD_H_ */
