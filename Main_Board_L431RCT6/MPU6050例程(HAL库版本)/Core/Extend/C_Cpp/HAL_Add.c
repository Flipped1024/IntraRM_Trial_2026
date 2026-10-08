/*
 * HAL_Add.c
 *
 *  Created on: Sep 25, 2024
 *      Author: benjamin
 */
#include "HAL_Add.h"

///////////////////////USART////////////////////////////
#ifdef HAL_UART_MODULE_ENABLED
	int __io_putchar(int ch){
		HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1,100);
		return ch;
	}
#endif

///////////////////////I2C////////////////////////////
#ifdef HAL_I2C_MODULE_ENABLED

	void HI2C_Init(){

	}

#endif

///////////////////////TIM////////////////////////////
#ifdef HAL_TIM_MODULE_ENABLED//
#define Use_TIM6 0
#if Use_TIM6//如果没有使用htim6,静止使用该函数
	void Delay_us(uint16_t us){
		__HAL_TIM_SET_COUNTER(&htim6, 0);//对定时器清0
		HAL_TIM_Base_Start(&htim6);//1Mhz
		while(__HAL_TIM_GET_COUNTER(&htim6)<=us);//延时
		HAL_TIM_Base_Stop(&htim6);
	}
#else
#endif
//1ms~1300ms
void TIM_Reset(TIM_HandleTypeDef* Timer,uint32_t ms){
	Timer->Init.Prescaler = 16800 - 1;
	Timer->Init.Period = 5*ms;
	HAL_TIM_Base_Init(Timer);
	HAL_TIM_Base_Start_IT(Timer);
}

uint32_t timer_over_float = 0;
void TIM_Speed_Check_Start(TIM_HandleTypeDef* Timer){
	static uint8_t once_gate = 1;
	if(once_gate){	//让定时器程序只初始化一次
		Timer->Init.Period = 65535;
		Timer->Init.Prescaler = 72 -1;//TIM1 APB1 is 84M 84/84 = 1M speed to counter 1us
		Timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		Timer->Init.CounterMode = TIM_COUNTERMODE_UP;
		Timer->Init.RepetitionCounter = 0;
		HAL_TIM_Base_Init(Timer);
		__HAL_TIM_SET_COUNTER(Timer, 0);
		once_gate=0;
	}
	HAL_TIM_Base_Start_IT(Timer);//锟斤拷始锟斤拷锟斤拷
}
void TIM_Speed_Check_Stop(TIM_HandleTypeDef* Timer,uint32_t* MCU_Speed){//单位ms
	HAL_TIM_Base_Stop_IT(Timer);
	*MCU_Speed = timer_over_float*65535 + __HAL_TIM_GET_COUNTER(Timer);//unit us
	if(*MCU_Speed>=1000)*MCU_Speed/=1000;	//us转为ms
	timer_over_float = 0;					//清除溢出值
	__HAL_TIM_SET_COUNTER(Timer, 0);		//清0计数器
}

void PWM_Init(TIM_HandleTypeDef* Timer,uint32_t Channel,uint32_t hz){
	if(hz>50000){
		Timer->Init.Prescaler = 500000/hz - 1;
		Timer->Init.Period = 168 - 1;
	}
	else{
		Timer->Init.Prescaler = 50000/hz - 1;
		Timer->Init.Period = 1680 - 1;
	}
	HAL_TIM_Base_Init(Timer);
	HAL_TIM_PWM_Start(Timer, Channel);
}
void PWM_complementary(TIM_HandleTypeDef* Timer,uint32_t Channel,uint32_t hz){
	Timer->Init.Prescaler = 100000/hz - 1;
	Timer->Init.Period = 1680 - 1;
	HAL_TIM_Base_Init(Timer);
	HAL_TIM_PWM_Start(Timer, Channel);
	HAL_TIMEx_PWMN_Start(Timer, Channel);
}
void PWM_Dut(TIM_HandleTypeDef* Timer,uint32_t Channel,uint8_t dut){
	__HAL_TIM_SET_COMPARE(Timer,Channel,Timer->Init.Period*dut/100);
}
#endif


/*
 ADC_SAMPLETIME_3CYCLES
 ADC_SAMPLETIME_15CYCLES
 ADC_SAMPLETIME_28CYCLES
 ADC_SAMPLETIME_56CYCLES
 ADC_SAMPLETIME_84CYCLES
 ADC_SAMPLETIME_112CYCLES
 ADC_SAMPLETIME_144CYCLES
 ADC_SAMPLETIME_480CYCLES
 */
///////////////////////ADC////////////////////////////
#ifdef HAL_ADC_MODULE_ENABLED
uint32_t ADC_Get(ADC_HandleTypeDef* Adc,uint32_t Channel){
	HAL_ADC_Start(Adc);
	HAL_ADC_PollForConversion(Adc, 10); // wait 10 ms for convertion

	return HAL_ADC_GetValue(Adc);
}
float ADC_Get_Voltage(ADC_HandleTypeDef* Adc,uint32_t Channel){
	HAL_ADC_Start(Adc);
	HAL_ADC_PollForConversion(Adc, 10);

	return HAL_ADC_GetValue(Adc)*3.3/4095;
}
//(where do your data come from,)
uint16_t ADC_DMA_GET(ADC_HandleTypeDef* Adc,uint16_t* Buffer,uint8_t num){
	HAL_ADC_Start_DMA(Adc, (uint32_t*)Buffer, sizeof(Buffer));
	return Buffer[num];
}
float ADC_Chip_temp_Get(){
	float temprature = 0;
	uint32_t average = 0;
	uint8_t i;
	uint8_t count = 100;
	HAL_ADC_Start(&hadc1);
	for(i = 0;i<count;i++){
		average += HAL_ADC_GetValue(&hadc1);
	}
	average =average/count;
	temprature = average*3.3f/4095;
	temprature = (temprature - 0.76f) / 0.0025f + 25;
	return temprature;
}
#endif
/*
#ifdef HAL_DAC_MODULE_ENABLED

void DAC_Set_voltage(uint32_t DAC_CHANNEL_x,float voltage){
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_x, DAC_ALIGN_12B_R, voltage*4095/3.3);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_x);
}
#ifdef HAL_ADC_MODULE_ENABLED
void DAC_Set_voltage_Hight(uint32_t DAC_CHANNEL_x,float voltage){
	//float real_voltage = ;
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_x, DAC_ALIGN_12B_R, voltage*4095/3.3);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_x);
}
#endif
*/

