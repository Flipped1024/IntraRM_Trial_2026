#include "tim.h"

void MX_TIM15_Init(void)
{
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_TIM15_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  gpio.Pin = GPIO_PIN_15;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF14_TIM15;
  HAL_GPIO_Init(GPIOB, &gpio);
  TIM15->PSC = 79U;
  TIM15->ARR = 999U;
  TIM15->CCR2 = 0U;
  TIM15->CCMR1 = (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
  TIM15->CCER |= TIM_CCER_CC2E;
  TIM15->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
  TIM15->EGR = TIM_EGR_UG;
}
