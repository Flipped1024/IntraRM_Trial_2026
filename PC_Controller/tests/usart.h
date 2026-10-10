#ifndef TEST_USART_H
#define TEST_USART_H
#include <stdint.h>
typedef struct { int unused; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart1, huart3;
#define HAL_OK 0
#define USART1_IRQn 37
uint32_t HAL_GetTick(void);
void HAL_NVIC_SetPriority(int irq, unsigned priority, unsigned sub);
void HAL_NVIC_EnableIRQ(int irq);
int HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
int HAL_UART_AbortReceive(UART_HandleTypeDef *);
void Error_Handler(void);
#endif
