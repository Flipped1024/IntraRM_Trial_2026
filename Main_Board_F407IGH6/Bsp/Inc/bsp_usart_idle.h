#ifndef __BSP_USART_IDLE_H
#define __BSP_USART_IDLE_H

#include "usart.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"

void uartIdleInit(UART_HandleTypeDef *huart, uint8_t *rx_buf, uint16_t dma_buf_num);
void uartIdleIrqHandler(UART_HandleTypeDef *huart);
void uartRestart(uint16_t dma_buf_num);

__weak void userUartRxIdleCallback(UART_HandleTypeDef *huart);

#endif