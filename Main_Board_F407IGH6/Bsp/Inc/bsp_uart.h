#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "main.h"

#define UART_RX_BUF_SIZE  64

typedef struct {
    UART_HandleTypeDef *huart_;
    uint8_t             rx_buffer_[UART_RX_BUF_SIZE];
    volatile uint8_t    rx_index_;
    volatile uint8_t    rx_data_ready_;
} UartHandle;

void     uartInit(UART_HandleTypeDef *huart);
void     uartRxCallback(UART_HandleTypeDef *huart);
uint8_t  uartGetByte(uint8_t *byte);
void     uartSendByte(uint8_t byte);
void     uartSendBytes(const uint8_t *data, uint16_t len);

#endif /* __BSP_UART_H */