#include "bsp_usart_idle.h"

void uartIdleInit(UART_HandleTypeDef *huart, uint8_t *rx_buf, uint16_t dma_buf_num)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buf, dma_buf_num) == HAL_OK) {
        /* Circular DMA is consumed through UART IDLE events. */
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT | DMA_IT_TC);
    }
}

void uartIdleIrqHandler(UART_HandleTypeDef *huart)
{
    UNUSED(huart);
}

__weak void userUartRxIdleCallback(UART_HandleTypeDef *huart)
{
    UNUSED(huart);
}
