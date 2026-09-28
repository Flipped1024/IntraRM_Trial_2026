#include "bsp_usart_idle.h"

void uartIdleInit(UART_HandleTypeDef *huart, uint8_t *rx_buf, uint16_t dma_buf_num)
{
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    __HAL_DMA_DISABLE(huart->hdmarx);
    while (huart->hdmarx->Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(huart->hdmarx);
    }

    huart->hdmarx->Instance->PAR  = (uint32_t)&(huart->Instance->DR);
    huart->hdmarx->Instance->M0AR = (uint32_t)(rx_buf);
    huart->hdmarx->Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(huart->hdmarx);
}

void uartIdleIrqHandler(UART_HandleTypeDef *huart)
{
    uint32_t isrflags = READ_REG(huart->Instance->SR);
    uint32_t cr1its   = READ_REG(huart->Instance->CR1);
    uint32_t cr3its   = READ_REG(huart->Instance->CR3);

    if (huart->Instance->SR & UART_FLAG_RXNE) {
        __HAL_UART_CLEAR_PEFLAG(huart);
    } else if (huart->Instance->SR & UART_FLAG_IDLE) {
        __HAL_UART_CLEAR_PEFLAG(huart);

        __HAL_DMA_DISABLE(huart->hdmarx);

        __HAL_DMA_ENABLE(huart->hdmarx);

        userUartRxIdleCallback(huart);
    }

    if (((isrflags & USART_SR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET)) {
        __HAL_UART_DISABLE_IT(huart, UART_IT_TC);
        huart->gState = HAL_UART_STATE_READY;

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
        huart->TxCpltCallback(huart);
#else
        HAL_UART_TxCpltCallback(huart);
#endif
    }
}

__weak void userUartRxIdleCallback(UART_HandleTypeDef *huart)
{
    UNUSED(huart);
}