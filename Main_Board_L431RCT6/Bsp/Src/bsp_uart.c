#include "bsp_uart.h"

static UartHandle uart_handle_;

void uartInit(UART_HandleTypeDef *huart)
{
    uart_handle_.huart_        = huart;
    uart_handle_.rx_index_     = 0;
    uart_handle_.rx_data_ready_ = 0;

    __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
}

void uartRxCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == uart_handle_.huart_->Instance) {
        uint8_t byte;
        if (HAL_UART_Receive(huart, &byte, 1, 0) == HAL_OK) {
            if (uart_handle_.rx_index_ < UART_RX_BUF_SIZE) {
                uart_handle_.rx_buffer_[uart_handle_.rx_index_++] = byte;
            }
            uart_handle_.rx_data_ready_ = 1;
        }
    }
}

uint8_t uartGetByte(uint8_t *byte)
{
    if (uart_handle_.rx_data_ready_ && uart_handle_.rx_index_ > 0) {
        *byte = uart_handle_.rx_buffer_[0];
        for (uint8_t i = 1; i < uart_handle_.rx_index_; i++) {
            uart_handle_.rx_buffer_[i - 1] = uart_handle_.rx_buffer_[i];
        }
        uart_handle_.rx_index_--;
        if (uart_handle_.rx_index_ == 0) {
            uart_handle_.rx_data_ready_ = 0;
        }
        return 1;
    }
    return 0;
}

void uartSendByte(uint8_t byte)
{
    HAL_UART_Transmit(uart_handle_.huart_, &byte, 1, HAL_MAX_DELAY);
}

void uartSendBytes(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(uart_handle_.huart_, (uint8_t *)data, len, HAL_MAX_DELAY);
}