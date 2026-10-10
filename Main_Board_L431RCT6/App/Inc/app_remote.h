#ifndef __APP_REMOTE_H
#define __APP_REMOTE_H

#include "usart.h"
#include "stdint.h"

/* ESP32-S3 Xbox controller receiver UART protocol. */
#define CONTROLLER_RX_DMA_BUFFER_SIZE 36U
#define CONTROLLER_FRAME_SIZE         14U
#define CONTROLLER_FRAME_MAGIC0       0xC3U
#define CONTROLLER_FRAME_MAGIC1       0x3CU
#define CONTROLLER_VALID_FLAGS        0x03U
#define CONTROLLER_TIMEOUT_MS         150U

typedef enum {
    CONTROLLER_DISCONNECTED = 0,
    CONTROLLER_CONNECTED    = 1,
} ControllerConnectionState;

typedef enum {
    CONTROLLER_SAFETY_NORMAL = 0,
    CONTROLLER_SAFETY_LOST = 1,
    CONTROLLER_SAFETY_RECOVERING = 2,
} ControllerSafetyState;

typedef enum {
    CONTROLLER_DPAD_NONE = 0,
    CONTROLLER_DPAD_UP = 1,
    CONTROLLER_DPAD_UP_RIGHT = 2,
    CONTROLLER_DPAD_RIGHT = 3,
    CONTROLLER_DPAD_DOWN_RIGHT = 4,
    CONTROLLER_DPAD_DOWN = 5,
    CONTROLLER_DPAD_DOWN_LEFT = 6,
    CONTROLLER_DPAD_LEFT = 7,
    CONTROLLER_DPAD_UP_LEFT = 8,
} ControllerDpadDirection;

/* Individual button states decoded from the two protocol button bytes. */
typedef struct {
    uint8_t up_pressed_;
    uint8_t right_pressed_;
    uint8_t down_pressed_;
    uint8_t left_pressed_;
} ControllerDpad;

typedef struct {
    uint8_t a_pressed_;
    uint8_t b_pressed_;
    uint8_t x_pressed_;
    uint8_t y_pressed_;
    uint8_t lb_pressed_;
    uint8_t rb_pressed_;

    uint8_t back_pressed_;
    uint8_t start_pressed_;
    uint8_t xbox_pressed_;
    uint8_t left_stick_pressed_;
    uint8_t right_stick_pressed_;
} ControllerButtons;

typedef struct {
    UART_HandleTypeDef *uart_;

    /* Protocol signs are preserved: forward/left push produces a negative value. */
    int8_t left_stick_horizontal_;
    int8_t left_stick_vertical_;
    int8_t right_stick_horizontal_;
    int8_t right_stick_vertical_;

    uint8_t left_trigger_;   /* 0~255 */
    uint8_t right_trigger_;  /* 0~255 */
    uint8_t fire_requested_; /* right trigger > 0 */

    ControllerDpadDirection dpad_direction_;
    ControllerDpad dpad_;
    ControllerButtons buttons_;

    uint8_t flags_;
    uint8_t sequence_;
    uint8_t control_enabled_;
    ControllerConnectionState connection_state_;
    ControllerSafetyState safety_state_;
    uint32_t last_update_tick_;

    /* Runtime diagnostics, convenient to inspect in Ozone. */
    uint32_t valid_frame_count_;
    uint32_t crc_error_count_;
    uint32_t dropped_frame_count_;
} ControllerState;

extern ControllerState controller_state;
extern uint8_t controller_rx_dma_buffer[CONTROLLER_RX_DMA_BUFFER_SIZE];
extern uint8_t controller_last_frame[CONTROLLER_FRAME_SIZE];

/* 1 means enabled UART1 keyboard input currently has priority over gamepad. */
extern volatile uint8_t controller_pc_active_;
extern volatile uint32_t controller_pc_uart_errors_;
void controllerPcRxInit(void);
void controllerPcRxComplete(void);
void controllerPcRxError(void);

void    controllerRxInit(UART_HandleTypeDef *huart);
void    controllerRxCallback(ControllerState *state, uint8_t *buffer,
                             uint16_t dma_write_pos);
void    controllerRxResetParser(void);
uint8_t controllerIsLost(void);
uint8_t controllerDataIsValid(void);
uint8_t controllerDataIsError(void);
void    controllerRecover(void);
void    controllerRecoveryComplete(void);

#endif /* __APP_REMOTE_H */
