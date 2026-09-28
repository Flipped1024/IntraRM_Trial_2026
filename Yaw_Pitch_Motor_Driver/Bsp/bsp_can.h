// bsp_can.h
#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include "board_config.h"

#define CAN_TX_TIMEOUT 100U
#define CAN_FILTER_BANK 0U
typedef enum {
    CAN_STATUS_OK = 0,
    CAN_STATUS_ERROR = 1
} CanStatus;

typedef void (*CanRxCallback)(uint32_t id, const uint8_t* data);

/* Runtime CAN diagnostics; watch this object in Ozone. */
typedef struct {
    uint32_t filter_status;
    uint32_t start_status;
    uint32_t notification_status;
    uint32_t tx_attempts;
    uint32_t tx_accepted;
    uint32_t tx_errors;
    uint32_t tx_last_id;
    uint32_t tx_last_mailbox;
    uint32_t rx_frames;
    uint32_t rx_ctrl_frames;
    uint32_t rx_other_frames;
    uint32_t rx_last_id;
    uint32_t rx_last_dlc;
    uint8_t rx_last_data[8];
    uint32_t error_callbacks;
    uint32_t hal_error_code;
} CanDebugData;

extern volatile CanDebugData can_debug_data;

void bspCanInit(CAN_HandleTypeDef* hcan);
CanStatus bspCanTransmit(CAN_HandleTypeDef* hcan, uint32_t id, uint8_t* data);

void bspCanRegisterCallback(CanRxCallback callback);

#endif /* BSP_CAN_H */
