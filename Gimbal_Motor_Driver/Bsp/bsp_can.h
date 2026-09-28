// bsp_can.h
#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define CAN_TX_TIMEOUT 100U
#define CAN_FILTER_BANK 0U
#define CAN_FILTER_ID 0x113U

typedef enum {
    CAN_STATUS_OK = 0,
    CAN_STATUS_ERROR = 1
} CanStatus;

typedef void (*CanRxCallback)(uint32_t id, const uint8_t* data);

void bspCanInit(CAN_HandleTypeDef* hcan);
CanStatus bspCanTransmit(CAN_HandleTypeDef* hcan, uint32_t id, uint8_t* data);

void bspCanRegisterCallback(CanRxCallback callback);

#endif /* BSP_CAN_H */