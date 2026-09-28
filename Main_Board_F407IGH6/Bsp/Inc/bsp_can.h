#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "main.h"

#define CAN_ID_BOARD1_CMD       0x111
#define CAN_ID_BOARD2_CMD       0x112
#define CAN_ID_BOARD1_SPEED     0x222
#define CAN_ID_BOARD1_POS       0x223
#define CAN_ID_BOARD2_SPEED     0x224
#define CAN_ID_BOARD2_POS       0x225
#define CAN_ID_GIMBAL_FEEDBACK  0x206

#define CAN_SEND_TIMEOUT        100000U
#define CAN_FRAME_DATA_LEN      8

typedef struct {
    CAN_HandleTypeDef *hcan_;
    uint32_t           tx_mailbox_;
} CanHandle;

void             canInit(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef canSend(uint32_t id, const uint8_t *data, uint8_t len);
void             canRxCallback(CAN_HandleTypeDef *hcan, uint32_t rx_fifo);

#endif /* __BSP_CAN_H */