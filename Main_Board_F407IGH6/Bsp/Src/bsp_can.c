#include "bsp_can.h"

static CanHandle can_handle_;

void canInit(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef can_filter_config;

    can_handle_.hcan_ = hcan;

    can_filter_config.FilterActivation    = ENABLE;
    can_filter_config.FilterBank          = 0;
    can_filter_config.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter_config.FilterMode          = CAN_FILTERMODE_IDLIST;
    can_filter_config.FilterScale         = CAN_FILTERSCALE_16BIT;
    can_filter_config.FilterIdHigh        = CAN_ID_BOARD1_SPEED << 5;
    can_filter_config.FilterIdLow         = CAN_ID_BOARD1_POS << 5;
    can_filter_config.FilterMaskIdHigh    = CAN_ID_BOARD2_SPEED << 5;
    can_filter_config.FilterMaskIdLow     = CAN_ID_BOARD2_POS << 5;
    can_filter_config.SlaveStartFilterBank = 0;

    HAL_CAN_ConfigFilter(hcan, &can_filter_config);

    can_filter_config.FilterBank          = 1;
    can_filter_config.FilterIdHigh        = CAN_ID_GIMBAL_FEEDBACK << 5;
    can_filter_config.FilterIdLow         = CAN_ID_GIMBAL_FEEDBACK << 5;
    can_filter_config.FilterMaskIdHigh    = CAN_ID_GIMBAL_FEEDBACK << 5;
    can_filter_config.FilterMaskIdLow     = CAN_ID_GIMBAL_FEEDBACK << 5;

    HAL_CAN_ConfigFilter(hcan, &can_filter_config);
    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

HAL_StatusTypeDef canSend(uint32_t id, const uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t            send_mail_box;
    uint32_t            timeout = CAN_SEND_TIMEOUT;

    tx_header.StdId               = id;
    tx_header.ExtId               = 0;
    tx_header.IDE                 = CAN_ID_STD;
    tx_header.RTR                 = CAN_RTR_DATA;
    tx_header.DLC                 = len;
    tx_header.TransmitGlobalTime  = DISABLE;

    while (timeout--) {
        if (can_handle_.hcan_->State == HAL_CAN_STATE_READY ||
            can_handle_.hcan_->State == HAL_CAN_STATE_LISTENING) {
            break;
        }
    }
    if (timeout == 0) {
        return HAL_ERROR;
    }

    timeout = CAN_SEND_TIMEOUT;
    while (timeout--) {
        if (HAL_CAN_GetTxMailboxesFreeLevel(can_handle_.hcan_) > 0) {
            break;
        }
    }
    if (timeout == 0) {
        return HAL_ERROR;
    }

    if ((can_handle_.hcan_->Instance->TSR & CAN_TSR_TME0) != 0U) {
        send_mail_box = CAN_TX_MAILBOX0;
    } else if ((can_handle_.hcan_->Instance->TSR & CAN_TSR_TME1) != 0U) {
        send_mail_box = CAN_TX_MAILBOX1;
    } else if ((can_handle_.hcan_->Instance->TSR & CAN_TSR_TME2) != 0U) {
        send_mail_box = CAN_TX_MAILBOX2;
    } else {
        return HAL_ERROR;
    }

    return HAL_CAN_AddTxMessage(can_handle_.hcan_, &tx_header,
                                (uint8_t *)data, &send_mail_box);
}

__weak void canRxCallback(CAN_HandleTypeDef *hcan, uint32_t rx_fifo)
{
    (void)hcan;
    (void)rx_fifo;
}