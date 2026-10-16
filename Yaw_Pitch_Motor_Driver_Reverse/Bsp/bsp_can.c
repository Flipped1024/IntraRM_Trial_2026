// bsp_can.c
#include "bsp_can.h"

static CanRxCallback g_rx_callback = NULL;
volatile CanDebugData can_debug_data;

void bspCanRegisterCallback(CanRxCallback callback)
{
    g_rx_callback = callback;
}

void bspCanInit(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filter_config = {0};

    if (hcan == NULL)
    {
        return;
    }

    filter_config.FilterBank = CAN_FILTER_BANK;
    filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
    filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
    filter_config.FilterIdHigh = (CAN_FILTER_ID << 5);
    filter_config.FilterIdLow = 0x0000;
    filter_config.FilterMaskIdHigh = (0x7FF << 5);
    filter_config.FilterMaskIdLow = 0x0000;
    filter_config.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter_config.FilterActivation = ENABLE;

    can_debug_data.filter_status = (uint32_t)HAL_CAN_ConfigFilter(hcan, &filter_config);
    can_debug_data.start_status = (uint32_t)HAL_CAN_Start(hcan);
    can_debug_data.notification_status =
        (uint32_t)HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    can_debug_data.hal_error_code = hcan->ErrorCode;
}

CanStatus bspCanTransmit(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    CanStatus result = CAN_STATUS_ERROR;

    if (hcan == NULL || data == NULL)
    {
        result = CAN_STATUS_ERROR;
        return result;
    }

    tx_header.StdId = id;
    tx_header.ExtId = 0x00;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;
    can_debug_data.tx_attempts++;
    can_debug_data.tx_last_id = id;

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
    {
        can_debug_data.tx_errors++;
        can_debug_data.hal_error_code = hcan->ErrorCode;
        result = CAN_STATUS_ERROR;
        return result;
    }

    if (HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox) != HAL_OK)
    {
        can_debug_data.tx_errors++;
        can_debug_data.hal_error_code = hcan->ErrorCode;
        result = CAN_STATUS_ERROR;
        return result;
    }

    can_debug_data.tx_accepted++;
    can_debug_data.tx_last_mailbox = tx_mailbox;
    result = CAN_STATUS_OK;
    return result;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {
        can_debug_data.rx_frames++;
        can_debug_data.rx_last_id = rx_header.StdId;
        can_debug_data.rx_last_dlc = rx_header.DLC;
        for (uint8_t i = 0; i < 8; i++)
        {
            can_debug_data.rx_last_data[i] = rx_data[i];
        }
        if (rx_header.StdId == CTRL_MSG_ID)
        {
            can_debug_data.rx_ctrl_frames++;
        }
        else
        {
            can_debug_data.rx_other_frames++;
        }
        if (g_rx_callback != NULL)
        {
            g_rx_callback(rx_header.StdId, rx_data);
        }
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan != NULL)
    {
        can_debug_data.error_callbacks++;
        can_debug_data.hal_error_code = hcan->ErrorCode;
    }
}
