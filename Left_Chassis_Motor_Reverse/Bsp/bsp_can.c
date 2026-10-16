// bsp_can.c
#include "bsp_can.h"
#include "app_debug.h"

static CanRxCallback g_rx_callback = NULL;

void bspCanRegisterCallback(CanRxCallback callback)
{
    g_rx_callback = callback;
}

void bspCanInit(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filter_config;

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

    HAL_StatusTypeDef filter_status = HAL_CAN_ConfigFilter(hcan, &filter_config);
    HAL_StatusTypeDef start_status = HAL_CAN_Start(hcan);
    HAL_StatusTypeDef notification_status = HAL_CAN_ActivateNotification(
        hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR_WARNING |
        CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE |
        CAN_IT_ERROR);
    appDebugCanInitResult(filter_status, start_status, notification_status);
    appDebugCanState(hcan);
}

CanStatus bspCanTransmit(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    volatile CanStatus result = CAN_STATUS_ERROR;

    if (hcan == NULL || data == NULL)
    {
        result = CAN_STATUS_ERROR;
        appDebugCanTx((uint32_t)result);
        return result;
    }

    tx_header.StdId = id;
    tx_header.ExtId = 0x00;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
    {
        result = CAN_STATUS_ERROR;
        appDebugCanTx((uint32_t)result);
        appDebugCanState(hcan);
        return result;
    }

    if (HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox) != HAL_OK)
    {
        result = CAN_STATUS_ERROR;
        appDebugCanTx((uint32_t)result);
        appDebugCanState(hcan);
        return result;
    }

    result = CAN_STATUS_OK;
    appDebugCanTx((uint32_t)result);
    appDebugCanState(hcan);
    return result;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {
        appDebugCanRx(rx_header.StdId, rx_data);
        appDebugCanState(hcan);
        if (g_rx_callback != NULL)
        {
            g_rx_callback(rx_header.StdId, rx_data);
        }
    }
    else
    {
        appDebugCanRxError();
        appDebugCanState(hcan);
    }
}
