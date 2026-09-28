#include "bsp_can.h"
#include "can.h"

extern CAN_HandleTypeDef hcan1;

void BSP_CAN_Init(void)
{
    CAN_FilterTypeDef filter = {0};
    MX_CAN1_Init();
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING |
                                          CAN_IT_ERROR_WARNING |
                                          CAN_IT_ERROR_PASSIVE |
                                          CAN_IT_BUSOFF |
                                          CAN_IT_LAST_ERROR_CODE);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 5, 1);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
}

uint8_t BSP_CAN_SendMessage(uint32_t id, uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    tx_header.StdId = id;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;
    tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &tx_mailbox) != HAL_OK)
    {
        return 0;
    }
    return 1;
}
