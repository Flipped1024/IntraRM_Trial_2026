#include "app_debug.h"
#include "bsp_can.h"
#include "app_protocol.h"

DebugData g_debug_;

void debugInit(void)
{
    uint8_t i;

    g_debug_.mode_ = DEBUG_MODE_NONE;

    g_debug_.chassis_.vx_ = 0.0f;
    g_debug_.chassis_.vy_ = 0.0f;
    g_debug_.chassis_.vw_ = 0.0f;

    for (i = 0; i < 4; i++) {
        g_debug_.can_.motor_rpm_[i] = 0;
    }
    g_debug_.can_.tx_count_           = 0;
    g_debug_.can_.tx_ok_count_        = 0;
    g_debug_.can_.tx_fail_count_      = 0;
    g_debug_.can_.rx_count_           = 0;
    g_debug_.can_.last_rx_id_         = 0;
    for (i = 0; i < 8; i++) {
        g_debug_.can_.last_rx_data_[i] = 0;
    }
    g_debug_.can_.can_state_          = 0;
    g_debug_.can_.can_error_          = 0;
    g_debug_.can_.can_tx_mailbox_free_ = 0;
    g_debug_.can_.can_init_ok_        = 0;
}

void debugUpdateCanTx(HAL_StatusTypeDef status)
{
    g_debug_.can_.tx_count_++;
    if (status == HAL_OK) {
        g_debug_.can_.tx_ok_count_++;
    } else {
        g_debug_.can_.tx_fail_count_++;
    }
}

void debugUpdateCanRx(uint32_t can_id, const uint8_t *data)
{
    uint8_t i;

    g_debug_.can_.rx_count_++;
    g_debug_.can_.last_rx_id_ = can_id;
    for (i = 0; i < 8; i++) {
        g_debug_.can_.last_rx_data_[i] = data[i];
    }
}

void debugUpdateCanState(void)
{
    g_debug_.can_.can_state_          = (uint32_t)HAL_CAN_GetState(&hcan1);
    g_debug_.can_.can_error_          = (uint32_t)HAL_CAN_GetError(&hcan1);
    g_debug_.can_.can_tx_mailbox_free_ = (uint32_t)HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
}

void debugRunChassis(Chassis *chassis)
{
    HAL_StatusTypeDef status;

    chassisSetTarget(chassis,
                     g_debug_.chassis_.vx_,
                     g_debug_.chassis_.vy_,
                     g_debug_.chassis_.vw_);
    chassisUpdate(chassis);

    status = protocolSendBoardCmd(1, &chassis->motor_cmds_[0]);
    debugUpdateCanTx(status);

    status = protocolSendBoardCmd(2, &chassis->motor_cmds_[1]);
    debugUpdateCanTx(status);

    debugUpdateCanState();
}

void debugRunCanDirect(void)
{
    MotorCmd cmd_board1;
    MotorCmd cmd_board2;
    HAL_StatusTypeDef status;

    cmd_board1.motor1_speed_ = g_debug_.can_.motor_rpm_[0];
    cmd_board1.motor2_speed_ = g_debug_.can_.motor_rpm_[1];
    cmd_board2.motor1_speed_ = g_debug_.can_.motor_rpm_[2];
    cmd_board2.motor2_speed_ = g_debug_.can_.motor_rpm_[3];

    status = protocolSendBoardCmd(1, &cmd_board1);
    debugUpdateCanTx(status);
    debugUpdateCanState();

    status = protocolSendBoardCmd(2, &cmd_board2);
    debugUpdateCanTx(status);
}
