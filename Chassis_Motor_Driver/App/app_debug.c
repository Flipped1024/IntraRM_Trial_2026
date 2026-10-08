#include "app_debug.h"
#include "app_control.h"
#include "can.h"

volatile DebugData debug_data;

void appDebugInit(void)
{
    debug_data.spd_kp = MOTOR_SPEED_PID_KP;
    debug_data.spd_ki = MOTOR_SPEED_PID_KI;
    debug_data.spd_kd = MOTOR_SPEED_PID_KD;
    debug_data.spd_max_out = (float)MOTOR_SPEED_PID_MAX_OUT;
    debug_data.spd_max_integral = MOTOR_SPEED_PID_MAX_INTEGRAL_OUT;
    debug_data.spd_filter_alpha = MOTOR_SPEED_FILTER_ALPHA;
    debug_data.spd_ff_accel = MOTOR_SPEED_FF_ACCEL_GAIN;
    debug_data.debug_mode = 0;
    debug_data.can.state = 0U;
    debug_data.can.error = 0U;
    debug_data.can.rx_count = 0U;
    debug_data.can.rx_error_count = 0U;
    debug_data.can.last_rx_id = 0U;
    debug_data.can.tx_count = 0U;
    debug_data.can.tx_ok_count = 0U;
    debug_data.can.tx_fail_count = 0U;
    debug_data.can.tx_mailbox_free = 0U;
    debug_data.can.init_ok = 0U;
    for (uint8_t i = 0U; i < 8U; ++i)
    {
        debug_data.can.last_rx_data[i] = 0U;
    }
    for (uint8_t i = 0; i < DEBUG_MOTOR_NUM; i++)
    {
        debug_data.motor[i].debug_target_speed_rpm = 0.0f;
    }
}

void appDebugUpdate(void)
{
    appControlGetDebugData(&debug_data);
}

void appDebugCanState(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL)
    {
        return;
    }

    debug_data.can.state = (uint32_t)HAL_CAN_GetState(hcan);
    debug_data.can.error = (uint32_t)HAL_CAN_GetError(hcan);
    debug_data.can.tx_mailbox_free =
        (uint32_t)HAL_CAN_GetTxMailboxesFreeLevel(hcan);
}

void appDebugCanRx(uint32_t id, const uint8_t *data)
{
    debug_data.can.rx_count++;
    debug_data.can.last_rx_id = id;
    if (data != NULL)
    {
        for (uint8_t i = 0U; i < 8U; ++i)
        {
            debug_data.can.last_rx_data[i] = data[i];
        }
    }
}

void appDebugCanRxError(void)
{
    debug_data.can.rx_error_count++;
}

void appDebugCanTx(uint32_t status)
{
    debug_data.can.tx_count++;
    if (status == 0U)
    {
        debug_data.can.tx_ok_count++;
    }
    else
    {
        debug_data.can.tx_fail_count++;
    }
}

void appDebugCanInitResult(HAL_StatusTypeDef filter_status,
                           HAL_StatusTypeDef start_status,
                           HAL_StatusTypeDef notification_status)
{
    debug_data.can.init_ok =
        (filter_status == HAL_OK && start_status == HAL_OK &&
         notification_status == HAL_OK) ? 1U : 0U;
}
