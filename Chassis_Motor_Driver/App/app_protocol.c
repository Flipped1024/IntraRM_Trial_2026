/* app_protocol.c */
#include "app_protocol.h"

extern CAN_HandleTypeDef hcan;

void appProtocolGetStatusFrame(ProtocolStatusFrame *status_frame_out)
{
    if (status_frame_out == NULL) {
        return;
    }
    status_frame_out->num1_current_speed_ = bspGetEncoderDeltaPulse(1);
    status_frame_out->num2_current_speed_ = bspGetEncoderDeltaPulse(2);
    status_frame_out->num1_current_pos_ = bspGetEncoderTotalPulse(1);
    status_frame_out->num2_current_pos_ = bspGetEncoderTotalPulse(2);
}

void appProtocolParse(uint32_t msg_id, const uint8_t *rx_buffer)
{
    if (rx_buffer == NULL)
    {
        return;
    }

    if (msg_id == CTRL_MSG_ID)
    {
        ProtocolControlFrame ctrl_frame;

        ctrl_frame.num1_target_speed_ = (int16_t)((rx_buffer[0] << 8) | rx_buffer[1]);
        ctrl_frame.num2_target_speed_ = (int16_t)((rx_buffer[2] << 8) | rx_buffer[3]);

        appControlSetTargetSpeed(MOTOR_1, (float)ctrl_frame.num1_target_speed_);
        appControlSetTargetSpeed(MOTOR_2, (float)ctrl_frame.num2_target_speed_);
    }
}

void appProtocolInit(void) {
    bspCanRegisterCallback(appProtocolParse);
    bspCanInit(&hcan);
}

void appProtocolReport(CAN_HandleTypeDef *hcan, const ProtocolStatusFrame *status_frame)
{
    if (hcan == NULL || status_frame == NULL)
    {
        return;
    }

    uint8_t tx_buffer_speed[8] = {0};
    uint8_t tx_buffer_pos[8] = {0};

    tx_buffer_speed[0] = (uint8_t)((status_frame->num1_current_speed_ >> 8) & 0xFF);
    tx_buffer_speed[1] = (uint8_t)(status_frame->num1_current_speed_ & 0xFF);
    tx_buffer_speed[2] = (uint8_t)((status_frame->num2_current_speed_ >> 8) & 0xFF);
    tx_buffer_speed[3] = (uint8_t)(status_frame->num2_current_speed_ & 0xFF);

    /* Queue each status frame independently. A failed speed-frame enqueue
       must not suppress the position-frame attempt in this control period. */
    (void)bspCanTransmit(hcan, REPORT_SPEED_MSG_ID, tx_buffer_speed);

    tx_buffer_pos[0] = (uint8_t)((status_frame->num1_current_pos_ >> 24) & 0xFF);
    tx_buffer_pos[1] = (uint8_t)((status_frame->num1_current_pos_ >> 16) & 0xFF);
    tx_buffer_pos[2] = (uint8_t)((status_frame->num1_current_pos_ >> 8) & 0xFF);
    tx_buffer_pos[3] = (uint8_t)(status_frame->num1_current_pos_ & 0xFF);

    tx_buffer_pos[4] = (uint8_t)((status_frame->num2_current_pos_ >> 24) & 0xFF);
    tx_buffer_pos[5] = (uint8_t)((status_frame->num2_current_pos_ >> 16) & 0xFF);
    tx_buffer_pos[6] = (uint8_t)((status_frame->num2_current_pos_ >> 8) & 0xFF);
    tx_buffer_pos[7] = (uint8_t)(status_frame->num2_current_pos_ & 0xFF);

    (void)bspCanTransmit(hcan, REPORT_POS_MSG_ID, tx_buffer_pos);
}
