/* app_protocol.c */
#include "app_protocol.h"

extern CAN_HandleTypeDef hcan;

static int32_t protocolReadInt32Be(const uint8_t *data)
{
    uint32_t value = ((uint32_t)data[0] << 24) |
                     ((uint32_t)data[1] << 16) |
                     ((uint32_t)data[2] << 8) |
                     (uint32_t)data[3];
    return (int32_t)value;
}

static void protocolWriteInt16Be(uint8_t *data, int16_t value)
{
    uint16_t raw = (uint16_t)value;
    data[0] = (uint8_t)(raw >> 8);
    data[1] = (uint8_t)raw;
}

static void protocolWriteInt32Be(uint8_t *data, int32_t value)
{
    uint32_t raw = (uint32_t)value;
    data[0] = (uint8_t)(raw >> 24);
    data[1] = (uint8_t)(raw >> 16);
    data[2] = (uint8_t)(raw >> 8);
    data[3] = (uint8_t)raw;
}

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

        ctrl_frame.pitch_target_position_ = protocolReadInt32Be(&rx_buffer[0]);
        ctrl_frame.yaw_target_position_ = protocolReadInt32Be(&rx_buffer[4]);

        appControlSetTargetPosition(MOTOR_PITCH, ctrl_frame.pitch_target_position_);
        appControlSetTargetPosition(MOTOR_YAW, ctrl_frame.yaw_target_position_);
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

    protocolWriteInt16Be(&tx_buffer_speed[0], status_frame->num1_current_speed_);
    protocolWriteInt16Be(&tx_buffer_speed[2], status_frame->num2_current_speed_);

    if (bspCanTransmit(hcan, REPORT_SPEED_MSG_ID, tx_buffer_speed) != CAN_STATUS_OK)
    {
        return;
    }

    protocolWriteInt32Be(&tx_buffer_pos[0], status_frame->num1_current_pos_);
    protocolWriteInt32Be(&tx_buffer_pos[4], status_frame->num2_current_pos_);

    bspCanTransmit(hcan, REPORT_POS_MSG_ID, tx_buffer_pos);
}
