#include "app_protocol.h"

void protocolPackMotorCmd(const MotorCmd *cmd, uint8_t *data)
{
    data[0] = (uint8_t)(cmd->motor1_speed_ & 0xFF);
    data[1] = (uint8_t)((cmd->motor1_speed_ >> 8) & 0xFF);
    data[2] = (uint8_t)(cmd->motor2_speed_ & 0xFF);
    data[3] = (uint8_t)((cmd->motor2_speed_ >> 8) & 0xFF);
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
}

void protocolUnpackSpeedFeedback(const uint8_t *data, MotorFeedback *fb)
{
    fb->motor1_speed_ = (int16_t)(data[0] | (data[1] << 8));
    fb->motor2_speed_ = (int16_t)(data[2] | (data[3] << 8));
}

void protocolUnpackPosFeedback(const uint8_t *data, MotorFeedback *fb)
{
    fb->motor1_pos_ = (int32_t)(data[0] | (data[1] << 8) |
                                (data[2] << 16) | (data[3] << 24));
    fb->motor2_pos_ = (int32_t)(data[4] | (data[5] << 8) |
                                (data[6] << 16) | (data[7] << 24));
}

HAL_StatusTypeDef protocolSendBoardCmd(uint8_t board_id, const MotorCmd *cmd)
{
    uint8_t tx_data[CAN_FRAME_DATA_LEN];
    uint32_t can_id;

    protocolPackMotorCmd(cmd, tx_data);

    if (board_id == 1) {
        can_id = CAN_ID_BOARD1_CMD;
    } else {
        can_id = CAN_ID_BOARD2_CMD;
    }

    return canSend(can_id, tx_data, CAN_FRAME_DATA_LEN);
}

void protocolProcessCanMessage(uint32_t can_id, const uint8_t *data,
                               MotorFeedback *fb_board1, MotorFeedback *fb_board2)
{
    switch (can_id) {
    case CAN_ID_BOARD1_SPEED:
        protocolUnpackSpeedFeedback(data, fb_board1);
        break;
    case CAN_ID_BOARD1_POS:
        protocolUnpackPosFeedback(data, fb_board1);
        break;
    case CAN_ID_BOARD2_SPEED:
        protocolUnpackSpeedFeedback(data, fb_board2);
        break;
    case CAN_ID_BOARD2_POS:
        protocolUnpackPosFeedback(data, fb_board2);
        break;
    default:
        break;
    }
}