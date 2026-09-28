#ifndef __APP_PROTOCOL_H
#define __APP_PROTOCOL_H

#include "main.h"
#include "bsp_can.h"

typedef struct {
    int16_t motor1_speed_;
    int16_t motor2_speed_;
} MotorCmd;

typedef struct {
    int16_t motor1_delta_pulse_;
    int16_t motor2_delta_pulse_;
    int32_t motor1_pos_;
    int32_t motor2_pos_;
} MotorFeedback;

void protocolPackMotorCmd(const MotorCmd *cmd, uint8_t *data);
void protocolUnpackSpeedFeedback(const uint8_t *data, MotorFeedback *fb);
void protocolUnpackPosFeedback(const uint8_t *data, MotorFeedback *fb);
HAL_StatusTypeDef protocolSendBoardCmd(uint8_t board_id, const MotorCmd *cmd);
void protocolProcessCanMessage(uint32_t can_id, const uint8_t *data,
                               MotorFeedback *fb_board1, MotorFeedback *fb_board2);

#endif /* __APP_PROTOCOL_H */
