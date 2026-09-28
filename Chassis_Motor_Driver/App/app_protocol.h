/* app_protocol.h */
#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include "board_config.h"
#include "bsp_can.h"
#include "bsp_encoder.h"
#include "app_control.h"

typedef struct {
    int16_t num1_target_speed_;
    int16_t num2_target_speed_;
} ProtocolControlFrame;

typedef struct {
    int16_t num1_current_speed_;
    int16_t num2_current_speed_;
    int32_t num1_current_pos_;
    int32_t num2_current_pos_;
} ProtocolStatusFrame;

void appProtocolParse(uint32_t msg_id, const uint8_t *rx_buffer);
void appProtocolInit(void);
void appProtocolReport(CAN_HandleTypeDef *hcan, const ProtocolStatusFrame *status_frame);

void appProtocolGetStatusFrame(ProtocolStatusFrame *status_frame_out);

#endif /* APP_PROTOCOL_H */
