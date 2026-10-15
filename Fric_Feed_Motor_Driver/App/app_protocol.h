#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H

#include <stdint.h>
#include "bsp_can.h"

#define CTRL_MSG_ID 0x113U
#define REPORT_STATUS_MSG_ID 0x206U

typedef struct {
    int16_t fric_duty_;
    int16_t feed_duty_;
} ProtocolControlFrame;

typedef struct {
    uint8_t reserved_[8];
} ProtocolStatusFrame;

void appProtocolParse(uint32_t msg_id, const uint8_t *rx_buffer);
void appProtocolInit(void);
void appProtocolReport(CAN_HandleTypeDef *hcan, const ProtocolStatusFrame *status_frame);
void appProtocolGetStatusFrame(ProtocolStatusFrame *status_frame_out);

#endif /* APP_PROTOCOL_H */
