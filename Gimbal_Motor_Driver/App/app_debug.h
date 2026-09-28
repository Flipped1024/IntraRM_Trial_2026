#ifndef APP_DEBUG_H
#define APP_DEBUG_H

#include <stdint.h>

typedef struct {
    int32_t fric1_duty;
    int32_t fric2_duty;
    int32_t feed_duty;

    uint8_t debug_mode;

    /* CAN receive diagnostics; inspect these fields in Ozone Watch. */
    uint32_t can_init_status;
    uint32_t can_start_status;
    uint32_t can_notification_status;
    uint32_t can_error_code;
    uint32_t can_rx_count;
    uint32_t can_callback_count;
    uint32_t can_ctrl_count;
    uint32_t can_last_rx_tick;
    uint32_t can_last_ctrl_tick;
    uint32_t can_last_id;
    uint32_t can_last_dlc;
    uint8_t  can_last_data[8];
    int16_t  can_last_fric_duty;
    int16_t  can_last_feed_duty;
} DebugData;

extern volatile DebugData debug_data;

#endif /* APP_DEBUG_H */
