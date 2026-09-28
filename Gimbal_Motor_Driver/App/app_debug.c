#include "app_debug.h"
#include "stm32f1xx_hal.h"

volatile DebugData debug_data = {
    .fric1_duty = 0,
    .fric2_duty = 0,
    .feed_duty = 0,

    .debug_mode = 0,

    .can_init_status = HAL_OK,
    .can_start_status = HAL_OK,
    .can_notification_status = HAL_OK,
    .can_error_code = 0,
    .can_rx_count = 0,
    .can_callback_count = 0,
    .can_ctrl_count = 0,
    .can_last_rx_tick = 0,
    .can_last_ctrl_tick = 0,
    .can_last_id = 0,
    .can_last_dlc = 0,
    .can_last_data = {0},
    .can_last_fric_duty = 0,
    .can_last_feed_duty = 0
};
