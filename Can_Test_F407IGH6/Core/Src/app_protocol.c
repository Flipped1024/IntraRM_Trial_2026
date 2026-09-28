#include "app_protocol.h"
#include "bsp_can.h"
#include "can.h"

volatile APP_DebugData_t debug_data = {
    .test_mode = CAN_TEST_MODE,
    .chassis_motor1_speed = 100,
    .chassis_motor2_speed = -50,
    .gimbal_pitch_target_pos = 0,
    .gimbal_fric_duty = 0,
    .gimbal_feed_duty = 0,
    .yaw_pitch_pitch_target_pos = 0,
    .yaw_pitch_yaw_target_pos = 0,
    .tick_counter = 0,
    .send_flag = 0,
    .tx_id = 0,
    .tx_data = {0},
    .can_error_code = 0,
    .can_state = 0,
    .tx_ok_count = 0,
    .tx_fail_count = 0,
    .rx_count = 0,
    .last_rx_id = 0,
    .test_sequence = 0
};

#if CAN_TEST_MODE == CAN_TEST_MODE_CHASSIS
static void pack_chassis_data(volatile uint8_t *buf, int16_t speed1, int16_t speed2)
{
    buf[0] = (uint8_t)((speed1 >> 8) & 0xFF);
    buf[1] = (uint8_t)(speed1 & 0xFF);
    buf[2] = (uint8_t)((speed2 >> 8) & 0xFF);
    buf[3] = (uint8_t)(speed2 & 0xFF);
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}
#elif CAN_TEST_MODE == CAN_TEST_MODE_GIMBAL
static void pack_gimbal_data(volatile uint8_t *buf, int32_t pitch_target_pos,
                      int16_t fric_duty, int16_t feed_duty)
{
    buf[0] = (uint8_t)((pitch_target_pos >> 24) & 0xFF);
    buf[1] = (uint8_t)((pitch_target_pos >> 16) & 0xFF);
    buf[2] = (uint8_t)((pitch_target_pos >> 8) & 0xFF);
    buf[3] = (uint8_t)(pitch_target_pos & 0xFF);
    buf[4] = (uint8_t)((fric_duty >> 8) & 0xFF);
    buf[5] = (uint8_t)(fric_duty & 0xFF);
    buf[6] = (uint8_t)((feed_duty >> 8) & 0xFF);
    buf[7] = (uint8_t)(feed_duty & 0xFF);
}
#elif CAN_TEST_MODE == CAN_TEST_MODE_YAW_PITCH
static void pack_yaw_pitch_data(volatile uint8_t *buf, int32_t pitch_target_pos,
                                int32_t yaw_target_pos)
{
    uint32_t pitch_raw = (uint32_t)pitch_target_pos;
    uint32_t yaw_raw = (uint32_t)yaw_target_pos;

    buf[0] = (uint8_t)(pitch_raw >> 24);
    buf[1] = (uint8_t)(pitch_raw >> 16);
    buf[2] = (uint8_t)(pitch_raw >> 8);
    buf[3] = (uint8_t)pitch_raw;
    buf[4] = (uint8_t)(yaw_raw >> 24);
    buf[5] = (uint8_t)(yaw_raw >> 16);
    buf[6] = (uint8_t)(yaw_raw >> 8);
    buf[7] = (uint8_t)yaw_raw;
}
#endif

static uint8_t send_frame(uint32_t id, const uint8_t *data)
{
    debug_data.tx_id = id;
    for (uint8_t i = 0; i < CAN_DATA_LEN; i++) debug_data.tx_data[i] = data[i];
    if (BSP_CAN_SendMessage(id, (uint8_t *)data, CAN_DATA_LEN) != 0) {
        debug_data.tx_ok_count++;
        return 1;
    }
    debug_data.tx_fail_count++;
    return 0;
}

void APP_Protocol_Init(void)
{
    BSP_CAN_Init();
    debug_data.tick_counter = 0;
    debug_data.send_flag = 0;
    debug_data.tx_id = 0;
    debug_data.can_error_code = hcan1.ErrorCode;
    debug_data.can_state = hcan1.State;
}

void APP_Protocol_Task(void)
{
    if (debug_data.send_flag == 0)
    {
        return;
    }
    debug_data.send_flag = 0;

#if CAN_TEST_MODE == CAN_TEST_MODE_CHASSIS
    debug_data.tx_id = CHASSIS_CAN_SEND_ID;
    pack_chassis_data(debug_data.tx_data, debug_data.chassis_motor1_speed,
                      debug_data.chassis_motor2_speed);
#elif CAN_TEST_MODE == CAN_TEST_MODE_GIMBAL
    debug_data.tx_id = GIMBAL_CAN_SEND_ID;
    pack_gimbal_data(debug_data.tx_data, debug_data.gimbal_pitch_target_pos,
                     debug_data.gimbal_fric_duty, debug_data.gimbal_feed_duty);
#elif CAN_TEST_MODE == CAN_TEST_MODE_YAW_PITCH
    debug_data.tx_id = YAW_PITCH_CAN_SEND_ID;
    pack_yaw_pitch_data(debug_data.tx_data,
                        debug_data.yaw_pitch_pitch_target_pos,
                        debug_data.yaw_pitch_yaw_target_pos);
#elif CAN_TEST_MODE == CAN_TEST_MODE_LINK
    {
        uint8_t data[CAN_DATA_LEN] = {0};
        uint32_t sequence = debug_data.test_sequence++;
        data[0] = (uint8_t)(sequence >> 24);
        data[1] = (uint8_t)(sequence >> 16);
        data[2] = (uint8_t)(sequence >> 8);
        data[3] = (uint8_t)sequence;
        data[4] = 0x4C; data[5] = 0x34; data[6] = 0x33; data[7] = 0x31;
        send_frame(LINK_CAN_ID_SPEED1, data);
        data[7]++;
        send_frame(LINK_CAN_ID_POS1, data);
        data[7]++;
        send_frame(LINK_CAN_ID_SPEED2, data);
        data[7]++;
        send_frame(LINK_CAN_ID_POS2, data);
    }
    debug_data.can_error_code = hcan1.ErrorCode;
    debug_data.can_state = hcan1.State;
    return;
#endif
    if (send_frame(debug_data.tx_id, (const uint8_t *)debug_data.tx_data) != 0)
    {
        debug_data.tx_ok_count++;
    }
    else
    {
        debug_data.tx_fail_count++;
    }
    debug_data.can_error_code = hcan1.ErrorCode;
    debug_data.can_state = hcan1.State;
}

void APP_Protocol_OnCanRx(uint32_t id, const uint8_t *data)
{
    (void)data;
    debug_data.rx_count++;
    debug_data.last_rx_id = id;
}

void HAL_SYSTICK_Callback(void)
{
    debug_data.tick_counter++;
    if (debug_data.tick_counter >= SEND_PERIOD_MS)
    {
        debug_data.tick_counter = 0;
        debug_data.send_flag = 1;
    }
}
