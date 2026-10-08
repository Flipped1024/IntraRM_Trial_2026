#include "app_debug.h"
#include "bsp_can.h"
#include "app_protocol.h"

DebugData g_debug_;

void debugInit(void)
{
    uint8_t i;

    g_debug_.mode_ = DEBUG_MODE_NONE;

    g_debug_.chassis_.vx_ = 0.0f;
    g_debug_.chassis_.vy_ = 0.0f;
    g_debug_.chassis_.vw_ = 0.0f;
    g_debug_.gimbal_.pitch_target_pos_ = 0;
    g_debug_.gimbal_.pitch_current_pos_ = 0;
    g_debug_.gimbal_.pitch_current_speed_ = 0;
    g_debug_.gimbal_.yaw_target_pos_ = 0;
    g_debug_.gimbal_.yaw_current_pos_ = 0;
    g_debug_.gimbal_.yaw_center_pos_ = 0;
    g_debug_.gimbal_.yaw_current_speed_ = 0;
    g_debug_.gimbal_.right_stick_horizontal_ = 0;
    g_debug_.gimbal_.right_stick_vertical_ = 0;
    g_debug_.gimbal_.state_ = 0;
    g_debug_.gimbal_.yaw_feedback_valid_ = 0;
    g_debug_.gimbal_.friction_wheels_enabled_ = 0;
    g_debug_.gimbal_.fire_active_ = 0;
    g_debug_.gimbal_.fric_duty_ = 999;
    g_debug_.gimbal_.feed_duty_ = 0;

    for (i = 0; i < 4; i++) {
        g_debug_.can_.motor_rpm_[i] = 0;
        g_debug_.can_.motor_delta_pulse_[i] = 0;
        g_debug_.can_.motor_speed_rpm_[i] = 0.0f;
        g_debug_.can_.motor_position_[i] = 0;
    }
    g_debug_.can_.tx_count_           = 0;
    g_debug_.can_.tx_ok_count_        = 0;
    g_debug_.can_.tx_fail_count_      = 0;
    g_debug_.can_.rx_count_           = 0;
    g_debug_.can_.last_rx_id_         = 0;
    for (i = 0; i < 8; i++) {
        g_debug_.can_.last_rx_data_[i] = 0;
    }
    g_debug_.can_.can_state_          = 0;
    g_debug_.can_.can_error_          = 0;
    g_debug_.can_.can_tx_mailbox_free_ = 0;
    g_debug_.can_.can_init_ok_        = 0;
    g_debug_.imu_.initialized_ = 0;
    g_debug_.imu_.online_ = 0;
    g_debug_.imu_.data_ready_ = 0;
    g_debug_.imu_.calibration_ready_ = 0;
    g_debug_.imu_.yaw_axis_ = 2U;
    g_debug_.imu_.yaw_axis_sign_ = 1;
    g_debug_.imu_.who_am_i_ = 0;
    g_debug_.imu_.gyro_config_reg_ = 0;
    g_debug_.imu_.accel_config_reg_ = 0;
    for (i = 0; i < 3; i++) g_debug_.imu_.gyro_peak_raw_[i] = 0;
    g_debug_.imu_.gyro_raw_x_ = 0;
    g_debug_.imu_.gyro_raw_y_ = 0;
    g_debug_.imu_.gyro_raw_z_ = 0;
    g_debug_.imu_.gyro_z_dps_ = 0.0f;
    g_debug_.imu_.i2c_error_ = 0;
    g_debug_.imu_.sample_count_ = 0;
    g_debug_.imu_.error_count_ = 0;
    g_debug_.imu_.temp_raw_ = 0;
    g_debug_.imu_.temperature_c_ = 0.0f;
    g_debug_.imu_.roll_deg_ = 0.0f;
    g_debug_.imu_.pitch_deg_ = 0.0f;
    g_debug_.imu_.yaw_deg_ = 0.0f;
    g_debug_.imu_.yaw_euler_deg_ = 0.0f;
    g_debug_.imu_.yaw_rate_dps_ = 0.0f;
    g_debug_.imu_.dt_s_ = 0.0f;
    g_debug_.imu_.yaw_bias_dps_ = 0.0f;
    g_debug_.imu_.quat_[0] = 1.0f;
    g_debug_.imu_.quat_[1] = 0.0f;
    g_debug_.imu_.quat_[2] = 0.0f;
    g_debug_.imu_.quat_[3] = 0.0f;
    g_debug_.imu_.accel_norm_g_ = 0.0f;
    g_debug_.imu_.heater_pwm_ = 0;
    g_debug_.imu_.heater_temp_target_c_ = 40.0f;
    g_debug_.imu_.heater_pid_output_ = 0.0f;
    for (i = 0; i < 3; i++) {
        g_debug_.imu_.accel_raw_[i] = 0;
        g_debug_.imu_.gyro_raw_[i] = 0;
        g_debug_.imu_.accel_g_[i] = 0.0f;
        g_debug_.imu_.gyro_dps_[i] = 0.0f;
        g_debug_.imu_.gyro_integral_deg_[i] = 0.0f;
    }
}

void debugUpdateCanTx(HAL_StatusTypeDef status)
{
    g_debug_.can_.tx_count_++;
    if (status == HAL_OK) {
        g_debug_.can_.tx_ok_count_++;
    } else {
        g_debug_.can_.tx_fail_count_++;
    }
}

void debugUpdateCanRx(uint32_t can_id, const uint8_t *data)
{
    uint8_t i;

    g_debug_.can_.rx_count_++;
    g_debug_.can_.last_rx_id_ = can_id;
    for (i = 0; i < 8; i++) {
        g_debug_.can_.last_rx_data_[i] = data[i];
    }
}

void debugUpdateCanState(void)
{
    g_debug_.can_.can_state_          = (uint32_t)HAL_CAN_GetState(&hcan1);
    g_debug_.can_.can_error_          = (uint32_t)HAL_CAN_GetError(&hcan1);
    g_debug_.can_.can_tx_mailbox_free_ = (uint32_t)HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
}

void debugUpdateGimbal(int32_t pitch_target, int32_t pitch_current,
                       int16_t pitch_speed, int32_t yaw_target,
                       int32_t yaw_current, int16_t yaw_speed,
                       int32_t yaw_center, uint8_t gimbal_state, uint8_t yaw_feedback_valid,
                       uint8_t friction_enabled, uint8_t fire_active,
                       uint16_t fric_duty, uint16_t feed_duty,
                       int8_t stick_horizontal, int8_t stick_vertical)
{
    g_debug_.gimbal_.pitch_target_pos_ = pitch_target;
    g_debug_.gimbal_.pitch_current_pos_ = pitch_current;
    g_debug_.gimbal_.pitch_current_speed_ = pitch_speed;
    g_debug_.gimbal_.yaw_target_pos_ = yaw_target;
    g_debug_.gimbal_.yaw_current_pos_ = yaw_current;
    g_debug_.gimbal_.yaw_current_speed_ = yaw_speed;
    g_debug_.gimbal_.yaw_center_pos_ = yaw_center;
    g_debug_.gimbal_.state_ = gimbal_state;
    g_debug_.gimbal_.yaw_feedback_valid_ = yaw_feedback_valid;
    g_debug_.gimbal_.friction_wheels_enabled_ = friction_enabled;
    g_debug_.gimbal_.fire_active_ = fire_active;
    g_debug_.gimbal_.fric_duty_ = fric_duty;
    g_debug_.gimbal_.feed_duty_ = feed_duty;
    g_debug_.gimbal_.right_stick_horizontal_ = stick_horizontal;
    g_debug_.gimbal_.right_stick_vertical_ = stick_vertical;
}

void debugRunChassis(Chassis *chassis)
{
    HAL_StatusTypeDef status;

    chassisSetTarget(chassis,
                     g_debug_.chassis_.vx_,
                     g_debug_.chassis_.vy_,
                     g_debug_.chassis_.vw_);
    chassisUpdate(chassis);

    status = protocolSendBoardCmd(1, &chassis->motor_cmds_[0]);
    debugUpdateCanTx(status);

    status = protocolSendBoardCmd(2, &chassis->motor_cmds_[1]);
    debugUpdateCanTx(status);

    debugUpdateCanState();
}

void debugRunCanDirect(void)
{
    MotorCmd cmd_board1;
    MotorCmd cmd_board2;
    HAL_StatusTypeDef status;

    for (uint8_t i = 0U; i < 4U; ++i) {
        if (g_debug_.can_.motor_rpm_[i] > (int16_t)CHASSIS_MAX_SPEED_RPM) g_debug_.can_.motor_rpm_[i] = (int16_t)CHASSIS_MAX_SPEED_RPM;
        if (g_debug_.can_.motor_rpm_[i] < -(int16_t)CHASSIS_MAX_SPEED_RPM) g_debug_.can_.motor_rpm_[i] = -(int16_t)CHASSIS_MAX_SPEED_RPM;
    }
    cmd_board1.motor1_speed_ = g_debug_.can_.motor_rpm_[0];
    cmd_board1.motor2_speed_ = g_debug_.can_.motor_rpm_[1];
    cmd_board2.motor1_speed_ = g_debug_.can_.motor_rpm_[2];
    cmd_board2.motor2_speed_ = g_debug_.can_.motor_rpm_[3];

    status = protocolSendBoardCmd(1, &cmd_board1);
    debugUpdateCanTx(status);
    debugUpdateCanState();

    status = protocolSendBoardCmd(2, &cmd_board2);
    debugUpdateCanTx(status);
}
