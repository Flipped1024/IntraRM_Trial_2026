#ifndef __APP_DEBUG_H
#define __APP_DEBUG_H

#include "main.h"
#include "can.h"
#include "app_chassis.h"

typedef enum {
    DEBUG_MODE_NONE    = 0,
    DEBUG_MODE_CHASSIS = 1,
    DEBUG_MODE_CAN     = 2,
} DebugMode;

typedef struct {
    volatile float vx_;
    volatile float vy_;
    volatile float vw_;
} DebugChassis;

typedef struct {
    volatile int32_t pitch_target_pos_;
    volatile int32_t pitch_current_pos_;
    volatile int16_t pitch_current_speed_;
    volatile int16_t pitch_target_duty_;
    volatile int32_t yaw_target_pos_;
    volatile int32_t yaw_current_pos_;
    volatile int32_t yaw_center_pos_;
    volatile int16_t yaw_current_speed_;
    volatile int16_t yaw_target_duty_;
    volatile uint32_t yaw_tx_status_;
    volatile int8_t right_stick_horizontal_;
    volatile int8_t right_stick_vertical_;
    volatile uint8_t state_;
    volatile uint8_t yaw_feedback_valid_;
    volatile uint8_t friction_wheels_enabled_;
    volatile uint8_t fire_active_;
    volatile uint16_t fric_duty_;
    volatile uint16_t feed_duty_;
    volatile int16_t pitch_open_loop_duty_;
    volatile int16_t yaw_open_loop_duty_;
    volatile int16_t pitch_brake_duty_;
    volatile int16_t yaw_brake_duty_; /* Reserved: unused in Yaw open-loop mode. */
    volatile uint8_t pseudo_closed_loop_enable_; /* Pitch only; Yaw is always open-loop. */
    volatile uint32_t pseudo_drive_ms_;
    volatile uint32_t pseudo_brake_ms_;
    volatile float pseudo_brake_ratio_; /* Reserved: braking uses pitch_brake_duty_. */
    volatile uint8_t pitch_pseudo_active_;
    volatile uint8_t pitch_pseudo_brake_phase_;
    volatile uint8_t yaw_pseudo_active_; /* Always zero. */
    volatile uint8_t yaw_pseudo_brake_phase_; /* Always zero. */
} DebugGimbal;

typedef struct {
    /* All four-element motor arrays: [FL, RL, RR, FR], in CAN board/slot order. */
    volatile int16_t motor_rpm_[4];
    volatile int16_t motor_delta_pulse_[4];
    volatile float motor_speed_rpm_[4];
    volatile int32_t motor_position_[4];
    volatile uint32_t tx_count_;
    volatile uint32_t tx_ok_count_;
    volatile uint32_t tx_fail_count_;
    volatile uint32_t rx_count_;
    volatile uint32_t last_rx_id_;
    volatile uint8_t  last_rx_data_[8];
    volatile uint32_t can_state_;
    volatile uint32_t can_error_;
    volatile uint32_t can_tx_mailbox_free_;
    volatile uint8_t  can_init_ok_;
} DebugCan;

typedef struct {
    volatile uint8_t initialized_;
    volatile uint8_t online_;
    volatile uint8_t data_ready_;
    volatile uint8_t calibration_ready_;
    volatile uint8_t yaw_axis_;
    volatile int8_t yaw_axis_sign_;
    volatile uint8_t who_am_i_;
    volatile uint8_t gyro_config_reg_;
    volatile uint8_t accel_config_reg_;
    volatile int16_t gyro_peak_raw_[3];
    volatile int16_t gyro_raw_x_;
    volatile int16_t gyro_raw_y_;
    volatile int16_t gyro_raw_z_;
    volatile float gyro_z_dps_;
    volatile uint8_t i2c_error_;
    volatile uint32_t sample_count_;
    volatile uint32_t error_count_;
    volatile int16_t accel_raw_[3];
    volatile int16_t gyro_raw_[3];
    volatile int16_t temp_raw_;
    volatile float accel_g_[3];
    volatile float gyro_dps_[3];
    volatile float gyro_integral_deg_[3];
    volatile float temperature_c_;
    volatile float roll_deg_;
    volatile float pitch_deg_;
    volatile float yaw_deg_;
    volatile float yaw_euler_deg_;
    volatile float yaw_rate_dps_;
    volatile float dt_s_;
    volatile float yaw_bias_dps_;
    volatile float quat_[4];
    volatile float accel_norm_g_;
    volatile uint16_t heater_pwm_;
    volatile float heater_temp_target_c_;
    volatile float heater_pid_output_;
} DebugImu;

typedef struct {
    volatile DebugMode mode_;
    DebugChassis       chassis_;
    DebugGimbal        gimbal_;
    DebugCan           can_;
    DebugImu           imu_;
} DebugData;

extern DebugData g_debug_;

void debugInit(void);
void debugUpdateCanTx(HAL_StatusTypeDef status);
void debugUpdateCanRx(uint32_t can_id, const uint8_t *data);
void debugUpdateCanState(void);
void debugUpdateGimbal(int32_t pitch_target, int32_t pitch_current,
                       int16_t pitch_speed, int32_t yaw_target,
                       int32_t yaw_current, int16_t yaw_speed,
                       int32_t yaw_center, uint8_t gimbal_state, uint8_t yaw_feedback_valid,
                       uint8_t friction_enabled, uint8_t fire_active,
                       uint16_t fric_duty, uint16_t feed_duty,
                       int8_t stick_horizontal, int8_t stick_vertical);
void debugRunChassis(Chassis *chassis);
void debugRunCanDirect(void);

#endif /* __APP_DEBUG_H */
