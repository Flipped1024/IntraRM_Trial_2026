#ifndef __APP_GIMBAL_H
#define __APP_GIMBAL_H

#include "main.h"
#include "app_remote.h"

#define GIMBAL_CAN_CMD_ID       0x113
#define YAW_PITCH_CAN_CMD_ID    0x114
#define GIMBAL_CAN_FEEDBACK_ID  0x206
#define GIMBAL_CAN_POSITION_FEEDBACK_ID 0x207
#define GIMBAL_CAN_YAW_PITCH_SPEED_ID 0x208
#define GIMBAL_CAN_YAW_PITCH_POS_ID 0x209

#define GIMBAL_FRIC_DUTY_ON     0
#define GIMBAL_FEED_DUTY_ON     999
#define GIMBAL_FRIC_DUTY_OFF    999
#define GIMBAL_FEED_DUTY_OFF    0

#define GIMBAL_CALIB_SPEED      100
#define GIMBAL_CALIB_STALL_MS   500
#define GIMBAL_CALIB_STALL_THRESHOLD  5
#define GIMBAL_CALIB_SETTLE_MS  300

#define GIMBAL_PITCH_MAX_ANGLE  3000
#define GIMBAL_PITCH_STICK_RANGE 660.0f
#define GIMBAL_YAW_PULSES_PER_REV (11.0f * 4.0f * 45.0f)
#define GIMBAL_YAW_MAX_LEAD_DEG 1.0f
#define GIMBAL_YAW_STICK_RANGE 127.0f
#define GIMBAL_YAW_MAX_SPEED_RPM 70
#define GIMBAL_PITCH_MAX_SPEED_RPM 70
#define GIMBAL_YAW_OPEN_LOOP_DUTY 999
#define GIMBAL_PITCH_OPEN_LOOP_DUTY 999
#define GIMBAL_YAW_MAX_ANGLE_DEG 100.0f
#define GIMBAL_PITCH_STICK_DEADBAND 80
#define GIMBAL_YAW_STICK_DEADBAND 80
#define GIMBAL_YAW_FEEDBACK_TIMEOUT_MS 100U

typedef enum {
    GIMBAL_STATE_CALIB_UP = 0,
    GIMBAL_STATE_CALIB_DOWN = 1,
    GIMBAL_STATE_CALIB_CENTER = 2,
    GIMBAL_STATE_READY = 3,
} GimbalState;

typedef struct {
    GimbalState state_;

    int32_t calib_up_pos_;
    int32_t calib_down_pos_;
    int32_t calib_zero_pos_;
    int32_t calib_range_;
    uint32_t calib_stall_start_;
    int32_t calib_last_pos_;
    uint8_t calib_stall_detected_;

    int32_t pitch_target_pos_;
    int16_t pitch_target_duty_;
    int32_t pitch_current_pos_;
    int16_t pitch_current_speed_;
    int32_t yaw_target_pos_;
    int16_t yaw_target_duty_;
    int32_t yaw_current_pos_;
    int16_t yaw_current_speed_;
    int32_t yaw_center_pos_;
    uint8_t yaw_feedback_valid_;
    uint32_t yaw_feedback_tick_;

    uint8_t friction_wheels_enabled_;
    uint8_t fire_active_;
    uint16_t fric_duty_;
    uint16_t feed_duty_;

    uint8_t lb_was_pressed_;
    uint8_t lb_toggle_request_;
    uint8_t a_was_pressed_;
    uint8_t a_toggle_request_;

    float pitch_sensitivity_;
} Gimbal;

extern Gimbal gimbal_;

void gimbalInit(void);
void gimbalUpdate(void);
void gimbalEmergencyStop(void);
void gimbalSendCmd(void);
void gimbalSendYawPitchCmd(void);
void gimbalProcessFeedback(uint32_t can_id, const uint8_t *data);

#endif /* __APP_GIMBAL_H */
