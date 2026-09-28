#ifndef __APP_GIMBAL_H
#define __APP_GIMBAL_H

#include "main.h"
#include "app_remote.h"

#define GIMBAL_CAN_CMD_ID       0x113
#define GIMBAL_CAN_FEEDBACK_ID  0x206

#define GIMBAL_FRIC_DUTY_ON     800
#define GIMBAL_FEED_DUTY_ON     600
#define GIMBAL_FRIC_DUTY_OFF    0
#define GIMBAL_FEED_DUTY_OFF    0

#define GIMBAL_CALIB_SPEED      100
#define GIMBAL_CALIB_STALL_MS   500
#define GIMBAL_CALIB_STALL_THRESHOLD  5
#define GIMBAL_CALIB_SETTLE_MS  300

#define GIMBAL_PITCH_MAX_ANGLE  3000
#define GIMBAL_PITCH_STICK_RANGE 660.0f

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
    int32_t pitch_current_pos_;
    int16_t pitch_current_speed_;

    uint8_t fric_feed_on_;
    uint16_t fric_duty_;
    uint16_t feed_duty_;

    uint8_t switch_left_prev_;
    uint8_t switch_left_toggle_;

    float pitch_sensitivity_;
} Gimbal;

extern Gimbal gimbal_;

void gimbalInit(void);
void gimbalUpdate(void);
void gimbalSendCmd(void);
void gimbalProcessFeedback(uint32_t can_id, const uint8_t *data);

#endif /* __APP_GIMBAL_H */