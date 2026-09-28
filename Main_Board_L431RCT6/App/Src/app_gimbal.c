#include "app_gimbal.h"
#include "bsp_can.h"
#include "bsp_tim.h"
#include "stdlib.h"

Gimbal gimbal_;

void gimbalInit(void)
{
    gimbal_.state_ = GIMBAL_STATE_CALIB_UP;

    gimbal_.calib_up_pos_   = 0;
    gimbal_.calib_down_pos_ = 0;
    gimbal_.calib_zero_pos_ = 0;
    gimbal_.calib_range_    = 0;
    gimbal_.calib_stall_start_ = 0;
    gimbal_.calib_last_pos_ = 0;
    gimbal_.calib_stall_detected_ = 0;

    gimbal_.pitch_target_pos_  = 0;
    gimbal_.pitch_current_pos_ = 0;
    gimbal_.pitch_current_speed_ = 0;

    gimbal_.friction_wheels_enabled_ = 0U;
    gimbal_.fire_active_ = 0U;
    gimbal_.fric_duty_    = GIMBAL_FRIC_DUTY_OFF;
    gimbal_.feed_duty_    = GIMBAL_FEED_DUTY_OFF;
    gimbal_.lb_was_pressed_ = 0U;
    gimbal_.lb_toggle_request_ = 0U;

    gimbal_.pitch_sensitivity_ = (float)GIMBAL_PITCH_MAX_ANGLE / GIMBAL_PITCH_STICK_RANGE;
}

static void gimbalProcessLbButton(void)
{
    const uint8_t lb_pressed = controller_state.buttons_.lb_pressed_;

    gimbal_.lb_toggle_request_ =
        (gimbal_.lb_was_pressed_ == 0U && lb_pressed != 0U) ? 1U : 0U;
    gimbal_.lb_was_pressed_ = lb_pressed;
}

static void gimbalUpdateFricFeed(void)
{
    if (controllerDataIsValid() == 0U) {
        gimbal_.friction_wheels_enabled_ = 0U;
        gimbal_.fire_active_ = 0U;
    } else {
        if (gimbal_.lb_toggle_request_ != 0U) {
            gimbal_.friction_wheels_enabled_ =
                (gimbal_.friction_wheels_enabled_ == 0U) ? 1U : 0U;
        }
        gimbal_.fire_active_ = controller_state.fire_requested_;
    }
    gimbal_.lb_toggle_request_ = 0U;

    if (gimbal_.friction_wheels_enabled_ != 0U) {
        gimbal_.fric_duty_ = GIMBAL_FRIC_DUTY_ON;
    } else {
        gimbal_.fric_duty_ = GIMBAL_FRIC_DUTY_OFF;
    }

    if (gimbal_.fire_active_ != 0U) {
        gimbal_.feed_duty_ = GIMBAL_FEED_DUTY_ON;
    } else {
        gimbal_.feed_duty_ = GIMBAL_FEED_DUTY_OFF;
    }
}

static void gimbalCalibUp(void)
{
    uint32_t now = bspTimGetTick();

    gimbal_.pitch_target_pos_ += GIMBAL_CALIB_SPEED;

    if (gimbal_.calib_stall_detected_ == 0) {
        if (abs(gimbal_.pitch_current_speed_) < GIMBAL_CALIB_STALL_THRESHOLD) {
            if (gimbal_.calib_stall_start_ == 0) {
                gimbal_.calib_stall_start_ = now;
            } else if (now - gimbal_.calib_stall_start_ >= GIMBAL_CALIB_STALL_MS) {
                gimbal_.calib_stall_detected_ = 1;
                gimbal_.calib_up_pos_ = gimbal_.pitch_current_pos_;
            }
        } else {
            gimbal_.calib_stall_start_ = 0;
        }
    } else {
        if (now - gimbal_.calib_stall_start_ >=
            GIMBAL_CALIB_STALL_MS + GIMBAL_CALIB_SETTLE_MS) {
            gimbal_.state_ = GIMBAL_STATE_CALIB_DOWN;
            gimbal_.calib_stall_detected_ = 0;
            gimbal_.calib_stall_start_ = 0;
            gimbal_.pitch_target_pos_ = gimbal_.calib_up_pos_;
        }
    }
}

static void gimbalCalibDown(void)
{
    uint32_t now = bspTimGetTick();

    gimbal_.pitch_target_pos_ -= GIMBAL_CALIB_SPEED;

    if (gimbal_.calib_stall_detected_ == 0) {
        if (abs(gimbal_.pitch_current_speed_) < GIMBAL_CALIB_STALL_THRESHOLD) {
            if (gimbal_.calib_stall_start_ == 0) {
                gimbal_.calib_stall_start_ = now;
            } else if (now - gimbal_.calib_stall_start_ >= GIMBAL_CALIB_STALL_MS) {
                gimbal_.calib_stall_detected_ = 1;
                gimbal_.calib_down_pos_ = gimbal_.pitch_current_pos_;
            }
        } else {
            gimbal_.calib_stall_start_ = 0;
        }
    } else {
        if (now - gimbal_.calib_stall_start_ >=
            GIMBAL_CALIB_STALL_MS + GIMBAL_CALIB_SETTLE_MS) {
            gimbal_.calib_zero_pos_ = (gimbal_.calib_up_pos_ + gimbal_.calib_down_pos_) / 2;
            gimbal_.calib_range_ = (gimbal_.calib_up_pos_ - gimbal_.calib_down_pos_) / 2;
            gimbal_.state_ = GIMBAL_STATE_CALIB_CENTER;
            gimbal_.calib_stall_detected_ = 0;
            gimbal_.calib_stall_start_ = 0;
            gimbal_.pitch_target_pos_ = gimbal_.calib_zero_pos_;
        }
    }
}

static void gimbalCalibCenter(void)
{
    if (abs(gimbal_.pitch_current_pos_ - gimbal_.calib_zero_pos_) < 50 &&
        abs(gimbal_.pitch_current_speed_) < GIMBAL_CALIB_STALL_THRESHOLD) {
        gimbal_.state_ = GIMBAL_STATE_READY;
    }
}

static void gimbalNormalUpdate(void)
{
    int32_t stick_offset;

    if (controllerDataIsValid()) {
        stick_offset = (int32_t)((float)controller_state.right_stick_vertical_ *
                                  gimbal_.pitch_sensitivity_);

        if (stick_offset > gimbal_.calib_range_) {
            stick_offset = gimbal_.calib_range_;
        } else if (stick_offset < -gimbal_.calib_range_) {
            stick_offset = -gimbal_.calib_range_;
        }

        gimbal_.pitch_target_pos_ = gimbal_.calib_zero_pos_ + stick_offset;
    }
}

void gimbalUpdate(void)
{
    gimbalProcessLbButton();

    switch (gimbal_.state_) {
    case GIMBAL_STATE_CALIB_UP:
        gimbalCalibUp();
        break;

    case GIMBAL_STATE_CALIB_DOWN:
        gimbalCalibDown();
        break;

    case GIMBAL_STATE_CALIB_CENTER:
        gimbalCalibCenter();
        break;

    case GIMBAL_STATE_READY:
        gimbalNormalUpdate();
        break;

    default:
        break;
    }

    gimbalUpdateFricFeed();
}

void gimbalEmergencyStop(void)
{
    gimbal_.friction_wheels_enabled_ = 0U;
    gimbal_.fire_active_ = 0U;
    gimbal_.fric_duty_ = GIMBAL_FRIC_DUTY_OFF;
    gimbal_.feed_duty_ = GIMBAL_FEED_DUTY_OFF;
    gimbal_.pitch_target_pos_ = gimbal_.pitch_current_pos_;
    gimbal_.lb_was_pressed_ = 0U;
    gimbal_.lb_toggle_request_ = 0U;
}

void gimbalSendCmd(void)
{
    uint8_t tx_data[CAN_FRAME_DATA_LEN];

    tx_data[0] = (uint8_t)(gimbal_.pitch_target_pos_ & 0xFF);
    tx_data[1] = (uint8_t)((gimbal_.pitch_target_pos_ >> 8) & 0xFF);
    tx_data[2] = (uint8_t)((gimbal_.pitch_target_pos_ >> 16) & 0xFF);
    tx_data[3] = (uint8_t)((gimbal_.pitch_target_pos_ >> 24) & 0xFF);

    tx_data[4] = (uint8_t)(gimbal_.fric_duty_ & 0xFF);
    tx_data[5] = (uint8_t)((gimbal_.fric_duty_ >> 8) & 0xFF);

    tx_data[6] = (uint8_t)(gimbal_.feed_duty_ & 0xFF);
    tx_data[7] = (uint8_t)((gimbal_.feed_duty_ >> 8) & 0xFF);

    canSend(GIMBAL_CAN_CMD_ID, tx_data, CAN_FRAME_DATA_LEN);
}

void gimbalProcessFeedback(uint32_t can_id, const uint8_t *data)
{
    if (can_id != GIMBAL_CAN_FEEDBACK_ID) {
        return;
    }

    gimbal_.pitch_current_speed_ = (int16_t)(data[0] | (data[1] << 8));
    gimbal_.pitch_current_pos_   = (int32_t)(data[2] | (data[3] << 8) |
                                             (data[4] << 16) | (data[5] << 24));
}
