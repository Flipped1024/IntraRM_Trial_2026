#include "app_gimbal.h"
#include "bsp_can.h"
#include "bsp_tim.h"
#include "app_debug.h"
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
    gimbal_.pitch_target_duty_ = 0;
    gimbal_.pitch_current_pos_ = 0;
    gimbal_.pitch_current_speed_ = 0;
    gimbal_.yaw_target_pos_ = 0;
    gimbal_.yaw_target_duty_ = 0;
    gimbal_.yaw_current_pos_ = 0;
    gimbal_.yaw_current_speed_ = 0;
    gimbal_.yaw_center_pos_ = 0;
    gimbal_.yaw_feedback_valid_ = 0U;
    gimbal_.yaw_feedback_tick_ = 0U;

    gimbal_.friction_wheels_enabled_ = 0U;
    gimbal_.fire_active_ = 0U;
    gimbal_.fric_duty_    = GIMBAL_FRIC_DUTY_OFF;
    gimbal_.feed_duty_    = GIMBAL_FEED_DUTY_OFF;
    gimbal_.lb_was_pressed_ = 0U;
    gimbal_.lb_toggle_request_ = 0U;
    gimbal_.a_was_pressed_ = 0U;
    gimbal_.a_toggle_request_ = 0U;

    gimbal_.pitch_sensitivity_ = (float)GIMBAL_PITCH_MAX_ANGLE / GIMBAL_PITCH_STICK_RANGE;
}

static void gimbalProcessLbButton(void)
{
    const uint8_t a_pressed = controller_state.buttons_.a_pressed_;

    gimbal_.a_toggle_request_ =
        (gimbal_.a_was_pressed_ == 0U && a_pressed != 0U) ? 1U : 0U;
    gimbal_.a_was_pressed_ = a_pressed;
}

static void gimbalUpdateFricFeed(void)
{
    if (controllerDataIsValid() == 0U) {
        gimbal_.friction_wheels_enabled_ = 0U;
        gimbal_.fire_active_ = 0U;
    } else {
        if (gimbal_.a_toggle_request_ != 0U) {
            gimbal_.friction_wheels_enabled_ =
                (gimbal_.friction_wheels_enabled_ == 0U) ? 1U : 0U;
        }
        gimbal_.fire_active_ = controller_state.fire_requested_;
    }
    gimbal_.a_toggle_request_ = 0U;

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

static void gimbalPitchSpeedUpdate(void)
{
    int32_t stick = controller_state.right_stick_vertical_;
    if (controllerDataIsValid() == 0U || abs(stick) <= GIMBAL_PITCH_STICK_DEADBAND) {
        gimbal_.pitch_target_duty_ = 0;
        return;
    }
    if (stick > 127) stick = 127;
    if (stick < -127) stick = -127;
    gimbal_.pitch_target_duty_ = (stick > 0) ? GIMBAL_PITCH_OPEN_LOOP_DUTY : -GIMBAL_PITCH_OPEN_LOOP_DUTY;
}

/* Bounded target around the first valid position after power-up. Release holds
 * the last target; the stick does not command an automatic return-to-center. */
static void gimbalYawUpdate(void)
{
    int32_t stick = controller_state.right_stick_horizontal_;
    if (controllerDataIsValid() == 0U) {
        gimbal_.yaw_target_duty_ = 0;
        return;
    }
    if (abs(stick) <= GIMBAL_YAW_STICK_DEADBAND) {
        gimbal_.yaw_target_duty_ = 0;
        return;
    }
    if (stick > 127) stick = 127;
    if (stick < -127) stick = -127;
    /* Negative encoder direction is clockwise on the current vehicle. */
    gimbal_.yaw_target_duty_ = (stick > 0) ? -GIMBAL_YAW_OPEN_LOOP_DUTY : GIMBAL_YAW_OPEN_LOOP_DUTY;
}

void gimbalUpdate(void)
{
    gimbalProcessLbButton();
    gimbalPitchSpeedUpdate();
    gimbalYawUpdate();
    gimbalUpdateFricFeed();
    debugUpdateGimbal(gimbal_.pitch_target_pos_, gimbal_.pitch_current_pos_,
                      gimbal_.pitch_current_speed_, gimbal_.yaw_target_pos_,
                      gimbal_.yaw_current_pos_, gimbal_.yaw_current_speed_,
                      gimbal_.yaw_center_pos_,
                      (uint8_t)gimbal_.state_, gimbal_.yaw_feedback_valid_,
                      gimbal_.friction_wheels_enabled_, gimbal_.fire_active_,
                      gimbal_.fric_duty_, gimbal_.feed_duty_,
                      controller_state.right_stick_horizontal_,
                      controller_state.right_stick_vertical_);
}

void gimbalEmergencyStop(void)
{
    gimbal_.pitch_target_duty_ = 0;
    gimbal_.yaw_target_duty_ = 0;
    gimbal_.friction_wheels_enabled_ = 0U;
    gimbal_.fire_active_ = 0U;
    gimbal_.fric_duty_ = GIMBAL_FRIC_DUTY_OFF;
    gimbal_.feed_duty_ = GIMBAL_FEED_DUTY_OFF;
    gimbal_.pitch_target_pos_ = gimbal_.pitch_current_pos_;
    gimbal_.yaw_target_pos_ = gimbal_.yaw_current_pos_;
    gimbal_.yaw_center_pos_ = gimbal_.yaw_current_pos_;
    gimbal_.lb_was_pressed_ = 0U;
    gimbal_.lb_toggle_request_ = 0U;
    gimbal_.a_was_pressed_ = 0U;
    gimbal_.a_toggle_request_ = 0U;
}

void gimbalSendCmd(void)
{
    uint8_t tx_data[CAN_FRAME_DATA_LEN];

    /* 0x113 闁告瑦鍨甸惃鐘诲级閸喖绀傞悹渚囧枙鐠愮喐寰勮椤忣剟鏁嶅顐ょ闁?Pitch 閻庢稒顨嗛宀勫Υ娴ｈ鍣伴柟鍖＄畳閻?duty闁靛棔鐒︾€氀冾嚕?duty闁?*/
    tx_data[0] = (uint8_t)((gimbal_.pitch_target_duty_ >> 8) & 0xFF);
    tx_data[1] = (uint8_t)(gimbal_.pitch_target_duty_ & 0xFF);
    tx_data[2] = 0;
    tx_data[3] = 0;

    tx_data[4] = (uint8_t)((gimbal_.fric_duty_ >> 8) & 0xFF);
    tx_data[5] = (uint8_t)(gimbal_.fric_duty_ & 0xFF);
    tx_data[6] = (uint8_t)((gimbal_.feed_duty_ >> 8) & 0xFF);
    tx_data[7] = (uint8_t)(gimbal_.feed_duty_ & 0xFF);

    canSend(GIMBAL_CAN_CMD_ID, tx_data, CAN_FRAME_DATA_LEN);
}

void gimbalProcessFeedback(uint32_t can_id, const uint8_t *data)
{
    /* 0x206/0x207 閻忕偟鍋樼花顒勬偑椤掑倻褰岄柛娆愬灥閻ㄧ娀寮堕崠锛勭闁告瑯鍨甸崗姗€寮伴姘伎闂傚棗澧庢慨鎼佸箑娴ｅ憡濮庨柨娑欑◥缁垶宕ｉ弶鍨摕閺夌偞娼欒ぐ褔骞掗妷锔芥毆
     * 闁煎浜滅换渚€鎯?0x208/0x209闁挎稑鐭傛导鈺呭礂瀹ュ懏鍊卞☉鎾亾闁诡剝宕甸崵搴㈢▔婵犲嫭鐣遍柡鍐勫嫭绶查柣妯垮煐閳ь兛娴囬々顐︽儎閺嶎偅鍩傞悗鍦仜瀵姤锛冮崼娑掑亾?*/
    if (can_id == GIMBAL_CAN_YAW_PITCH_SPEED_ID) {
        gimbal_.pitch_current_speed_ = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
        gimbal_.yaw_current_speed_ = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    } else if (can_id == GIMBAL_CAN_YAW_PITCH_POS_ID) {
        gimbal_.pitch_current_pos_ = (int32_t)(((uint32_t)data[0] << 24) |
                                               ((uint32_t)data[1] << 16) |
                                               ((uint32_t)data[2] << 8) | data[3]);
        gimbal_.yaw_current_pos_ = (int32_t)(((uint32_t)data[4] << 24) |
                                             ((uint32_t)data[5] << 16) |
                                             ((uint32_t)data[6] << 8) | data[7]);
        if (gimbal_.yaw_feedback_valid_ == 0U) {
            gimbal_.yaw_target_pos_ = gimbal_.yaw_current_pos_;
            gimbal_.yaw_center_pos_ = gimbal_.yaw_current_pos_;
        }
        gimbal_.yaw_feedback_tick_ = HAL_GetTick();
        gimbal_.yaw_feedback_valid_ = 1U;
    }
}

void gimbalSendYawPitchCmd(void)
{
    uint8_t tx_data[CAN_FRAME_DATA_LEN];
    tx_data[0] = (uint8_t)((gimbal_.pitch_target_duty_ >> 8) & 0xFF);
    tx_data[1] = (uint8_t)(gimbal_.pitch_target_duty_ & 0xFF);
    tx_data[2] = 0;
    tx_data[3] = 0;
    tx_data[4] = (uint8_t)((gimbal_.yaw_target_duty_ >> 8) & 0xFF);
    tx_data[5] = (uint8_t)(gimbal_.yaw_target_duty_ & 0xFF);
    tx_data[6] = 0;
    tx_data[7] = 0;
    g_debug_.gimbal_.pitch_target_duty_ = gimbal_.pitch_target_duty_;
    g_debug_.gimbal_.yaw_target_duty_ = gimbal_.yaw_target_duty_;
    g_debug_.gimbal_.yaw_tx_status_ = canSend(YAW_PITCH_CAN_CMD_ID, tx_data, CAN_FRAME_DATA_LEN);
}
