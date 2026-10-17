/* bsp_encoder.c */
#include "bsp_encoder.h"

static EncoderInstance encoder_instances[ENCODER_NUM];

void bspEncoderInit(void)
{
    encoder_instances[MOTOR_PITCH_ID - 1].htim_ = &htim1;
    encoder_instances[MOTOR_PITCH_ID - 1].delta_pulse_ = 0;
    encoder_instances[MOTOR_PITCH_ID - 1].total_pulse_ = 0;
    encoder_instances[MOTOR_PITCH_ID - 1].last_cnt_ = 0;
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

    encoder_instances[MOTOR_YAW_ID - 1].htim_ = &htim3;
    encoder_instances[MOTOR_YAW_ID - 1].delta_pulse_ = 0;
    encoder_instances[MOTOR_YAW_ID - 1].total_pulse_ = 0;
    encoder_instances[MOTOR_YAW_ID - 1].last_cnt_ = 0;
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

void bspEncoderUpdate(uint8_t motor_id)
{
    if (motor_id < 1 || motor_id > ENCODER_NUM)
    {
        return;
    }

    EncoderInstance *enc = &encoder_instances[motor_id - 1];

    uint16_t current_cnt = __HAL_TIM_GET_COUNTER(enc->htim_);

    enc->delta_pulse_ = (int16_t)(current_cnt - enc->last_cnt_);
    /* Only Yaw uses the reversed cable; keep Pitch feedback unchanged. */
    if (motor_id == MOTOR_YAW_ID)
    {
        enc->delta_pulse_ = (int16_t)-enc->delta_pulse_;
    }
    enc->total_pulse_ += enc->delta_pulse_;
    enc->last_cnt_ = current_cnt;
}

int16_t bspGetEncoderDeltaPulse(uint8_t motor_id)
{
    if (motor_id < 1 || motor_id > ENCODER_NUM)
        return 0;
    return encoder_instances[motor_id - 1].delta_pulse_;
}

int32_t bspGetEncoderTotalPulse(uint8_t motor_id)
{
    if (motor_id < 1 || motor_id > ENCODER_NUM)
        return 0;
    return encoder_instances[motor_id - 1].total_pulse_;
}
