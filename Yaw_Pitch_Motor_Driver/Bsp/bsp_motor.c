#include "bsp_motor.h"

static MotorHandleType motor_list[] = {
    [MOTOR_PITCH] = {.htim_ = &htim2, .channel_in1_ = TIM_CHANNEL_1, .channel_in2_ = TIM_CHANNEL_2, .current_speed_ = 0},
    [MOTOR_YAW] = {.htim_ = &htim2, .channel_in1_ = TIM_CHANNEL_3, .channel_in2_ = TIM_CHANNEL_4, .current_speed_ = 0}};

float bspMotorPulsesPerShaftRev(MotorIdType motor_id)
{
    return (motor_id == MOTOR_YAW) ? YAW_PULSES_PER_SHAFT_REV : PITCH_PULSES_PER_SHAFT_REV;
}

float bspMotorPulseToRpm(MotorIdType motor_id)
{
    return 60.0f / (CONTROL_PERIOD_S * bspMotorPulsesPerShaftRev(motor_id));
}

/**
 * @brief 鍒濆鍖栫數鏈虹浉鍏冲畾鏃跺櫒閫氶亾
 */
void bspMotorInit(void)
{
    HAL_TIM_PWM_Start(motor_list[MOTOR_PITCH].htim_, motor_list[MOTOR_PITCH].channel_in1_);
    HAL_TIM_PWM_Start(motor_list[MOTOR_PITCH].htim_, motor_list[MOTOR_PITCH].channel_in2_);
    HAL_TIM_PWM_Start(motor_list[MOTOR_YAW].htim_, motor_list[MOTOR_YAW].channel_in1_);
    HAL_TIM_PWM_Start(motor_list[MOTOR_YAW].htim_, motor_list[MOTOR_YAW].channel_in2_);

    bspMotorSetSpeed(MOTOR_PITCH, 0);
    bspMotorSetSpeed(MOTOR_YAW, 0);
}

/**
 * @brief 璁剧疆鐢垫満閫熷害涓庢柟鍚?
 * @param motor_id 鐢垫満缂栧彿
 * @param speed 鎺у埗閲忚寖鍥?[-MAX_PWM_VALUE, MAX_PWM_VALUE]
 */
void bspMotorSetSpeed(MotorIdType motor_id, int32_t speed)
{
    uint32_t duty_cycle = 0;
    int32_t pwm_speed;

    if (motor_id >= MOTOR_COUNT)
    {
        return;
    }
    MotorHandleType *motor = &motor_list[motor_id];

    if (speed > MAX_PWM_VALUE)
    {
        speed = MAX_PWM_VALUE;
    }
    else if (speed < -MAX_PWM_VALUE)
    {
        speed = -MAX_PWM_VALUE;
    }

    /* Keep current_speed_ in logical units; only the H-bridge command is inverted. */
    motor->current_speed_ = speed;
    pwm_speed = (motor_id == MOTOR_YAW) ? (speed * YAW_MOTOR_DIRECTION) : speed;

    if (pwm_speed > 0)
    {
        duty_cycle = (uint32_t)pwm_speed;
        __HAL_TIM_SET_COMPARE(motor->htim_, motor->channel_in1_, duty_cycle);
        __HAL_TIM_SET_COMPARE(motor->htim_, motor->channel_in2_, 0);
    }
    else if (pwm_speed < 0)
    {
        duty_cycle = (uint32_t)(-pwm_speed);
        __HAL_TIM_SET_COMPARE(motor->htim_, motor->channel_in1_, 0);
        __HAL_TIM_SET_COMPARE(motor->htim_, motor->channel_in2_, duty_cycle);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(motor->htim_, motor->channel_in1_, 0);
        __HAL_TIM_SET_COMPARE(motor->htim_, motor->channel_in2_, 0);
    }
}

int32_t bspMotorGetSpeed(MotorIdType motor_id)
{
    if (motor_id < MOTOR_COUNT)
    {
        return motor_list[motor_id].current_speed_;
    }
    return 0;
}

