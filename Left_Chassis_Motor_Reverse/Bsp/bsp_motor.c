#include "bsp_motor.h"

static MotorHandleType motor_list[] = {
    [MOTOR_1] = {.htim_ = &htim2, .channel_in1_ = TIM_CHANNEL_1, .channel_in2_ = TIM_CHANNEL_2, .current_speed_ = 0},
    [MOTOR_2] = {.htim_ = &htim2, .channel_in1_ = TIM_CHANNEL_3, .channel_in2_ = TIM_CHANNEL_4, .current_speed_ = 0}};

/**
 * @brief 初始化电机相关定时器通道
 */
void bspMotorInit(void)
{
    HAL_TIM_PWM_Start(motor_list[MOTOR_1].htim_, motor_list[MOTOR_1].channel_in1_);
    HAL_TIM_PWM_Start(motor_list[MOTOR_1].htim_, motor_list[MOTOR_1].channel_in2_);
    HAL_TIM_PWM_Start(motor_list[MOTOR_2].htim_, motor_list[MOTOR_2].channel_in1_);
    HAL_TIM_PWM_Start(motor_list[MOTOR_2].htim_, motor_list[MOTOR_2].channel_in2_);

    bspMotorSetSpeed(MOTOR_1, 0);
    bspMotorSetSpeed(MOTOR_2, 0);
}

/**
 * @brief 设置电机速度与方向
 * @param motor_id 电机编号
 * @param speed 控制量范围 [-MAX_PWM_VALUE, MAX_PWM_VALUE]
 */
void bspMotorSetSpeed(MotorIdType motor_id, int32_t speed)
{
    uint32_t duty_cycle = 0;
    MotorHandleType *motor = &motor_list[motor_id];

    if (speed > MAX_PWM_VALUE)
    {
        speed = MAX_PWM_VALUE;
    }
    else if (speed < -MAX_PWM_VALUE)
    {
        speed = -MAX_PWM_VALUE;
    }

    motor->current_speed_ = speed;

    /* Reverse cable: invert only the physical H-bridge direction. */
    speed = -speed;

    if (speed > 0)
    {
        duty_cycle = (uint32_t)speed;
        __HAL_TIM_SET_COMPARE(motor->htim_, motor->channel_in1_, duty_cycle);
        __HAL_TIM_SET_COMPARE(motor->htim_, motor->channel_in2_, 0);
    }
    else if (speed < 0)
    {
        duty_cycle = (uint32_t)(-speed);
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
    if (motor_id >= MOTOR_1 && motor_id <= MOTOR_2)
    {
        return motor_list[motor_id].current_speed_;
    }
    return 0;
}
