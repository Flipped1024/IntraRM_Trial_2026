#include "bsp_motor.h"
#include "main.h"

/*
 * 当前板级线序（与 Fric_Feed_Motor_Driver.ioc 保持一致）：
 *   370-1: TIM2_CH1 / PA0，方向 PA1
 *   370-2: TIM2_CH3 / PA2，方向 PA3
 *   N20 : TIM3_CH1 / PA6，TIM3_CH2 / PA7
 * PWM 口为低有效：比较值 0 对应最高速，比较值 MAX_PWM_VALUE 对应停止。
 * GPIO 方向脚低电平（接近负极）只切换方向，不控制转速。
 * 说明：TIM2/TIM3 的 CubeMX 配置也已写入 .ioc；不要恢复旧的 TIM2_CH2/CH4 绑定。
 */
static MotorHandleType motor_list[MOTOR_NUM] = {
    [MOTOR_370_FRIC1] = {
        .htim_ = &htim2,
        .pwm_mode_ = MOTOR_MODE_SINGLE_PWM,
        .channel_in1_ = TIM_CHANNEL_1,
        .channel_in2_ = 0,
        .dir_port_ = Motor370_1_Direction_GPIO_Port,
        .dir_pin_ = Motor370_1_Direction_Pin,
        .current_duty_ = 0,
        .direction_invert_ = 0
    },
    [MOTOR_370_FRIC2] = {
        .htim_ = &htim2,
        .pwm_mode_ = MOTOR_MODE_SINGLE_PWM,
        .channel_in1_ = TIM_CHANNEL_3,
        .channel_in2_ = 0,
        .dir_port_ = Motor370_2_Direction_GPIO_Port,
        .dir_pin_ = Motor370_2_Direction_Pin,
        .current_duty_ = 0,
        /* FRIC2 的实际安装/线序与当前逻辑方向相反，统一翻转其方向极性。 */
        .direction_invert_ = 1
    },
    [MOTOR_N20_FEED] = {
        .htim_ = &htim3,
        .pwm_mode_ = MOTOR_MODE_DUAL_PWM,
        /* 交换双 PWM 输出，使相同拨弹指令的实际转向反转。 */
        .channel_in1_ = TIM_CHANNEL_1,
        .channel_in2_ = TIM_CHANNEL_2,
        .dir_port_ = NULL,
        .dir_pin_ = 0,
        .current_duty_ = 0
    }
};

/**
 * @brief 初始化所有电机PWM通道并停止电机
 */
void bspMotorInit(void)
{
    /* 按各摩擦轮的固定方向设置方向脚；FRIC2 反向时 PA3 上电即为低电平。 */
    HAL_GPIO_WritePin(motor_list[MOTOR_370_FRIC1].dir_port_,
        motor_list[MOTOR_370_FRIC1].dir_pin_,
        motor_list[MOTOR_370_FRIC1].direction_invert_ ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor_list[MOTOR_370_FRIC2].dir_port_,
        motor_list[MOTOR_370_FRIC2].dir_pin_,
        motor_list[MOTOR_370_FRIC2].direction_invert_ ? GPIO_PIN_RESET : GPIO_PIN_SET);

    /* TIM 初始化的默认比较值为 0，而本板 0 是满速；启动 PWM 前先写入停转值。 */
    __HAL_TIM_SET_COMPARE(motor_list[MOTOR_370_FRIC1].htim_,
        motor_list[MOTOR_370_FRIC1].channel_in1_, MAX_PWM_VALUE);
    __HAL_TIM_SET_COMPARE(motor_list[MOTOR_370_FRIC2].htim_,
        motor_list[MOTOR_370_FRIC2].channel_in1_, MAX_PWM_VALUE);

    HAL_TIM_PWM_Start(motor_list[MOTOR_370_FRIC1].htim_, motor_list[MOTOR_370_FRIC1].channel_in1_);
    HAL_TIM_PWM_Start(motor_list[MOTOR_370_FRIC2].htim_, motor_list[MOTOR_370_FRIC2].channel_in1_);

    HAL_TIM_PWM_Start(motor_list[MOTOR_N20_FEED].htim_, motor_list[MOTOR_N20_FEED].channel_in1_);
    HAL_TIM_PWM_Start(motor_list[MOTOR_N20_FEED].htim_, motor_list[MOTOR_N20_FEED].channel_in2_);

    for (MotorIdType i = 0; i < MOTOR_NUM; i++)
    {
        bspMotorStop(i);
    }
}

/**
 * @brief 设置电机占空比
 * @param motor_id 电机编号
 * @param duty PWM 比较值；摩擦轮低有效，0 为最高速，MAX_PWM_VALUE 为停止。
 *              摩擦轮方向由 direction_invert_ 固定设置，不由 duty 符号切换。
 *  - 双PWM模式: 范围 [-MAX_PWM_VALUE, MAX_PWM_VALUE], 正负号控制方向
 *  - 单PWM+GPIO模式: GPIO 低电平（接负极）只切换方向；PWM 比较值 0 为最高速，
 *    MAX_PWM_VALUE 为停止。direction_invert_ 用于适配两个摩擦轮的机械安装方向。
 */
void bspMotorSetDuty(MotorIdType motor_id, int32_t duty)
{
    MotorHandleType *motor = &motor_list[motor_id];

    if (duty > MAX_PWM_VALUE)
    {
        duty = MAX_PWM_VALUE;
    }
    else if (duty < -MAX_PWM_VALUE)
    {
        duty = -MAX_PWM_VALUE;
    }

    motor->current_duty_ = duty;

    if (motor->pwm_mode_ == MOTOR_MODE_SINGLE_PWM)
    {
        if (motor->dir_port_ != NULL && motor->dir_pin_ != 0)
        {
            /* 方向脚固定：FRIC2（direction_invert_=1）低电平反向；duty 仅表示速度。 */
            HAL_GPIO_WritePin(motor->dir_port_, motor->dir_pin_,
                motor->direction_invert_ ? GPIO_PIN_RESET : GPIO_PIN_SET);
            uint32_t compare = (duty < 0) ? (uint32_t)(-duty) : (uint32_t)duty;
            __HAL_TIM_SET_COMPARE(motor->htim_, motor->channel_in1_, compare);
        }
        else
        {
            if (duty > 0)
            {
                __HAL_TIM_SET_COMPARE(motor->htim_, motor->channel_in1_, (uint32_t)duty);
            }
            else if (duty < 0)
            {
                __HAL_TIM_SET_COMPARE(motor->htim_, motor->channel_in1_, (uint32_t)(-duty));
            }
            else
            {
                /* PWM 低有效：比较值 0 对应最高速。 */
                __HAL_TIM_SET_COMPARE(motor->htim_, motor->channel_in1_, 0);
            }
        }
    }
    else
    {
        if (duty > 0)
        {
            __HAL_TIM_SET_COMPARE(motor->htim_, motor->channel_in1_, (uint32_t)duty);
            __HAL_TIM_SET_COMPARE(motor->htim_, motor->channel_in2_, 0);
        }
        else if (duty < 0)
        {
            __HAL_TIM_SET_COMPARE(motor->htim_, motor->channel_in1_, 0);
            __HAL_TIM_SET_COMPARE(motor->htim_, motor->channel_in2_, (uint32_t)(-duty));
        }
        else
        {
            __HAL_TIM_SET_COMPARE(motor->htim_, motor->channel_in1_, 0);
            __HAL_TIM_SET_COMPARE(motor->htim_, motor->channel_in2_, 0);
        }
    }
}

/**
 * @brief 停止指定电机
 * @param motor_id 电机编号
 */
void bspMotorStop(MotorIdType motor_id)
{
    MotorHandleType *motor = &motor_list[motor_id];

    if (motor->pwm_mode_ == MOTOR_MODE_SINGLE_PWM)
    {
        /* 注意：本板摩擦轮 PWM 低有效，比较值 999 才是真正停止。 */
        motor->current_duty_ = MAX_PWM_VALUE;
        __HAL_TIM_SET_COMPARE(motor->htim_, motor->channel_in1_, MAX_PWM_VALUE);
    }
    else
    {
        /* N20 双 PWM 保持原有刹停方式。 */
        bspMotorSetDuty(motor_id, 0);
    }
}

/**
 * @brief 获取电机当前占空比
 * @param motor_id 电机编号
 * @return 当前占空比值
 */
int32_t bspMotorGetDuty(MotorIdType motor_id)
{
    return motor_list[motor_id].current_duty_;
}
