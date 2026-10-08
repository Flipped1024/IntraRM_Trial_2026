#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "stm32f1xx_hal.h"
#include "tim.h"

#define MAX_PWM_VALUE 999  /* TIM ARR  */

/* 两路电机使用与 Chassis_Motor_Driver 相同的编码器/减速箱参数。 */
#define PITCH_GEAR_RATIO        30.0f
#define YAW_GEAR_RATIO          45.0f
#define ENCODER_PPR             11.0f
#define ENCODER_MULTIPLIER      4.0f     
#define CONTROL_PERIOD_S        0.01f     // 控制周期(秒)  100Hz 

/* 减速箱输出轴转一圈对应的总脉冲数 */
#define PITCH_PULSES_PER_SHAFT_REV (ENCODER_PPR * ENCODER_MULTIPLIER * PITCH_GEAR_RATIO)
#define YAW_PULSES_PER_SHAFT_REV   (ENCODER_PPR * ENCODER_MULTIPLIER * YAW_GEAR_RATIO)

/* 脉冲增量转输出轴 RPM 的转换系数: (60 / T) / PulsesPerRev */
#define PULSE_TO_RPM_COEF       (60.0f / (CONTROL_PERIOD_S * PITCH_PULSES_PER_SHAFT_REV))

typedef enum {
    MOTOR_PITCH = 0,
    MOTOR_YAW,
    MOTOR_COUNT
} MotorIdType;

typedef struct {
    TIM_HandleTypeDef* htim_;
    uint32_t channel_in1_;
    uint32_t channel_in2_;
    int32_t current_speed_;
} MotorHandleType;

void bspMotorInit(void);
void bspMotorSetSpeed(MotorIdType motor_id, int32_t speed);
int32_t bspMotorGetSpeed(MotorIdType motor_id);
float bspMotorPulsesPerShaftRev(MotorIdType motor_id);
float bspMotorPulseToRpm(MotorIdType motor_id);

#endif /* __BSP_MOTOR_H */
