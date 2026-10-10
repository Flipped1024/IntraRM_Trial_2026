/* app_control.h */
#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include "bsp_motor.h"
#include "bsp_encoder.h"
#include <stdint.h>
#include <stddef.h>

typedef struct DebugData DebugData;

#define CONTROL_MOTOR_NUM 2

#define MOTOR_POSITION_PID_KP 2.0f
#define MOTOR_POSITION_PID_KI 0.0f
#define MOTOR_POSITION_PID_KD 15.0f
#define MOTOR_POSITION_PID_MAX_OUT 120.0f       /* 浣嶇疆鐜緭鍑猴細鐩爣 RPM */
#define MOTOR_POSITION_PID_MAX_INTEGRAL_OUT 2000.0f

#define MOTOR_SPEED_PID_KP 5.0f
#define MOTOR_SPEED_PID_KI 0.1f
#define MOTOR_SPEED_PID_KD 0.0f
#define MOTOR_SPEED_PID_MAX_OUT MAX_PWM_VALUE
#define MOTOR_SPEED_PID_MAX_INTEGRAL_OUT 1000.0f
typedef enum
{
    PID_POSITION = 0,
    PID_INCREMENTAL
} PidType;

typedef struct
{
    PidType pid_type_;
    float kp_;
    float ki_;
    float kd_;

    float max_out_;
    float max_integral_;

    float target_;
    float measure_;

    float error_;
    float last_error_;
    float prev_error_;
    float integral_;
    float output_;
    float deadband_;
} PidController;

void pidInit(PidController *pid, PidType type, float kp, float ki, float kd, float max_out, float max_integral);
float pidCalculate(PidController *pid, float measure, float target);

void appControlInit(void);
void appControlLoop(void);
void appControlSetTargetPosition(MotorIdType motor_id, int32_t position);
void appControlSetTargetSpeed(MotorIdType motor_id, float speed_rpm);
void appControlSetOpenLoopDuty(MotorIdType motor_id, int32_t duty);
void appControlGetDebugData(volatile DebugData *debug);

#endif /* APP_CONTROL_H */
