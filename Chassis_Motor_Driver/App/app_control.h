/* app_control.h */
#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include "bsp_motor.h"
#include "bsp_encoder.h"
#include <stdint.h>
#include <stddef.h>

typedef struct DebugData DebugData;

#define CONTROL_MOTOR_NUM 2

#define MOTOR_SPEED_PID_KP 10.0f
#define MOTOR_SPEED_PID_KI 1.0f
#define MOTOR_SPEED_PID_KD 10.0f
#define MOTOR_SPEED_PID_MAX_OUT MAX_PWM_VALUE
#define MOTOR_SPEED_PID_MAX_INTEGRAL_OUT 1000.0f
#define MOTOR_SPEED_FILTER_ALPHA 0.25f
/* PWM output per (RPM/s); tune this value on the real wheel/motor. */
#define MOTOR_SPEED_FF_ACCEL_GAIN 1.0f
#define MOTOR_SPEED_FF_MAX_ACCEL 3000.0f

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
} PidController;

void pidInit(PidController *pid, PidType type, float kp, float ki, float kd, float max_out, float max_integral);
float pidCalculate(PidController *pid, float measure, float target);

void appControlInit(void);
void appControlLoop(void);
void appControlSetTargetSpeed(MotorIdType motor_id, float speed);
void appControlGetDebugData(volatile DebugData *debug);

#endif /* APP_CONTROL_H */
