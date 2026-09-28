#include "app_debug.h"
#include "app_control.h"

volatile DebugData debug_data;

void appDebugInit(void)
{
    debug_data.spd_kp = MOTOR_SPEED_PID_KP;
    debug_data.spd_ki = MOTOR_SPEED_PID_KI;
    debug_data.spd_kd = MOTOR_SPEED_PID_KD;
    debug_data.spd_max_out = (float)MOTOR_SPEED_PID_MAX_OUT;
    debug_data.spd_max_integral = MOTOR_SPEED_PID_MAX_INTEGRAL_OUT;
    debug_data.spd_filter_alpha = MOTOR_SPEED_FILTER_ALPHA;
    debug_data.spd_ff_accel = MOTOR_SPEED_FF_ACCEL_GAIN;
    debug_data.debug_mode = 0;
    for (uint8_t i = 0; i < DEBUG_MOTOR_NUM; i++)
    {
        debug_data.motor[i].debug_target_speed_rpm = 0.0f;
    }
}

void appDebugUpdate(void)
{
    appControlGetDebugData(&debug_data);
}
