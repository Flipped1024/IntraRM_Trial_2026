/* app_control.c */
#include "app_control.h"
#include "app_debug.h"

static PidController pid_motors[CONTROL_MOTOR_NUM];
static float target_speeds[CONTROL_MOTOR_NUM] = {0.0f, 0.0f};
static float active_target_speeds[CONTROL_MOTOR_NUM] = {0.0f, 0.0f};
static float filtered_speed_rpm[CONTROL_MOTOR_NUM] = {0.0f, 0.0f};
static float previous_target_speeds[CONTROL_MOTOR_NUM] = {0.0f, 0.0f};
static float feedforward_outputs[CONTROL_MOTOR_NUM] = {0.0f, 0.0f};

void pidInit(PidController *pid, PidType type, float kp, float ki, float kd, float max_out, float max_integral)
{
    if (pid == NULL)
    {
        return;
    }

    pid->pid_type_ = type;
    pid->kp_ = kp;
    pid->ki_ = ki;
    pid->kd_ = kd;
    pid->max_out_ = max_out;
    pid->max_integral_ = max_integral;

    pid->target_ = 0.0f;
    pid->measure_ = 0.0f;
    pid->error_ = 0.0f;
    pid->last_error_ = 0.0f;
    pid->prev_error_ = 0.0f;
    pid->integral_ = 0.0f;
    pid->output_ = 0.0f;
}

float pidCalculate(PidController *pid, float measure, float target)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->measure_ = measure;
    pid->target_ = target;
    pid->error_ = pid->target_ - pid->measure_;

    if (pid->pid_type_ == PID_POSITION)
    {
        pid->integral_ += pid->error_;

        if (pid->integral_ > pid->max_integral_)
        {
            pid->integral_ = pid->max_integral_;
        }
        else if (pid->integral_ < -pid->max_integral_)
        {
            pid->integral_ = -pid->max_integral_;
        }

        pid->output_ = (pid->kp_ * pid->error_) +
                       (pid->ki_ * pid->integral_) +
                       (pid->kd_ * (pid->error_ - pid->last_error_));

        pid->last_error_ = pid->error_;
    }
    else if (pid->pid_type_ == PID_INCREMENTAL)
    {
        float p_term = pid->kp_ * (pid->error_ - pid->last_error_);
        float i_term = pid->ki_ * pid->error_;
        float d_term = pid->kd_ * (pid->error_ - 2.0f * pid->last_error_ + pid->prev_error_);

        pid->output_ += (p_term + i_term + d_term);

        pid->prev_error_ = pid->last_error_;
        pid->last_error_ = pid->error_;
    }

    if (pid->output_ > pid->max_out_)
    {
        pid->output_ = pid->max_out_;
    }
    else if (pid->output_ < -pid->max_out_)
    {
        pid->output_ = -pid->max_out_;
    }

    return pid->output_;
}

void appControlInit(void)
{
    bspMotorInit();
    bspEncoderInit();

    for (uint8_t i = 0; i < CONTROL_MOTOR_NUM; i++)
    {
        pidInit(&pid_motors[i],
                PID_INCREMENTAL,
                MOTOR_SPEED_PID_KP, MOTOR_SPEED_PID_KI, MOTOR_SPEED_PID_KD,
                MOTOR_SPEED_PID_MAX_OUT, MOTOR_SPEED_PID_MAX_INTEGRAL_OUT);
    }

    appDebugInit();
}

void appControlSetTargetSpeed(MotorIdType motor_id, float speed)
{
    if (motor_id < CONTROL_MOTOR_NUM) 
    {
        target_speeds[motor_id] = speed;
    }
}

void appControlLoop(void)
{
    for (uint8_t i = 0; i < CONTROL_MOTOR_NUM; i++) 
    {
        pid_motors[i].kp_ = debug_data.spd_kp;
        pid_motors[i].ki_ = debug_data.spd_ki;
        pid_motors[i].kd_ = debug_data.spd_kd;
        pid_motors[i].max_out_ = debug_data.spd_max_out;
        pid_motors[i].max_integral_ = debug_data.spd_max_integral;

        uint8_t enc_id = i + 1; 
        float target_speed = target_speeds[i];

        if (debug_data.debug_mode == DEBUG_MODE_MANUAL_TARGET)
        {
            target_speed = debug_data.motor[i].debug_target_speed_rpm;
        }

        active_target_speeds[i] = target_speed;
        
        bspEncoderUpdate(enc_id);
        int16_t delta_pulse = bspGetEncoderDeltaPulse(enc_id);
        
        float raw_rpm = (float)delta_pulse * PULSE_TO_RPM_COEF;
        float filter_alpha = debug_data.spd_filter_alpha;

        if (filter_alpha < 0.0f)
        {
            filter_alpha = 0.0f;
        }
        else if (filter_alpha > 1.0f)
        {
            filter_alpha = 1.0f;
        }

        filtered_speed_rpm[i] += filter_alpha * (raw_rpm - filtered_speed_rpm[i]);

        float target_accel = (target_speed - previous_target_speeds[i]) / CONTROL_PERIOD_S;

        if (target_accel > MOTOR_SPEED_FF_MAX_ACCEL)
        {
            target_accel = MOTOR_SPEED_FF_MAX_ACCEL;
        }
        else if (target_accel < -MOTOR_SPEED_FF_MAX_ACCEL)
        {
            target_accel = -MOTOR_SPEED_FF_MAX_ACCEL;
        }

        feedforward_outputs[i] = debug_data.spd_ff_accel * target_accel;
        previous_target_speeds[i] = target_speed;

        float pid_output = pidCalculate(&pid_motors[i], filtered_speed_rpm[i], target_speed);
        float output = pid_output + feedforward_outputs[i];

        if (output > debug_data.spd_max_out)
        {
            output = debug_data.spd_max_out;
        }
        else if (output < -debug_data.spd_max_out)
        {
            output = -debug_data.spd_max_out;
        }
        
        bspMotorSetSpeed((MotorIdType)i, (int32_t)output); 
    }

    appDebugUpdate();
}

void appControlGetDebugData(volatile DebugData *debug)
{
    if (debug == NULL)
    {
        return;
    }

    for (uint8_t i = 0; i < CONTROL_MOTOR_NUM; i++)
    {
        uint8_t enc_id = i + 1;
        debug->motor[i].target_speed_rpm = active_target_speeds[i];
        debug->motor[i].current_speed_rpm = filtered_speed_rpm[i];
        debug->motor[i].raw_speed_rpm = (float)bspGetEncoderDeltaPulse(enc_id) * PULSE_TO_RPM_COEF;
        debug->motor[i].delta_pulse = bspGetEncoderDeltaPulse(enc_id);
        debug->motor[i].speed_error = pid_motors[i].error_;
        debug->motor[i].speed_output = pid_motors[i].output_;
        debug->motor[i].feedforward_output = feedforward_outputs[i];
        debug->motor[i].motor_duty = bspMotorGetSpeed((MotorIdType)i);
        debug->motor[i].total_pulse = bspGetEncoderTotalPulse(enc_id);
    }
}
