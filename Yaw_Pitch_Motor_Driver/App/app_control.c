/* app_control.c */
#include "app_control.h"
#include "app_debug.h"

static PidController position_pid_motors[CONTROL_MOTOR_NUM];
static PidController speed_pid_motors[CONTROL_MOTOR_NUM];
static volatile int32_t target_positions[CONTROL_MOTOR_NUM] = {0, 0};
static volatile float target_speeds_rpm[CONTROL_MOTOR_NUM] = {0.0f, 0.0f};
static volatile int32_t target_open_loop_duty[CONTROL_MOTOR_NUM] = {0, 0};
static volatile uint32_t yaw_command_tick;
static volatile uint8_t yaw_command_received;
static float filtered_speed_rpm[CONTROL_MOTOR_NUM] = {0.0f, 0.0f};
static float raw_speed_rpm[CONTROL_MOTOR_NUM] = {0.0f, 0.0f};

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
    pid->deadband_ = 0.0f;
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

    if (pid->deadband_ > 0.0f &&
        pid->error_ > -pid->deadband_ && pid->error_ < pid->deadband_)
    {
        pid->error_ = 0.0f;
    }

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
        filtered_speed_rpm[i] = 0.0f;
        raw_speed_rpm[i] = 0.0f;
    }

    for (uint8_t i = 0; i < CONTROL_MOTOR_NUM; i++)
    {
        pidInit(&position_pid_motors[i],
                PID_POSITION,
                MOTOR_POSITION_PID_KP, MOTOR_POSITION_PID_KI, MOTOR_POSITION_PID_KD,
                MOTOR_POSITION_PID_MAX_OUT, MOTOR_POSITION_PID_MAX_INTEGRAL_OUT);
        pidInit(&speed_pid_motors[i],
                PID_INCREMENTAL,
                MOTOR_SPEED_PID_KP, MOTOR_SPEED_PID_KI, MOTOR_SPEED_PID_KD,
                MOTOR_SPEED_PID_MAX_OUT, MOTOR_SPEED_PID_MAX_INTEGRAL_OUT);
    }

    appDebugInit();
}

void appControlSetTargetPosition(MotorIdType motor_id, int32_t position)
{
    if (motor_id < CONTROL_MOTOR_NUM)
    {
        target_positions[motor_id] = position;
    }
}

void appControlSetTargetSpeed(MotorIdType motor_id, float speed_rpm)
{
    if (motor_id < CONTROL_MOTOR_NUM) {
        target_speeds_rpm[motor_id] = speed_rpm;
        if (motor_id == MOTOR_YAW) {
            yaw_command_tick = HAL_GetTick();
            yaw_command_received = 1U;
        }
    }
}

void appControlSetOpenLoopDuty(MotorIdType motor_id, int32_t duty)
{
    if (motor_id < CONTROL_MOTOR_NUM) {
        target_open_loop_duty[motor_id] = duty;
    }
}

void appControlLoop(void)
{
    for (uint8_t i = 0; i < CONTROL_MOTOR_NUM; i++)
    {
        position_pid_motors[i].kp_ = debug_data.pos_kp;
        position_pid_motors[i].ki_ = debug_data.pos_ki;
        position_pid_motors[i].kd_ = debug_data.pos_kd;
        position_pid_motors[i].max_out_ = debug_data.pos_max_out;
        position_pid_motors[i].max_integral_ = debug_data.pos_max_integral;

        speed_pid_motors[i].kp_ = debug_data.spd_kp;
        speed_pid_motors[i].ki_ = debug_data.spd_ki;
        speed_pid_motors[i].kd_ = debug_data.spd_kd;
        speed_pid_motors[i].max_out_ = debug_data.spd_max_out;
        speed_pid_motors[i].max_integral_ = debug_data.spd_max_integral;
        speed_pid_motors[i].deadband_ = debug_data.spd_deadband;

        uint8_t enc_id = i + 1;
        bspEncoderUpdate(enc_id);
        int16_t delta_pulse = bspGetEncoderDeltaPulse(enc_id);
        int32_t current_position = bspGetEncoderTotalPulse(enc_id);
        raw_speed_rpm[i] = (float)delta_pulse * bspMotorPulseToRpm((MotorIdType)i);

        float filter_alpha = debug_data.spd_filter_alpha;
        if (filter_alpha <= 0.0f)
        {
            filter_alpha = 1.0f;
        }
        else if (filter_alpha > 1.0f)
        {
            filter_alpha = 1.0f;
        }

        filtered_speed_rpm[i] += filter_alpha *
                                 (raw_speed_rpm[i] - filtered_speed_rpm[i]);

        float position_target_pulse = (float)target_positions[i];
        float target_speed_rpm;

        if ((i == MOTOR_YAW || i == MOTOR_PITCH) && debug_data.debug_mode != 1U) {
            target_speed_rpm = target_speeds_rpm[i];
            if (!yaw_command_received || (uint32_t)(HAL_GetTick() - yaw_command_tick) > 150U) {
                target_speed_rpm = 0.0f;
            }
        } else if (debug_data.debug_mode == 1U)
        {
            /* Ozone debug mode: use the editable target directly as the
             * speed-loop setpoint, while retaining encoder feedback and the
             * speed PID/current limiting. */
            target_speed_rpm = debug_data.motor[i].debug_target_speed_rpm;
        }
        else
        {
            if (debug_data.debug_mode == 2U)
            {
                position_target_pulse =
                    debug_data.motor[i].debug_target_angle_deg *
                    (bspMotorPulsesPerShaftRev((MotorIdType)i) / 360.0f);
            }

            target_speed_rpm = pidCalculate(&position_pid_motors[i],
                                            (float)current_position,
                                            position_target_pulse);
        }
        (void)target_speed_rpm;
        bspMotorSetSpeed((MotorIdType)i, target_open_loop_duty[i]);
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
        if (debug_data.debug_mode == 2U)
        {
            debug->motor[i].target_pos_pulse = (int32_t)(
                debug_data.motor[i].debug_target_angle_deg *
                (bspMotorPulsesPerShaftRev((MotorIdType)i) / 360.0f));
        }
        else
        {
            debug->motor[i].target_pos_pulse = target_positions[i];
        }
        debug->motor[i].current_pos_pulse = bspGetEncoderTotalPulse(enc_id);
        debug->motor[i].current_pos_rev =
            (float)debug->motor[i].current_pos_pulse / bspMotorPulsesPerShaftRev((MotorIdType)i);
        debug->motor[i].current_pos_angle_deg =
            (float)debug->motor[i].current_pos_pulse *
            (360.0f / bspMotorPulsesPerShaftRev((MotorIdType)i));
        debug->motor[i].pos_error = position_pid_motors[i].error_;
        debug->motor[i].pos_output = position_pid_motors[i].output_;
        debug->motor[i].target_speed_rpm = speed_pid_motors[i].target_;
        debug->motor[i].current_speed_rpm = speed_pid_motors[i].measure_;
        debug->motor[i].raw_speed_rpm = raw_speed_rpm[i];
        debug->motor[i].delta_pulse = bspGetEncoderDeltaPulse(enc_id);
        debug->motor[i].speed_error = speed_pid_motors[i].error_;
        debug->motor[i].speed_output = speed_pid_motors[i].output_;
        debug->motor[i].motor_duty = bspMotorGetSpeed((MotorIdType)i);
        debug->motor[i].total_pulse = bspGetEncoderTotalPulse(enc_id);
    }
}
