#include "app_chassis.h"
#include <math.h>

void chassisInit(Chassis *chassis)
{
    chassis->target_.vx_ = 0.0f;
    chassis->target_.vy_ = 0.0f;
    chassis->target_.vw_ = 0.0f;

    chassis->radius_       = CHASSIS_RADIUS;
    chassis->wheel_radius_ = CHASSIS_WHEEL_RADIUS;

    for (uint8_t i = 0; i < CHASSIS_BOARD_COUNT; i++) {
        chassis->motor_cmds_[i].motor1_speed_ = 0;
        chassis->motor_cmds_[i].motor2_speed_ = 0;
        chassis->motor_fb_[i].motor1_speed_   = 0;
        chassis->motor_fb_[i].motor2_speed_   = 0;
        chassis->motor_fb_[i].motor1_pos_     = 0;
        chassis->motor_fb_[i].motor2_pos_     = 0;
    }
}

void chassisSetTarget(Chassis *chassis, float vx, float vy, float vw)
{
    chassis->target_.vx_ = vx;
    chassis->target_.vy_ = vy;
    chassis->target_.vw_ = vw;
}

void chassisUpdate(Chassis *chassis)
{
    float vx = chassis->target_.vx_;
    float vy = chassis->target_.vy_;
    float vw = chassis->target_.vw_;
    float R  = chassis->wheel_radius_;
    float Lc = chassis->radius_;

    float omega_FL = -((M_SQRT2 / 2.0f * (-vx - vy) + Lc * vw) / R * CHASSIS_RPM_CONVERT);
    float omega_FR = -((M_SQRT2 / 2.0f * (-vx + vy) + Lc * vw) / R * CHASSIS_RPM_CONVERT);
    float omega_RL = -((M_SQRT2 / 2.0f * (vx + vy) + Lc * vw) / R * CHASSIS_RPM_CONVERT);
    float omega_RR = -((M_SQRT2 / 2.0f * (vx - vy) + Lc * vw) / R * CHASSIS_RPM_CONVERT);

    chassis->motor_cmds_[0].motor1_speed_ = (int16_t)omega_FL;
    chassis->motor_cmds_[0].motor2_speed_ = (int16_t)omega_FR;
    chassis->motor_cmds_[1].motor1_speed_ = (int16_t)omega_RL;
    chassis->motor_cmds_[1].motor2_speed_ = (int16_t)omega_RR;
}

void chassisSendCmd(const Chassis *chassis)
{
    protocolSendBoardCmd(1, &chassis->motor_cmds_[0]);
    protocolSendBoardCmd(2, &chassis->motor_cmds_[1]);
}

void chassisProcessFeedback(Chassis *chassis, uint32_t can_id, const uint8_t *data)
{
    protocolProcessCanMessage(can_id, data,
                              &chassis->motor_fb_[0], &chassis->motor_fb_[1]);
}