#include "app_chassis.h"
#include "app_debug.h"
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
        chassis->motor_fb_[i].motor1_delta_pulse_ = 0;
        chassis->motor_fb_[i].motor2_delta_pulse_ = 0;
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

    /* Keep all four commands within the motor's measured 178 rpm limit while
     * preserving the direction and speed ratio of the current chassis mix. */
    float max_abs_speed = fabsf(omega_FL);
    float scale;

    if (fabsf(omega_FR) > max_abs_speed) {
        max_abs_speed = fabsf(omega_FR);
    }
    if (fabsf(omega_RL) > max_abs_speed) {
        max_abs_speed = fabsf(omega_RL);
    }
    if (fabsf(omega_RR) > max_abs_speed) {
        max_abs_speed = fabsf(omega_RR);
    }

    if (max_abs_speed > CHASSIS_MAX_SPEED_RPM) {
        scale = CHASSIS_MAX_SPEED_RPM / max_abs_speed;
        omega_FL *= scale;
        omega_FR *= scale;
        omega_RL *= scale;
        omega_RR *= scale;
    }

    /* 实际接线：驱动板 1 控制左前、右前。 */
    chassis->motor_cmds_[0].motor1_speed_ = (int16_t)omega_FL;
    chassis->motor_cmds_[0].motor2_speed_ = (int16_t)omega_FR;
    /* 实际接线：驱动板 2 控制左后、右后。 */
    chassis->motor_cmds_[1].motor1_speed_ = (int16_t)omega_RL;
    chassis->motor_cmds_[1].motor2_speed_ = (int16_t)omega_RR;
}

void chassisSendCmd(const Chassis *chassis)
{
    HAL_StatusTypeDef status;

    /* Keep the exact four normal-mode commands visible in Ozone. */
    g_debug_.can_.motor_rpm_[0] = chassis->motor_cmds_[0].motor1_speed_;
    g_debug_.can_.motor_rpm_[1] = chassis->motor_cmds_[0].motor2_speed_;
    g_debug_.can_.motor_rpm_[2] = chassis->motor_cmds_[1].motor1_speed_;
    g_debug_.can_.motor_rpm_[3] = chassis->motor_cmds_[1].motor2_speed_;

    status = protocolSendBoardCmd(1, &chassis->motor_cmds_[0]);
    debugUpdateCanTx(status);
    status = protocolSendBoardCmd(2, &chassis->motor_cmds_[1]);
    debugUpdateCanTx(status);
    debugUpdateCanState();
}

void chassisProcessFeedback(Chassis *chassis, uint32_t can_id, const uint8_t *data)
{
    protocolProcessCanMessage(can_id, data,
                              &chassis->motor_fb_[0], &chassis->motor_fb_[1]);
}

void chassisEmergencyStop(Chassis *chassis)
{
    chassis->target_.vx_ = 0.0f;
    chassis->target_.vy_ = 0.0f;
    chassis->target_.vw_ = 0.0f;

    for (uint8_t board = 0U; board < CHASSIS_BOARD_COUNT; ++board) {
        chassis->motor_cmds_[board].motor1_speed_ = 0;
        chassis->motor_cmds_[board].motor2_speed_ = 0;
    }
}
