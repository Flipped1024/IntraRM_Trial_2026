#ifndef __APP_CHASSIS_H
#define __APP_CHASSIS_H

#include "main.h"
#include "app_protocol.h"

#define CHASSIS_MOTOR_COUNT       4
#define CHASSIS_BOARD_COUNT       2
#define CHASSIS_RADIUS            0.17888f
#define CHASSIS_WHEEL_RADIUS      0.0765f
#define CHASSIS_MAX_SPEED_RPM     120.0f
/* Wheel angular velocity (rad/s) -> revolutions per minute. */
#define CHASSIS_RPM_CONVERT       (60.0f / (2.0f * 3.14159265358979323846f))
#define CHASSIS_STICK_RANGE       127.0f

/* Bottom-board feedback is encoder delta per 10 ms.  The motor driver uses
 * 13 PPR x4 quadrature x56:1 = 2912 output-shaft pulses/rev. */
#define CHASSIS_FEEDBACK_PULSE_TO_RPM \
    (60.0f / (0.01f * 2912.0f))

/* Full-scale joystick commands are chosen so one pure motion reaches the
 * protected 120 rpm wheel-speed limit.  Combined vx/vy/vw commands are
 * proportionally limited again in chassisUpdate(). */
#define CHASSIS_MAX_LINEAR_SPEED_MPS \
    (CHASSIS_MAX_SPEED_RPM * CHASSIS_WHEEL_RADIUS / \
     (CHASSIS_RPM_CONVERT * 0.70710678f))
#define CHASSIS_MAX_ANGULAR_SPEED_RADPS \
    (CHASSIS_MAX_SPEED_RPM * CHASSIS_WHEEL_RADIUS / \
     (CHASSIS_RPM_CONVERT * CHASSIS_RADIUS))

typedef struct {
    /* Body-frame velocity: right/forward in m/s, CCW in rad/s. */
    float vx_;
    float vy_;
    float vw_;
} ChassisCmd;

typedef struct {
    ChassisCmd     target_;
    /* Board 1: FL/RL; board 2: RR/FR (motor 1/motor 2). */
    MotorCmd       motor_cmds_[CHASSIS_BOARD_COUNT];
    MotorFeedback  motor_fb_[CHASSIS_BOARD_COUNT];
    float          radius_;
    float          wheel_radius_;
} Chassis;

void chassisInit(Chassis *chassis);
void chassisSetTarget(Chassis *chassis, float vx, float vy, float vw);
void chassisUpdate(Chassis *chassis);
void chassisEmergencyStop(Chassis *chassis);
void chassisSendCmd(const Chassis *chassis);
void chassisProcessFeedback(Chassis *chassis, uint32_t can_id, const uint8_t *data);

#endif /* __APP_CHASSIS_H */
