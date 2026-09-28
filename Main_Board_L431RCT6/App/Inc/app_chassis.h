#ifndef __APP_CHASSIS_H
#define __APP_CHASSIS_H

#include "main.h"
#include "app_protocol.h"

#define CHASSIS_MOTOR_COUNT       4
#define CHASSIS_BOARD_COUNT       2
#define CHASSIS_RADIUS            0.17888f
#define CHASSIS_WHEEL_RADIUS      0.0765f
#define CHASSIS_MAX_SPEED_RPM     178.0f
#define CHASSIS_RPM_CONVERT       60.0f
#define CHASSIS_STICK_RANGE       127.0f

/* Full-scale joystick commands are chosen so one pure motion reaches the
 * measured 178 rpm wheel-speed limit.  Combined vx/vy/vw commands are
 * proportionally limited again in chassisUpdate(). */
#define CHASSIS_MAX_LINEAR_SPEED_MPS \
    (CHASSIS_MAX_SPEED_RPM * CHASSIS_WHEEL_RADIUS / \
     (CHASSIS_RPM_CONVERT * 0.70710678f))
#define CHASSIS_MAX_ANGULAR_SPEED_RADPS \
    (CHASSIS_MAX_SPEED_RPM * CHASSIS_WHEEL_RADIUS / \
     (CHASSIS_RPM_CONVERT * CHASSIS_RADIUS))

typedef struct {
    float vx_;
    float vy_;
    float vw_;
} ChassisCmd;

typedef struct {
    ChassisCmd     target_;
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
