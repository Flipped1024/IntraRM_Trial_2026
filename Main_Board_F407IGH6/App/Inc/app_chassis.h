#ifndef __APP_CHASSIS_H
#define __APP_CHASSIS_H

#include "main.h"
#include "app_protocol.h"

#define CHASSIS_MOTOR_COUNT       4
#define CHASSIS_BOARD_COUNT       2
#define CHASSIS_RADIUS            0.17888f
#define CHASSIS_WHEEL_RADIUS      0.0765f
#define CHASSIS_MAX_SPEED_RPM     12000.0f
#define CHASSIS_RPM_CONVERT       60.0f

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
void chassisSendCmd(const Chassis *chassis);
void chassisProcessFeedback(Chassis *chassis, uint32_t can_id, const uint8_t *data);

#endif /* __APP_CHASSIS_H */