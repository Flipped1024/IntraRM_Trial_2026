#ifndef APP_DEBUG_H
#define APP_DEBUG_H

#include <stdint.h>

#define DEBUG_MOTOR_NUM 2

typedef struct DebugMotorData {
    int32_t target_pos_pulse;
    int32_t current_pos_pulse;
    float current_pos_rev;
    float current_pos_angle_deg;
    float pos_error;
    float pos_output;
    float target_speed_rpm;
    float current_speed_rpm;
    float raw_speed_rpm;
    int16_t delta_pulse;
    float speed_error;
    float speed_output;
    int32_t motor_duty;
    int32_t total_pulse;
    float debug_target_angle_deg;
    float debug_target_speed_rpm;
} DebugMotorData;

typedef struct DebugData {
    float pos_kp;
    float pos_ki;
    float pos_kd;
    float pos_max_out;
    float pos_max_integral;
    float spd_kp;
    float spd_ki;
    float spd_kd;
    float spd_max_out;
    float spd_max_integral;
    float spd_deadband;
    float spd_filter_alpha;

    /* 0: CAN position; 1: direct speed; 2: Ozone position. */
    uint8_t debug_mode;
    DebugMotorData motor[DEBUG_MOTOR_NUM];
} DebugData;

extern volatile DebugData debug_data;

void appDebugInit(void);
void appDebugUpdate(void);

#endif /* APP_DEBUG_H */
