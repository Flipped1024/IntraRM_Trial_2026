#ifndef APP_DEBUG_H
#define APP_DEBUG_H

#include <stdint.h>

#define DEBUG_MOTOR_NUM 2

typedef enum {
    DEBUG_MODE_CAN = 0U,
    DEBUG_MODE_MANUAL_TARGET = 1U
} DebugMode;

typedef struct DebugMotorData {
    float target_speed_rpm;
    float current_speed_rpm;
    float raw_speed_rpm;
    int16_t delta_pulse;
    float speed_error;
    float speed_output;
    float feedforward_output;
    int32_t motor_duty;
    int32_t total_pulse;
    float debug_target_speed_rpm;
} DebugMotorData;

typedef struct DebugData {
    float spd_kp;
    float spd_ki;
    float spd_kd;
    float spd_max_out;
    float spd_max_integral;
    float spd_deadband;
    float spd_filter_alpha;
    float spd_ff_accel;

    uint8_t debug_mode;
    DebugMotorData motor[DEBUG_MOTOR_NUM];
} DebugData;

extern volatile DebugData debug_data;

void appDebugInit(void);
void appDebugUpdate(void);

#endif /* APP_DEBUG_H */
