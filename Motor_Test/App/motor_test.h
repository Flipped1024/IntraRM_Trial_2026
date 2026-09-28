#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

#include <stdint.h>

#define MOTOR_TEST_MOTOR_COUNT 2U

/*
 * This object is intentionally simple and volatile so it can be watched in
 * Ozone while the board runs without a CAN connection.
 */
typedef struct
{
    uint32_t cycle_time_ms;
    uint32_t cycle_count;
    uint8_t phase;
    uint8_t encoder_seen_mask;
    int16_t motor1_delta_pulse;
    int16_t motor2_delta_pulse;
    int32_t motor1_total_pulse;
    int32_t motor2_total_pulse;
    int32_t motor1_pwm_command;
    int32_t motor2_pwm_command;
} MotorTestDebugData;

extern volatile MotorTestDebugData motor_test_debug;

void appMotorTestInit(void);
void appMotorTestLoop(void);

#endif /* MOTOR_TEST_H */
