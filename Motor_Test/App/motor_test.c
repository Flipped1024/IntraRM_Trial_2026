#include "motor_test.h"

#include "bsp_encoder.h"
#include "bsp_motor.h"

#define MOTOR_TEST_PERIOD_MS       8000U
#define MOTOR_TEST_TICK_MS         10U
#define MOTOR_TEST_MAX_PWM         MAX_PWM_VALUE

#define MOTOR_TEST_STOP_1_MS       500U
#define MOTOR_TEST_FORWARD_MS      800U
#define MOTOR_TEST_STOP_2_MS       300U
#define MOTOR_TEST_REVERSE_MS      800U
#define MOTOR_TEST_STOP_3_MS       300U
#define MOTOR_TEST_RAMP_1_MS       2000U
#define MOTOR_TEST_RAMP_2_MS       2000U

enum
{
    MOTOR_TEST_PHASE_STOP_BEFORE = 0U,
    MOTOR_TEST_PHASE_FORWARD_STEP,
    MOTOR_TEST_PHASE_STOP_AFTER_FORWARD,
    MOTOR_TEST_PHASE_REVERSE_STEP,
    MOTOR_TEST_PHASE_STOP_AFTER_REVERSE,
    MOTOR_TEST_PHASE_RAMP_FORWARD_TO_REVERSE,
    MOTOR_TEST_PHASE_RAMP_REVERSE_TO_FORWARD,
    MOTOR_TEST_PHASE_STOP_END
};

volatile MotorTestDebugData motor_test_debug;

static uint32_t test_time_ms;
static uint32_t test_cycle_count;

static int32_t rampCommand(uint32_t elapsed_ms, uint32_t duration_ms,
                           int32_t start, int32_t end)
{
    int32_t delta = end - start;

    if (elapsed_ms >= duration_ms)
    {
        return end;
    }

    return start + (delta * (int32_t)elapsed_ms) / (int32_t)duration_ms;
}

static int32_t getTestCommand(uint32_t elapsed_ms, uint8_t *phase)
{
    uint32_t phase_end = MOTOR_TEST_STOP_1_MS;

    if (elapsed_ms < phase_end)
    {
        *phase = MOTOR_TEST_PHASE_STOP_BEFORE;
        return 0;
    }

    phase_end += MOTOR_TEST_FORWARD_MS;
    if (elapsed_ms < phase_end)
    {
        *phase = MOTOR_TEST_PHASE_FORWARD_STEP;
        return MOTOR_TEST_MAX_PWM;
    }

    phase_end += MOTOR_TEST_STOP_2_MS;
    if (elapsed_ms < phase_end)
    {
        *phase = MOTOR_TEST_PHASE_STOP_AFTER_FORWARD;
        return 0;
    }

    phase_end += MOTOR_TEST_REVERSE_MS;
    if (elapsed_ms < phase_end)
    {
        *phase = MOTOR_TEST_PHASE_REVERSE_STEP;
        return -MOTOR_TEST_MAX_PWM;
    }

    phase_end += MOTOR_TEST_STOP_3_MS;
    if (elapsed_ms < phase_end)
    {
        *phase = MOTOR_TEST_PHASE_STOP_AFTER_REVERSE;
        return 0;
    }

    phase_end += MOTOR_TEST_RAMP_1_MS;
    if (elapsed_ms < phase_end)
    {
        *phase = MOTOR_TEST_PHASE_RAMP_FORWARD_TO_REVERSE;
        return rampCommand(elapsed_ms - (phase_end - MOTOR_TEST_RAMP_1_MS),
                           MOTOR_TEST_RAMP_1_MS,
                           MOTOR_TEST_MAX_PWM,
                           -MOTOR_TEST_MAX_PWM);
    }

    phase_end += MOTOR_TEST_RAMP_2_MS;
    if (elapsed_ms < phase_end)
    {
        *phase = MOTOR_TEST_PHASE_RAMP_REVERSE_TO_FORWARD;
        return rampCommand(elapsed_ms - (phase_end - MOTOR_TEST_RAMP_2_MS),
                           MOTOR_TEST_RAMP_2_MS,
                           -MOTOR_TEST_MAX_PWM,
                           MOTOR_TEST_MAX_PWM);
    }

    *phase = MOTOR_TEST_PHASE_STOP_END;
    return 0;
}

void appMotorTestInit(void)
{
    bspMotorInit();
    bspEncoderInit();

    test_time_ms = 0U;
    test_cycle_count = 0U;
    motor_test_debug = (MotorTestDebugData){0};
}

void appMotorTestLoop(void)
{
    uint8_t phase = MOTOR_TEST_PHASE_STOP_BEFORE;
    int32_t command = getTestCommand(test_time_ms, &phase);

    bspEncoderUpdate(1U);
    bspEncoderUpdate(2U);

    bspMotorSetSpeed(MOTOR_1, command);
    bspMotorSetSpeed(MOTOR_2, command);

    motor_test_debug.cycle_time_ms = test_time_ms;
    motor_test_debug.cycle_count = test_cycle_count;
    motor_test_debug.phase = phase;
    motor_test_debug.motor1_delta_pulse = bspGetEncoderDeltaPulse(1U);
    motor_test_debug.motor2_delta_pulse = bspGetEncoderDeltaPulse(2U);
    motor_test_debug.motor1_total_pulse = bspGetEncoderTotalPulse(1U);
    motor_test_debug.motor2_total_pulse = bspGetEncoderTotalPulse(2U);
    motor_test_debug.motor1_pwm_command = command;
    motor_test_debug.motor2_pwm_command = command;

    if (motor_test_debug.motor1_delta_pulse != 0)
    {
        motor_test_debug.encoder_seen_mask |= 0x01U;
    }
    if (motor_test_debug.motor2_delta_pulse != 0)
    {
        motor_test_debug.encoder_seen_mask |= 0x02U;
    }

    test_time_ms += MOTOR_TEST_TICK_MS;
    if (test_time_ms >= MOTOR_TEST_PERIOD_MS)
    {
        test_time_ms = 0U;
        ++test_cycle_count;
    }
}
