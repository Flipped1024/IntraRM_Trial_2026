#ifndef APP_DEBUG_H
#define APP_DEBUG_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

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

typedef struct DebugCanData {
    volatile uint32_t state;
    volatile uint32_t error;
    volatile uint32_t rx_count;
    volatile uint32_t rx_error_count;
    volatile uint32_t last_rx_id;
    volatile uint8_t last_rx_data[8];
    volatile uint32_t tx_count;
    volatile uint32_t tx_ok_count;
    volatile uint32_t tx_fail_count;
    volatile uint32_t tx_mailbox_free;
    volatile uint8_t init_ok;
} DebugCanData;

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
    DebugCanData can;
} DebugData;

extern volatile DebugData debug_data;

void appDebugInit(void);
void appDebugUpdate(void);
void appDebugCanState(CAN_HandleTypeDef *hcan);
void appDebugCanRx(uint32_t id, const uint8_t *data);
void appDebugCanRxError(void);
void appDebugCanTx(uint32_t status);
void appDebugCanInitResult(HAL_StatusTypeDef filter_status,
                           HAL_StatusTypeDef start_status,
                           HAL_StatusTypeDef notification_status);

#endif /* APP_DEBUG_H */
