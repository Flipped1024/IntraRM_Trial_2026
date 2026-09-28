#ifndef __APP_DEBUG_H
#define __APP_DEBUG_H

#include "main.h"
#include "can.h"
#include "app_chassis.h"

typedef enum {
    DEBUG_MODE_NONE    = 0,
    DEBUG_MODE_CHASSIS = 1,
    DEBUG_MODE_CAN     = 2,
} DebugMode;

typedef struct {
    volatile float vx_;
    volatile float vy_;
    volatile float vw_;
} DebugChassis;

typedef struct {
    volatile int16_t motor_rpm_[4];
    volatile uint32_t tx_count_;
    volatile uint32_t tx_ok_count_;
    volatile uint32_t tx_fail_count_;
    volatile uint32_t rx_count_;
    volatile uint32_t last_rx_id_;
    volatile uint8_t  last_rx_data_[8];
    volatile uint32_t can_state_;
    volatile uint32_t can_error_;
    volatile uint32_t can_tx_mailbox_free_;
    volatile uint8_t  can_init_ok_;
} DebugCan;

typedef struct {
    volatile DebugMode mode_;
    DebugChassis       chassis_;
    DebugCan           can_;
} DebugData;

extern DebugData g_debug_;

void debugInit(void);
void debugUpdateCanTx(HAL_StatusTypeDef status);
void debugUpdateCanRx(uint32_t can_id, const uint8_t *data);
void debugUpdateCanState(void);
void debugRunChassis(Chassis *chassis);
void debugRunCanDirect(void);

#endif /* __APP_DEBUG_H */