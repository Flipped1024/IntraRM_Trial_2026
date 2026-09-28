#include "bsp_tim.h"

static uint32_t last_control_tick_ = 0;

void bspTimInit(void)
{
    last_control_tick_ = HAL_GetTick();
}

uint8_t bspTimControlPeriodElapsed(void)
{
    uint32_t now = HAL_GetTick();

    if (now - last_control_tick_ >= CONTROL_PERIOD_MS) {
        last_control_tick_ = now;
        return 1;
    }
    return 0;
}

uint32_t bspTimGetTick(void)
{
    return HAL_GetTick();
}