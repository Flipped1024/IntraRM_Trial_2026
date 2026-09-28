#ifndef __BSP_TIM_H
#define __BSP_TIM_H

#include "main.h"

#define CONTROL_PERIOD_MS  10

void     bspTimInit(void);
uint8_t  bspTimControlPeriodElapsed(void);
uint32_t bspTimGetTick(void);

#endif /* __BSP_TIM_H */