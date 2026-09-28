#include "app_control.h"
#include "app_debug.h"
#include "bsp_motor.h"

void appControlInit(void)
{
    bspMotorInit();
}

void appControlLoop(void)
{
    if (debug_data.debug_mode)
    {
        bspMotorSetDuty(MOTOR_370_FRIC1, debug_data.fric1_duty);
        bspMotorSetDuty(MOTOR_370_FRIC2, debug_data.fric2_duty);
        bspMotorSetDuty(MOTOR_N20_FEED, debug_data.feed_duty);
    }
    else
    {
        debug_data.fric1_duty = bspMotorGetDuty(MOTOR_370_FRIC1);
        debug_data.fric2_duty = bspMotorGetDuty(MOTOR_370_FRIC2);
        debug_data.feed_duty = bspMotorGetDuty(MOTOR_N20_FEED);
    }
}
