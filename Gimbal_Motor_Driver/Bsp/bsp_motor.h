#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "stm32f1xx_hal.h"
#include "tim.h"

#define MAX_PWM_VALUE 999  /* TIM ARR = 1000 - 1 */
typedef enum {
    MOTOR_370_FRIC1 = 0,
    MOTOR_370_FRIC2,
    MOTOR_N20_FEED,
    MOTOR_NUM
} MotorIdType;

typedef enum {
    MOTOR_MODE_DUAL_PWM = 0,
    MOTOR_MODE_SINGLE_PWM
} MotorPwmModeType;

typedef struct {
    TIM_HandleTypeDef* htim_;
    MotorPwmModeType pwm_mode_;
    uint32_t channel_in1_;
    uint32_t channel_in2_;
    GPIO_TypeDef* dir_port_;
    uint16_t dir_pin_;
    int32_t current_duty_;
    uint8_t direction_invert_;
} MotorHandleType;

void bspMotorInit(void);
void bspMotorSetDuty(MotorIdType motor_id, int32_t duty);
void bspMotorStop(MotorIdType motor_id);
int32_t bspMotorGetDuty(MotorIdType motor_id);

#endif /* __BSP_MOTOR_H */
