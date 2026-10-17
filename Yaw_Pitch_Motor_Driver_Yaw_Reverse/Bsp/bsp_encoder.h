/* bsp_encoder.h */
#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include "stm32f1xx_hal.h"
#include "tim.h"
#include <stdint.h>

#define ENCODER_RESOLUTION 65536
#define ENCODER_HALF_RESOLUTION 32768

#define MOTOR_PITCH_ID 1
#define MOTOR_YAW_ID 2
#define ENCODER_NUM 2

typedef struct {
    TIM_HandleTypeDef* htim_;
    int16_t delta_pulse_; 
    int32_t total_pulse_; 
    uint16_t last_cnt_;
} EncoderInstance;

void bspEncoderInit(void);
void bspEncoderUpdate(uint8_t motor_id);

int16_t bspGetEncoderDeltaPulse(uint8_t motor_id);
int32_t bspGetEncoderTotalPulse(uint8_t motor_id);

#endif /* BSP_ENCODER_H */
