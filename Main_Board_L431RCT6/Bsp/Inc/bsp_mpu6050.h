#ifndef __BSP_MPU6050_H
#define __BSP_MPU6050_H

#include "main.h"

typedef struct {
    int16_t accel[3];
    int16_t gyro[3];
    int16_t temperature;
} Mpu6050Raw;

HAL_StatusTypeDef mpu6050Init(void);
HAL_StatusTypeDef mpu6050ReadRaw(Mpu6050Raw *raw);
void mpu6050Update(void);
void mpu6050SetTemperatureTarget(float target_c);

#endif
