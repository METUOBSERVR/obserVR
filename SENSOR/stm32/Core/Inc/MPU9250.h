//
// Created by ashha on 19/12/2024.
//

#ifndef STM32_MPU9250_H
#define STM32_MPU9250_H

#include "stm32f4xx_hal.h"
#include "arm_math.h"

#define MPU_DEV_ADDR    0x68
#define MPU_DEV_ID      0x70
#define MPU_WHOAMI      0x75

#define MPU_INT_CONFIG  0x37
#define MPU_INT_EN      0x38
#define MPU_INT_STATS   0x3A

#define MPU_POW_MAN_1   0x6B
#define MPU_POW_MAN_2   0x6C

#define MPU_CONFIG      0x1A
#define MPU_GYRO_CONF   0x1B
#define MPU_ACC_CONF    0x1C
#define MPU_ACC_CONF2   0x1D

#define MPU_ACC_XH      0x3B
#define MPU_ACC_XL      0x3C
#define MPU_ACC_YH      0x3D
#define MPU_ACC_YL      0x3E
#define MPU_ACC_ZH      0x3F
#define MPU_ACC_ZL      0x40

#define MPU_GYRO_XH     0x43
#define MPU_GYRO_XL     0x44
#define MPU_GYRO_YH     0x45
#define MPU_GYRO_YL     0x46
#define MPU_GYRO_ZH     0x47
#define MPU_GYRO_ZL     0x48

#define MPU_ACC_OFFSET_XH   0x77
#define MPU_ACC_OFFSET_XL   0x78
#define MPU_ACC_OFFSET_YH   0x7A
#define MPU_ACC_OFFSET_YL   0x7B
#define MPU_ACC_OFFSET_ZH   0x7D
#define MPU_ACC_OFFSET_ZL   0x7E

#define MPU_GYRO_OFFSET_XH  0x13
#define MPU_GYRO_OFFSET_XL  0x14
#define MPU_GYRO_OFFSET_YH  0x15
#define MPU_GYRO_OFFSET_YL  0x16
#define MPU_GYRO_OFFSET_ZH  0x17
#define MPU_GYRO_OFFSET_ZL  0x18

struct DATA_STRUCT{
    float32_t ACCX;
    float32_t ACCY;
    float32_t ACCZ;
    float32_t GYRX;
    float32_t GYRY;
    float32_t GYRZ;
};

typedef struct DATA_STRUCT MPU_DATA;

void MPU_StartUp(I2C_HandleTypeDef *hi2c);
void MPU_Init(I2C_HandleTypeDef *hi2c);
void MPU_ReadAll(I2C_HandleTypeDef *hi2c, MPU_DATA *data);

#endif //STM32_MPU9250_H