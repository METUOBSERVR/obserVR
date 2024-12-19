//
// Created by ashha on 19/12/2024.
//

#include "MPU9250.h"

uint8_t RX;
uint8_t TX;

void MPU_StartUp(I2C_HandleTypeDef *hi2c){
    TX = 0x00;
    HAL_I2C_Mem_Write(hi2c, (MPU_DEV_ADDR<<1), MPU_POW_MAN_1,
                      I2C_MEMADD_SIZE_8BIT, &TX, 1, 100);
    TX = 0x00;
    HAL_I2C_Mem_Write(hi2c, (MPU_DEV_ADDR<<1), (MPU_POW_MAN_2), I2C_MEMADD_SIZE_8BIT, &TX, 1, 100);
}

void MPU_Init(I2C_HandleTypeDef *hi2c){
    // Configure interrupt
    TX = 0x00;          //Interrupt clear by reading status
    HAL_I2C_Mem_Write(hi2c, (MPU_DEV_ADDR<<1), MPU_INT_CONFIG, I2C_MEMADD_SIZE_8BIT, &TX, 1, 100);
    TX = 0x01;          //Interrupt on raw data ready
    HAL_I2C_Mem_Write(hi2c, (MPU_DEV_ADDR<<1), MPU_INT_EN, I2C_MEMADD_SIZE_8BIT, &TX, 1, 100);

    //Configure Accelerometer
    TX = 0x00;          //2g scale
    HAL_I2C_Mem_Write(hi2c, (MPU_DEV_ADDR<<1), MPU_ACC_CONF, I2C_MEMADD_SIZE_8BIT, &TX, 1, 100);
    TX = 0x00;          //Not use LPF
    HAL_I2C_Mem_Write(hi2c, (MPU_DEV_ADDR<<1), MPU_ACC_CONF2, I2C_MEMADD_SIZE_8BIT, &TX, 1, 100);

    //Configure Gyroscope
    TX = 0x00;          //250dps scale
    HAL_I2C_Mem_Write(hi2c, (MPU_DEV_ADDR<<1), MPU_GYRO_CONF, I2C_MEMADD_SIZE_8BIT, &TX, 1, 100);
    TX = 0x00;          //250Hz LPF
    HAL_I2C_Mem_Write(hi2c, (MPU_DEV_ADDR<<1), MPU_CONFIG, I2C_MEMADD_SIZE_8BIT, &TX, 1, 100);
}

void MPU_ReadAll(I2C_HandleTypeDef *hi2c, MPU_DATA *data){
    uint16_t temp;
    //Check if ready
    HAL_I2C_Mem_Read(hi2c, (MPU_DEV_ADDR<<1), MPU_INT_STATS, I2C_MEMADD_SIZE_8BIT, &RX, 1, 1);
    if(RX == 0x01) {
        //Acc X
        HAL_I2C_Mem_Read(hi2c, (MPU_DEV_ADDR << 1), MPU_ACC_XH, I2C_MEMADD_SIZE_8BIT, &RX, 1, 1);
        temp = (uint16_t) RX << 8;
        HAL_I2C_Mem_Read(hi2c, (MPU_DEV_ADDR << 1), MPU_ACC_XL, I2C_MEMADD_SIZE_8BIT, &RX, 1, 1);
        temp |= RX;
        data->ACCX = (float32_t) temp * 2000.0 / (65535.0);

        //Acc Y
        HAL_I2C_Mem_Read(hi2c, (MPU_DEV_ADDR << 1), MPU_ACC_YH, I2C_MEMADD_SIZE_8BIT, &RX, 1, 1);
        temp = (uint16_t) RX << 8;
        HAL_I2C_Mem_Read(hi2c, (MPU_DEV_ADDR << 1), MPU_ACC_YL, I2C_MEMADD_SIZE_8BIT, &RX, 1, 1);
        temp |= RX;
        data->ACCY = (float32_t) temp * 2000.0 / (65535.0);

        //Acc Z
        HAL_I2C_Mem_Read(hi2c, (MPU_DEV_ADDR << 1), MPU_ACC_ZH, I2C_MEMADD_SIZE_8BIT, &RX, 1, 1);
        temp = (uint16_t) RX << 8;
        HAL_I2C_Mem_Read(hi2c, (MPU_DEV_ADDR << 1), MPU_ACC_ZL, I2C_MEMADD_SIZE_8BIT, &RX, 1, 1);
        temp |= RX;
        data->ACCZ = (float32_t) temp * 2000.0 / (65535.0);

        //GYRO X
        HAL_I2C_Mem_Read(hi2c, (MPU_DEV_ADDR << 1), MPU_GYRO_XH, I2C_MEMADD_SIZE_8BIT, &RX, 1, 1);
        temp = (uint16_t) RX << 8;
        HAL_I2C_Mem_Read(hi2c, (MPU_DEV_ADDR << 1), MPU_GYRO_XL, I2C_MEMADD_SIZE_8BIT, &RX, 1, 1);
        temp |= RX;
        data->GYRX = (float32_t) temp * 250.0 / (65535.0);

        //GYRO Y
        HAL_I2C_Mem_Read(hi2c, (MPU_DEV_ADDR << 1), MPU_GYRO_YH, I2C_MEMADD_SIZE_8BIT, &RX, 1, 1);
        temp = (uint16_t) RX << 8;
        HAL_I2C_Mem_Read(hi2c, (MPU_DEV_ADDR << 1), MPU_GYRO_YL, I2C_MEMADD_SIZE_8BIT, &RX, 1, 1);
        temp |= RX;
        data->GYRY = (float32_t) temp * 250.0 / (65535.0);

        //GYRO Z
        HAL_I2C_Mem_Read(hi2c, (MPU_DEV_ADDR << 1), MPU_GYRO_ZH, I2C_MEMADD_SIZE_8BIT, &RX, 1, 1);
        temp = (uint16_t) RX << 8;
        HAL_I2C_Mem_Read(hi2c, (MPU_DEV_ADDR << 1), MPU_GYRO_ZL, I2C_MEMADD_SIZE_8BIT, &RX, 1, 1);
        temp |= RX;
        data->GYRZ = (float32_t) temp * 250.0 / (65535.0);
    }
}