#pragma once

#include "stm32f0xx_hal.h"

#define PI 3.141592653589793
#define RAD_TO_DEG 57.29577951308232087
#define DEG_TO_RAD 0.017453292519943295

// MPU6050 structure
typedef struct
{
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    float Ax;
    float Ay;
    float Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    float Gx;
    float Gy;
    float Gz;

    int16_t Temp_RAW;
    float Temperature = 29.9829407;

    float Gx_bias = -0.0395212956;
    float Gy_bias = -0.0430579931;
    float Gz_bias = -0.000511327176;

} MPU6050_t;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);

int MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

int MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

int MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

int MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Calc_Gyro_Bias(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Calc_Accel(MPU6050_t *DataStruct);

void MPU6050_Calc_Temp(MPU6050_t *DataStruct);

void MPU6050_Calc_Gyro(MPU6050_t *DataStruct);

void MPU6050_Calc_All(MPU6050_t *DataStruct);
