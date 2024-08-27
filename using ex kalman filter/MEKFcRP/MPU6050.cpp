#include <MPU6050.hpp>

#define MPU6050_ADDR 0xD0 // 0x68 << 1
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43
#define DLPF_CONFIG_REG 0x1A

#define I2C_TIMEOUT 1000
#define GYRO_SCALE 2000.0*DEG_TO_RAD/(1<<15)
#define ACCE_SCALE 16.0/(1<<15)


uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check;
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, I2C_TIMEOUT);

    if(check != 0x68){
    	return -1;
    }

    uint8_t Data;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &(Data=0x00), 1, I2C_TIMEOUT);
	HAL_Delay(100);

	// if dlpf is enabled => Gyroscope Output Rate = 1khz
//	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &(Data=0x07), 1, I2C_TIMEOUT);
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &(Data=0x00), 1, I2C_TIMEOUT);

	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, I2C_TIMEOUT);
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &(Data|=0b000'11'000), 1, I2C_TIMEOUT);

	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, I2C_TIMEOUT);
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &(Data|=0b000'11'000), 1, I2C_TIMEOUT);

	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, DLPF_CONFIG_REG, 1, &Data, 1, I2C_TIMEOUT);
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, DLPF_CONFIG_REG, 1, &(Data|=0b00000'010), 1, I2C_TIMEOUT);

	HAL_Delay(100);
    return 0;
}

int MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];
    auto status = HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, I2C_TIMEOUT);

    if(status != HAL_OK){
    	return -1;
    }

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    MPU6050_Calc_Accel(DataStruct);
    return 0;
}

int MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];
    auto status = HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, I2C_TIMEOUT);

    if(status != HAL_OK){
    	return -1;
    }

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    MPU6050_Calc_Gyro(DataStruct);
    return 0;
}

int MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[2];
    auto status = HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, I2C_TIMEOUT);

    if(status != HAL_OK){
    	return -1;
    }

    DataStruct->Temp_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);

    MPU6050_Calc_Temp(DataStruct);
    return 0;
}

int MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[14];
    auto status = HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, I2C_TIMEOUT);

    if(status != HAL_OK){
    	return -1;
    }

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    DataStruct->Temp_RAW = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    if(DataStruct->Temp_RAW==0){
    	return -1;
    }

    MPU6050_Calc_All(DataStruct);
    return 0;
}

void MPU6050_Calc_Gyro(MPU6050_t *DataStruct){
    DataStruct->Gx = DataStruct->Gyro_X_RAW * GYRO_SCALE - DataStruct->Gx_bias;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW * GYRO_SCALE - DataStruct->Gy_bias;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW * GYRO_SCALE - DataStruct->Gz_bias;
}

void MPU6050_Calc_Temp(MPU6050_t *DataStruct){
    DataStruct->Temperature = (float)((int16_t)DataStruct->Temp_RAW / (float)340.0 + (float)36.53);
}

void MPU6050_Calc_Accel(MPU6050_t *DataStruct){
    DataStruct->Ax = DataStruct->Accel_X_RAW * ACCE_SCALE;
    DataStruct->Ay = DataStruct->Accel_Y_RAW * ACCE_SCALE;
    DataStruct->Az = DataStruct->Accel_Z_RAW * ACCE_SCALE;
}

void MPU6050_Calc_All(MPU6050_t *DataStruct){
	MPU6050_Calc_Accel(DataStruct);
	MPU6050_Calc_Temp(DataStruct);
	MPU6050_Calc_Gyro(DataStruct);
}

void MPU6050_Calc_Gyro_Bias(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct){

	const uint16_t T = 50;

	float x_bias = 0;
	float y_bias = 0;
	float z_bias = 0;

	DataStruct->Gx_bias = 0;
	DataStruct->Gy_bias = 0;
	DataStruct->Gz_bias = 0;

	for(int i = 0; i < T; i++){
		MPU6050_Read_Gyro(I2Cx, DataStruct);
		x_bias += DataStruct->Gx;
		y_bias += DataStruct->Gy;
		z_bias += DataStruct->Gz;
		HAL_Delay(10);
	}

	DataStruct->Gx_bias = (x_bias / T);
	DataStruct->Gy_bias = (y_bias / T);
	DataStruct->Gz_bias = (z_bias / T);
}
