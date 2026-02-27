#ifndef MPU_6050_H
#define MPU_6050_H

#include "stm32l4xx_hal.h"

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t *tx_buffer;
    uint8_t *rx_buffer;
} MPU6050_t;

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} MPU6050_selftest_t;

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float temp;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} MPU6050_data_t;

typedef enum {
    DPS_250 = 0U,
    DPS_500 = 1U,
    DPS_1000 = 2U,
    DPS_2000 = 3U
} gyro_range_t;

typedef enum {
    G_2 = 0U,
    G_4 = 1U,
    G_8 = 2U,
    G_16 = 3U
} accel_range_t;

HAL_StatusTypeDef MPU_6050_Init(MPU6050_t *handles);
HAL_StatusTypeDef MPU_6050_Self_Test(MPU6050_t *handles, MPU6050_selftest_t *result);
HAL_StatusTypeDef MPU6050_Set_Gyro_Range(MPU6050_t *handles, gyro_range_t range);
HAL_StatusTypeDef MPU6050_Set_Accel_Range(MPU6050_t *handles, accel_range_t range);
HAL_StatusTypeDef MPU_6050_Single_Read(MPU6050_t *handles);
void MPU_6050_parse_payload(const uint8_t raw[14], int16_t *inter);
MPU6050_data_t MPU6050_payload_to_readable(const int16_t payload[7]);
#endif