#ifndef MPU_6050_H
#define MPU_6050_H

#include "stm32l4xx_hal.h"

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t *tx_buffer;
    uint8_t *rx_buffer;
    void (*delay_ms_wrapper)(uint32_t);
    uint8_t *fifo_counter_raw;
    uint16_t burst_count;
    uint16_t fifo_counter;
    uint8_t gyro_scale;
    uint8_t accel_scale;
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
} MPU_6050_gyro_range_t;

typedef enum {
    G_2 = 0U,
    G_4 = 1U,
    G_8 = 2U,
    G_16 = 3U
} MPU_6050_accel_range_t;

typedef enum {
    FIFO_ACCEL = 8U,
    FIFO_GYRO_Z = 16U,
    FIFO_GYRO_Y = 32U,
    FIFO_GYRO_X = 64U,
    FIFO_TEMP = 128U
} MPU_6050_fifo_content_t;

typedef enum {
    MPU_DISABLE = 0U,
    MPU_ENABLE = !MPU_DISABLE
} MPU_6050_state_t;

typedef enum {
    MPU_SINGLE_MODE = 0U,
    MPU_BURST_MODE = 1U,
} MPU_6050_mode_t;

HAL_StatusTypeDef MPU_6050_Init(MPU6050_t *handles);
HAL_StatusTypeDef MPU_6050_Set_Mode(MPU6050_t *handles, MPU_6050_mode_t mode);
HAL_StatusTypeDef MPU_6050_FIFO_Reset(MPU6050_t *handles);
HAL_StatusTypeDef MPU_6050_Self_Test(MPU6050_t *handles, MPU6050_selftest_t *result);
HAL_StatusTypeDef MPU6050_Set_Gyro_Range(MPU6050_t *handles, MPU_6050_gyro_range_t range);
HAL_StatusTypeDef MPU6050_Set_Accel_Range(MPU6050_t *handles, MPU_6050_accel_range_t range);
HAL_StatusTypeDef MPU6050_Set_FIFO_Content(MPU6050_t *handles, MPU_6050_fifo_content_t content, MPU_6050_state_t state);
HAL_StatusTypeDef MPU_6050_Single_Read(MPU6050_t *handles);
HAL_StatusTypeDef MPU_6050_Read_FIFO_Cnt(MPU6050_t *handles);
void MPU_6050_Process_Burst_Cnt(MPU6050_t *handles, const uint8_t raw[2]);
HAL_StatusTypeDef MPU_6050_Burst_Read(MPU6050_t *handles);
void MPU_6050_parse_payload(const uint8_t raw[14], int16_t *inter);
MPU6050_data_t MPU6050_payload_to_readable(MPU6050_t *handles, const int16_t payload[7]);
#endif