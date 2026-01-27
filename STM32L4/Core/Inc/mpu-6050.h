#ifndef MPU_6050_H
#define MPU_6050_H

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t *tx_buffer;
    uint8_t *rx_buffer;
} MPU6050_t;

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float temp;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} MPU6050_data_t;

void MPU_6050_Init(MPU6050_t*);
void MPU_6050_Single_Read(MPU6050_t*);
void MPU_6050_parse_payload(const uint8_t raw[14], int16_t *inter);
MPU6050_data_t MPU6050_payload_to_readable(const int16_t payload[7]);
#endif