#ifndef MPU_6050_H
#define MPU_6050_H

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t *tx_buffer;
    uint8_t *rx_buffer;
} MPU6050_t;

void MPU_6050_Init(MPU6050_t*);
void MPU_6050_Single_Read(MPU6050_t*);
#endif