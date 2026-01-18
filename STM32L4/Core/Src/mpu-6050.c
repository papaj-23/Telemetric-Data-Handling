#include "stm32l4xx_hal.h"
#include "mpu-6050.h"

#define I2C_ADDRESS_7B      (uint16_t) 0x68U //0x69
#define I2C_ADDRESS_HAL     (I2C_ADDRESS_7B << 1)
#define MPU6050_REG_SIZE    I2C_MEMADD_SIZE_8BIT
#define I2C_TIMEOUT         10U     /*  ms  */
#define PAYLOAD_SIZE        14U
/*  Sensor register addresses  */

/*  Registers to be initially configured  */
#define SMPLRT_DIV_REG      0x19U
#define SMPLTR_DIV_VAL      0x09U	/* 0000 1001 */
#define CONFIG_REG          0x1AU
#define CONFIG_VAL          0x04U	/* 0000 0100 */
#define GYRO_CONFIG_REG     0x1BU
#define GYRO_CONFIG_VAL     0x08U	/* 0000 1000 */
#define ACCEL_CONFIG_REG    0x1CU
#define ACCEL_CONFIG_VAL    0x08U	/* 0000 1000 */
#define FIFO_EN_REG         0x23U
#define FIFO_EN_VAL         0xF8U	/* 1111 1000 */
#define INT_PIN_CFG_REG     0x37U   
#define INT_PIN_CFG_VAL     0x20U   /* 0010 0000 */
#define INT_ENABLE_REG      0x38U
#define INT_ENABLE_VAL      0x11U   /* 0001 0001 */
#define USER_CTRL_REG       0x6AU
#define USER_CTRL_VAL       0x00U   /* 0000 0000 */

/*  Registers for runtime  */
#define INT_STATUS          0x3AU

#define ACCEL_XOUT_H        0x3BU
#define ACCEL_XOUT_L        0x3CU
#define ACCEL_YOUT_H        0x3DU
#define ACCEL_YOUT_L        0x3EU
#define ACCEL_ZOUT_H        0x3FU
#define ACCEL_ZOUT_L        0x40U

#define TEMP_OUT_H          0x41U
#define TEMP_OUT_L          0x42U

#define GYRO_XOUT_H         0x43U
#define GYRO_XOUT_L         0x44U
#define GYRO_YOUT_H         0x45U
#define GYRO_YOUT_L         0x46U
#define GYRO_ZOUT_H         0x47U
#define GYRO_ZOUT_L         0x48U

#define PWR_MGMT_1          0x6BU
#define PWR_MGMT_2          0x6CU
#define FIFO_COUNT_H        0X72U
#define FIFO_COUNT_L        0X73U
#define FIFO_R_W            0x74U
#define WHO_AM_I            0x75U

typedef struct {
    uint8_t addr;
    uint8_t val;
} reg_t;

static const reg_t init_registers[] = {
    { PWR_MGMT_1,        0x00U},
    { SMPLRT_DIV_REG,    SMPLTR_DIV_VAL },
    { CONFIG_REG,        CONFIG_VAL },
    { GYRO_CONFIG_REG,   GYRO_CONFIG_VAL },
    { ACCEL_CONFIG_REG,  ACCEL_CONFIG_VAL },
    { FIFO_EN_REG,       FIFO_EN_VAL },
    { INT_ENABLE_REG,    INT_ENABLE_VAL },
    { USER_CTRL_REG,  USER_CTRL_VAL }
};

void MPU_6050_Init(MPU6050_t *handles) {
    for(size_t i = 0; i < sizeof(init_registers)/sizeof(reg_t); i++) {
        HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL, init_registers[i].addr, MPU6050_REG_SIZE, &init_registers[i].val, 1, I2C_TIMEOUT);
    }
}

void MPU_6050_Single_Read(MPU6050_t *handles) {
        HAL_I2C_Mem_Read_DMA(handles->hi2c, I2C_ADDRESS_HAL, (uint16_t)ACCEL_XOUT_H, MPU6050_REG_SIZE, handles->rx_buffer ,PAYLOAD_SIZE);
  
}



