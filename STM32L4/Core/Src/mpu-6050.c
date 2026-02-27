#include "mpu-6050.h"
#include <math.h>

#define I2C_ADDRESS_7B      (uint16_t) 0x68U //0x69 - the other available address for MPU6050
#define I2C_ADDRESS_HAL     (I2C_ADDRESS_7B << 1)
#define MPU6050_REG_SIZE    I2C_MEMADD_SIZE_8BIT
#define I2C_TIMEOUT         10U     /*  ms  */
#define PAYLOAD_SIZE        14U
#define SELFTEST_PAYLOAD    12U
/*  Sensor register addresses  */

/*  Registers  */
#define SELF_TEST_X         0x0DU
#define SELF_TEST_Y         0x0EU
#define SELF_TEST_Z         0x0FU
#define SELF_TEST_A         0x10U

#define SMPLRT_DIV_REG      0x19U
#define CONFIG_REG          0x1AU
#define GYRO_CONFIG_REG     0x1BU
#define ACCEL_CONFIG_REG    0x1CU
#define FIFO_EN_REG         0x23U
#define INT_PIN_CFG_REG     0x37U
#define INT_ENABLE_REG      0x38U
#define USER_CTRL_REG       0x6AU

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

/*  Some default register values  */

#define SMPLTR_DIV_VAL_DEFAULT      0x09U   /* 0000 1001 */
/* Sample Rate = GyroRate / (1 + SMPLRT_DIV)
 * GyroRate = 1kHz (DLPF on) or 8kHz (DLPF off) */

#define CONFIG_VAL_DEFAULT          0x04U   /* 0000 0100 */
#define GYRO_CONFIG_VAL_DEFAULT     0x08U   /* 0000 1000 */
#define ACCEL_CONFIG_VAL_DEFAULT    0x08U   /* 0000 1000 */
#define FIFO_EN_VAL_DEFAULT         0xF8U   /* 1111 1000 */
#define INT_PIN_CFG_VAL_DEFAULT     0x20U   /* 0010 0000 */
#define INT_ENABLE_VAL_DEFAULT      0x11U   /* 0001 0001 */
#define USER_CTRL_VAL_DEFAULT       0x00U   /* 0000 0000 */

#define GYRO_RANGE_POS      3U
#define GYRO_RANGE          (3U << GYRO_RANGE_POS)

#define ACCEL_RANGE_POS     3U
#define ACCEL_RANGE         (3U << ACCEL_RANGE_POS)

#define STATUS_CHECK(status) (status != HAL_OK)

/* local strucutres definitions */

typedef struct {
    uint8_t addr;
    uint8_t val;
} reg_t;

/* static functions declarations */

static MPU6050_selftest_t calculate_ft(const uint8_t gyro[3], const uint8_t accel[3]);
static void parse_payload_selftest(const uint8_t raw[12], int16_t *inter);
static inline float selftest_ratio(int16_t diff, float ft);
static inline int16_t conv_to_i16(uint8_t msb, uint8_t lsb);

static const reg_t init_registers[] = {
    { PWR_MGMT_1,        0x00U},
    { SMPLRT_DIV_REG,    SMPLTR_DIV_VAL_DEFAULT },
    { CONFIG_REG,        CONFIG_VAL_DEFAULT },
    { GYRO_CONFIG_REG,   GYRO_CONFIG_VAL_DEFAULT },
    { ACCEL_CONFIG_REG,  ACCEL_CONFIG_VAL_DEFAULT },
    { FIFO_EN_REG,       FIFO_EN_VAL_DEFAULT },
    { INT_ENABLE_REG,    INT_ENABLE_VAL_DEFAULT },
    { USER_CTRL_REG,     USER_CTRL_VAL_DEFAULT }
};

HAL_StatusTypeDef MPU_6050_Init(MPU6050_t *handles) {
    HAL_StatusTypeDef status = HAL_OK;
    for(size_t i = 0; i < sizeof(init_registers)/sizeof(init_registers[0]); i++) {
        uint8_t v = init_registers[i].val;
        status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL, init_registers[i].addr, MPU6050_REG_SIZE, &v, 1, I2C_TIMEOUT);
        if(STATUS_CHECK(status)) return status;
    }
    return HAL_OK;
}

HAL_StatusTypeDef MPU_6050_Self_Test(MPU6050_t *handles, MPU6050_selftest_t *result) {
    if(result == NULL) return HAL_ERROR;
    HAL_StatusTypeDef status = HAL_OK;

    /* During self test procedure the sensor must remain stationary to get a valid result */

    uint8_t gyro_config = GYRO_CONFIG_VAL_DEFAULT;
    uint8_t accel_config = ACCEL_CONFIG_VAL_DEFAULT;
    uint8_t gyro_config_test = 0U;   // set gyroscope range to +-250dps for selftest
    uint8_t accel_config_test = 1U << 4; //set accelerometer range to +-8g for selftest

    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL, GYRO_CONFIG_REG, MPU6050_REG_SIZE, &gyro_config_test, 1, I2C_TIMEOUT);
    if(STATUS_CHECK(status)) return status;
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL, ACCEL_CONFIG_REG, MPU6050_REG_SIZE, &accel_config_test, 1, I2C_TIMEOUT);
    if(STATUS_CHECK(status)) return status;
    HAL_Delay(50);
    
    uint8_t test_raw[4];
    uint8_t gyro_test[3];
    uint8_t accel_test[3];

    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL, SELF_TEST_X, MPU6050_REG_SIZE, test_raw, 4, I2C_TIMEOUT);
    if(STATUS_CHECK(status)) return status;

    enum{x = 0, y = 1, z = 2, a = 3};
    accel_test[x] = ((test_raw[x] & 0xE0U) >> 3) | ((test_raw[a] & 0x30U) >> 4);
    accel_test[y] = ((test_raw[y] & 0xE0U) >> 3) | ((test_raw[a] & 0x0CU) >> 2);
    accel_test[z] = ((test_raw[z] & 0xE0U) >> 3) | (test_raw[a] & 0x03U);
    gyro_test[x] = test_raw[x] & 0x1FU;
    gyro_test[y] = test_raw[y] & 0x1FU;
    gyro_test[z] = test_raw[z] & 0x1FU;

    MPU6050_selftest_t ft = calculate_ft(gyro_test, accel_test);
    
    uint8_t test_dis_data_raw[SELFTEST_PAYLOAD];
    uint8_t test_en_data_raw[SELFTEST_PAYLOAD];
    int16_t test_dis_data_numerical[SELFTEST_PAYLOAD/2];
    int16_t test_en_data_numerical[SELFTEST_PAYLOAD/2];

    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL, ACCEL_XOUT_H, MPU6050_REG_SIZE, test_dis_data_raw, SELFTEST_PAYLOAD/2, I2C_TIMEOUT);
    if(STATUS_CHECK(status)) return status;
    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL, GYRO_XOUT_H, MPU6050_REG_SIZE, test_dis_data_raw + SELFTEST_PAYLOAD/2, SELFTEST_PAYLOAD/2, I2C_TIMEOUT);
    if(STATUS_CHECK(status)) return status;

    gyro_config_test |= 7U << 5;
    accel_config_test |= 7U << 5;

    /* enable selftest */
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL, GYRO_CONFIG_REG, MPU6050_REG_SIZE, &gyro_config_test, 1, I2C_TIMEOUT);
    if(STATUS_CHECK(status)) return status;
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL, ACCEL_CONFIG_REG, MPU6050_REG_SIZE, &accel_config_test, 1, I2C_TIMEOUT);
    if(STATUS_CHECK(status)) return status;
    HAL_Delay(50);

    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL, ACCEL_XOUT_H, MPU6050_REG_SIZE, test_en_data_raw, SELFTEST_PAYLOAD/2, I2C_TIMEOUT);
    if(STATUS_CHECK(status)) return status;
    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL, GYRO_XOUT_H, MPU6050_REG_SIZE, test_en_data_raw + SELFTEST_PAYLOAD/2, SELFTEST_PAYLOAD/2, I2C_TIMEOUT);
    if(STATUS_CHECK(status)) return status;

    /* disable selftest, restore prevoius scale */
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL, GYRO_CONFIG_REG, MPU6050_REG_SIZE, &gyro_config, 1, I2C_TIMEOUT);
    if(STATUS_CHECK(status)) return status;
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL, ACCEL_CONFIG_REG, MPU6050_REG_SIZE, &accel_config, 1, I2C_TIMEOUT);
    if(STATUS_CHECK(status)) return status;

    parse_payload_selftest(test_dis_data_raw, test_dis_data_numerical);
    parse_payload_selftest(test_en_data_raw, test_en_data_numerical);

    struct {
        int16_t accel_x;
        int16_t accel_y;
        int16_t accel_z;
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;
    } diff;

    diff.accel_x = test_en_data_numerical[0] - test_dis_data_numerical[0];
    diff.accel_y = test_en_data_numerical[1] - test_dis_data_numerical[1];
    diff.accel_z = test_en_data_numerical[2] - test_dis_data_numerical[2];
    diff.gyro_x  = test_en_data_numerical[3] - test_dis_data_numerical[3];
    diff.gyro_y  = test_en_data_numerical[4] - test_dis_data_numerical[4];
    diff.gyro_z  = test_en_data_numerical[5] - test_dis_data_numerical[5];

    result->accel_x = selftest_ratio(diff.accel_x, ft.accel_x);
    result->accel_y = selftest_ratio(diff.accel_y, ft.accel_y);
    result->accel_z = selftest_ratio(diff.accel_z, ft.accel_z);
    result->gyro_x  = selftest_ratio(diff.gyro_x,  ft.gyro_x);
    result->gyro_y  = selftest_ratio(diff.gyro_y,  ft.gyro_y);
    result->gyro_z  = selftest_ratio(diff.gyro_z,  ft.gyro_z);

    return HAL_OK;
}

static MPU6050_selftest_t calculate_ft(const uint8_t gyro[3], const uint8_t accel[3]) {
    float ft_raw[6];
    for(int i = 0; i < 3; i++) {
        int8_t c = (i%2) ? -1 : 1;
        if(accel[i] == 0) {
            ft_raw[i] = 0;
        }
        else {
            ft_raw[i] = 4096.0f*0.34f*powf(0.92f/0.34f, (accel[i]-1.0f)/30.0f);
        }

        if(gyro[i] == 0) {
            ft_raw[i+3] = 0;
        }
        else {
            ft_raw[i+3] = c*25.0f*131.0f*powf(1.046f, gyro[i]-1.0f);
        }
    }
    MPU6050_selftest_t ft = {
        .accel_x = ft_raw[0],
        .accel_y = ft_raw[1],
        .accel_z = ft_raw[2],
        .gyro_x  = ft_raw[3],
        .gyro_y  = ft_raw[4],
        .gyro_z  = ft_raw[5]
    };
    return ft;
}

static void parse_payload_selftest(const uint8_t raw[12], int16_t *inter) {
    for(size_t i = 0; i < 6; i++){
        *inter++ = conv_to_i16(raw[2*i], raw[2*i+1]);
    }
}

static inline float selftest_ratio(int16_t diff, float ft) {
    return (ft == 0.0f) ? 0.0f : (((float)diff - ft) / ft)*100.0f /* result in % */;
}

HAL_StatusTypeDef MPU6050_Set_Gyro_Range(MPU6050_t *handles, gyro_range_t range) {
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t reg = 0;
    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL, GYRO_CONFIG_REG, MPU6050_REG_SIZE, &reg, 1, I2C_TIMEOUT);
    if(STATUS_CHECK(status)) return status;
    reg &= ~(GYRO_RANGE);
    reg |= range << GYRO_RANGE_POS;
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL, GYRO_CONFIG_REG, MPU6050_REG_SIZE, &reg, 1, I2C_TIMEOUT);

    return status;
}

HAL_StatusTypeDef MPU6050_Set_Accel_Range(MPU6050_t *handles, accel_range_t range) {
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t reg = 0;
    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL, ACCEL_CONFIG_REG, MPU6050_REG_SIZE, &reg, 1, I2C_TIMEOUT);
    if(STATUS_CHECK(status)) return status;
    reg &= ~(ACCEL_RANGE);
    reg |= range << ACCEL_RANGE_POS;
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL, ACCEL_CONFIG_REG, MPU6050_REG_SIZE, &reg, 1, I2C_TIMEOUT);

    return status;
}

HAL_StatusTypeDef MPU_6050_Single_Read(MPU6050_t *handles) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(handles->hi2c, I2C_ADDRESS_HAL, ACCEL_XOUT_H, MPU6050_REG_SIZE, handles->rx_buffer, PAYLOAD_SIZE);
    if(STATUS_CHECK(status)) return status;
    
    return HAL_OK;
}

static inline int16_t conv_to_i16(uint8_t msb, uint8_t lsb) {
    uint16_t u = ((uint16_t)msb << 8) | (uint16_t)lsb;
    return (int16_t)u;
}

void MPU_6050_parse_payload(const uint8_t raw[14], int16_t *inter) {
    for(size_t i = 0; i < 7; i++){
        *inter++ = conv_to_i16(raw[2*i], raw[2*i+1]);
    }
}

MPU6050_data_t MPU6050_payload_to_readable(const int16_t payload[7]) {
    MPU6050_data_t readable;
    readable.accel_x = payload[0]/8192.0f;
    readable.accel_y = payload[1]/8192.0f;
    readable.accel_z = payload[2]/8192.0f;
    readable.temp = (payload[3]/340.0f) + 35.0f;
    readable.gyro_x = payload[4]/65.5f;
    readable.gyro_y = payload[5]/65.5f;
    readable.gyro_z = payload[6]/65.5f;

    return readable;
}