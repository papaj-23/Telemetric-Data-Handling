#include "mpu-6050.h"
#include <math.h>

#define I2C_ADDRESS_7B      (uint16_t) 0x68U //0x69 - the other available address for MPU6050
#define I2C_ADDRESS_HAL     (I2C_ADDRESS_7B << 1)
#define MPU6050_REG_SIZE    I2C_MEMADD_SIZE_8BIT
#define I2C_TIMEOUT         10U     /*  ms  */
#define FULL_PAYLOAD_SIZE        14U
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
#define SIGNAL_PATH_RESET   0x68U
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
#define FIFO_EN_VAL_DEFAULT         0xF8U   /* 1111 1000 */
#define INT_PIN_CFG_VAL_DEFAULT     0x20U   /* 0010 0000 */
#define INT_ENABLE_VAL_DEFAULT      0x01U   /* 0000 0001 */
#define PWR_MGMT_1_VAL_DEFAULT      0x00U   /* 0000 0000 */     

#define GYRO_RANGE_POS      3U
#define GYRO_RANGE          ((uint8_t)(3U << GYRO_RANGE_POS))

#define ACCEL_RANGE_POS     3U
#define ACCEL_RANGE         ((uint8_t)(3U << ACCEL_RANGE_POS))   

#define DATA_READY_INT_POS  0U
#define DATA_READY_INT      ((uint8_t)(1U << DATA_READY_INT_POS))

#define FIFO_OFLOW_INT_POS  4U
#define FIFO_OFLOW_INT      ((uint8_t)(1U << FIFO_OFLOW_INT_POS))

#define FIFO_ENABLE_POS     6U
#define FIFO_ENABLE         ((uint8_t)(1U << FIFO_ENABLE_POS))

#define FIFO_RESET_POS      2U
#define FIFO_RESET          ((uint8_t)(1U << FIFO_RESET_POS))

#define LP_WAKE_CTRL_POS    6U
#define LP_WAKE_CTRL        ((uint8_t)(3U << LP_WAKE_CTRL_POS))

#define ACCEL_X_STANDBY_POS 5U
#define ACCEL_X_STANDBY     ((uint8_t)(1U << ACCEL_X_STANDBY_POS))

#define ACCEL_Y_STANDBY_POS 4U
#define ACCEL_Y_STANDBY     ((uint8_t)(1U << ACCEL_Y_STANDBY_POS))

#define ACCEL_Z_STANDBY_POS 3U
#define ACCEL_Z_STANDBY     ((uint8_t)(1U << ACCEL_Z_STANDBY_POS))

#define GYRO_X_STANDBY_POS  2U
#define GYRO_X_STANDBY      ((uint8_t)(1U << GYRO_X_STANDBY_POS))

#define GYRO_Y_STANDBY_POS  1U
#define GYRO_Y_STANDBY      ((uint8_t)(1U << GYRO_Y_STANDBY_POS))

#define GYRO_Z_STANDBY_POS  0U
#define GYRO_Z_STANDBY      ((uint8_t)(1U << GYRO_Z_STANDBY_POS))

#define TEMP_DIS_POS        3U
#define TEMP_DIS            ((uint8_t)(1U << TEMP_DIS_POS))

#define CYCLE_MODE_POS      5U
#define CYCLE_MODE          ((uint8_t)(1U << CYCLE_MODE_POS))

#define SLEEP_MODE_POS      6U
#define SLEEP_MODE          ((uint8_t)(1U << SLEEP_MODE_POS))

#define STATUS_CHECK(status)  do{                                  \
                            if((status) != HAL_OK)                 \
                            {                                      \
                                return status;                     \
                            }                                      \
                        }while (0)    
            
#define MEM_CHECK(memory)  do{                                     \
                            if((memory) == NULL)                   \
                            {                                      \
                                return HAL_ERROR;                  \
                            }                                      \
                        }while (0)
/* local strucutres definitions */

typedef struct {
    uint8_t addr;
    uint8_t val;
} reg_t;

/* static functions declarations */

static HAL_StatusTypeDef mpu_delay(MPU6050_t *handles, uint32_t ms);
static MPU6050_selftest_t calculate_ft(const uint8_t gyro[3], const uint8_t accel[3]);
static void parse_payload_selftest(const uint8_t raw[12], int16_t *inter);
static inline float selftest_ratio(int16_t diff, float ft);
static inline int16_t conv_to_i16(uint8_t msb, uint8_t lsb);
static inline HAL_StatusTypeDef gyro_path_reset(MPU6050_t *handles);
static inline HAL_StatusTypeDef accel_path_reset(MPU6050_t *handles);
static HAL_StatusTypeDef bitset_helper(MPU6050_t *handles, uint8_t reg_address, uint8_t mask, MPU_6050_state_t state);



static const reg_t init_registers[] = {
    { PWR_MGMT_1,        PWR_MGMT_1_VAL_DEFAULT},
    { SMPLRT_DIV_REG,    SMPLTR_DIV_VAL_DEFAULT },
    { CONFIG_REG,        CONFIG_VAL_DEFAULT },
    { FIFO_EN_REG,       FIFO_EN_VAL_DEFAULT },
    { INT_ENABLE_REG,    INT_ENABLE_VAL_DEFAULT }
};


/**
  * @brief  Initialize MPU6050 sensor with default configuration.
  * @param  handles Pointer to MPU6050 handle structure.
  * 
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_Init(MPU6050_t *handles) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    for(size_t i = 0; i < sizeof(init_registers)/sizeof(init_registers[0]); i++) {
        uint8_t v = init_registers[i].val;
        status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                                  init_registers[i].addr, MPU6050_REG_SIZE,
                                  &v, 1, I2C_TIMEOUT);
        STATUS_CHECK(status);
    }
    handles->gyro_scale = DPS_250;
    handles->accel_scale = G_2;

    return HAL_OK;
}


/**
  * @brief  Configure MPU6050 power/streaming mode (single sample, FIFO burst, or low-power cycle).
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  mode    Selected mode: MPU_SINGLE_MODE, MPU_BURST_MODE, or MPU_LOWPOWER_CYCLE_MODE.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_Set_Mode(MPU6050_t *handles, MPU_6050_mode_t mode)
{
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t reg;

    switch (mode)
    {
    case MPU_SINGLE_MODE:
        /* INT: Data Ready */
        reg = DATA_READY_INT;
        status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                                  INT_ENABLE_REG, MPU6050_REG_SIZE,
                                  &reg, 1, I2C_TIMEOUT);
        STATUS_CHECK(status);

        /* FIFO OFF */
        status = bitset_helper(handles, USER_CTRL_REG, FIFO_ENABLE, MPU_DISABLE);
        STATUS_CHECK(status);

        /* Normal power (no cycle, no sleep) */
        status = bitset_helper(handles, PWR_MGMT_1, CYCLE_MODE, MPU_DISABLE);
        STATUS_CHECK(status);
        status = bitset_helper(handles, PWR_MGMT_1, SLEEP_MODE, MPU_DISABLE);
        STATUS_CHECK(status);

        /* Enable temp + gyro axes */
        status = bitset_helper(handles, PWR_MGMT_1, TEMP_DIS, MPU_DISABLE);
        STATUS_CHECK(status);

        status = bitset_helper(handles, PWR_MGMT_2,
                               (uint8_t)(GYRO_X_STANDBY | GYRO_Y_STANDBY | GYRO_Z_STANDBY),
                               MPU_DISABLE);
        STATUS_CHECK(status);

        /* Enable accel axes (clear standby) */
        status = bitset_helper(handles, PWR_MGMT_2,
                               (uint8_t)(ACCEL_X_STANDBY | ACCEL_Y_STANDBY | ACCEL_Z_STANDBY),
                               MPU_DISABLE);
        STATUS_CHECK(status);
        break;

    case MPU_BURST_MODE:
        /* INT: FIFO overflow */
        reg = FIFO_OFLOW_INT;
        status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                                  INT_ENABLE_REG, MPU6050_REG_SIZE,
                                  &reg, 1, I2C_TIMEOUT);
        STATUS_CHECK(status);

        /* FIFO ON */
        status = bitset_helper(handles, USER_CTRL_REG, FIFO_ENABLE, MPU_ENABLE);
        STATUS_CHECK(status);

        /* Normal power (no cycle, no sleep) */
        status = bitset_helper(handles, PWR_MGMT_1, CYCLE_MODE, MPU_DISABLE);
        STATUS_CHECK(status);
        status = bitset_helper(handles, PWR_MGMT_1, SLEEP_MODE, MPU_DISABLE);
        STATUS_CHECK(status);

        /* Enable temp + gyro axes */
        status = bitset_helper(handles, PWR_MGMT_1, TEMP_DIS, MPU_DISABLE);
        STATUS_CHECK(status);

        status = bitset_helper(handles, PWR_MGMT_2,
                               (uint8_t)(GYRO_X_STANDBY | GYRO_Y_STANDBY | GYRO_Z_STANDBY),
                               MPU_DISABLE);
        STATUS_CHECK(status);

        /* Enable accel axes (clear standby) */
        status = bitset_helper(handles, PWR_MGMT_2,
                               (uint8_t)(ACCEL_X_STANDBY | ACCEL_Y_STANDBY | ACCEL_Z_STANDBY),
                               MPU_DISABLE);
        STATUS_CHECK(status);
        break;

    case MPU_LOWPOWER_CYCLE_MODE:
        /* INT: Data Ready */
        reg = DATA_READY_INT;
        status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                                  INT_ENABLE_REG, MPU6050_REG_SIZE,
                                  &reg, 1, I2C_TIMEOUT);
        STATUS_CHECK(status);

        /* FIFO OFF (cycle mode is accel-only low power; FIFO streaming doesn't make sense here) */
        status = bitset_helper(handles, USER_CTRL_REG, FIFO_ENABLE, MPU_DISABLE);
        STATUS_CHECK(status);

        /* Required for accel-only low power cycle:
           - TEMP disabled
           - Gyro in standby
           - CYCLE enabled
           - SLEEP disabled */
        status = bitset_helper(handles, PWR_MGMT_1, TEMP_DIS, MPU_ENABLE); /* TEMP_DIS=1 */
        STATUS_CHECK(status);

        status = bitset_helper(handles, PWR_MGMT_2,
                               (uint8_t)(GYRO_X_STANDBY | GYRO_Y_STANDBY | GYRO_Z_STANDBY),
                               MPU_ENABLE); /* standby=1 */
        STATUS_CHECK(status);

        /* Ensure accel enabled (standby bits cleared) */
        status = bitset_helper(handles, PWR_MGMT_2,
                               (uint8_t)(ACCEL_X_STANDBY | ACCEL_Y_STANDBY | ACCEL_Z_STANDBY),
                               MPU_DISABLE);
        STATUS_CHECK(status);

        status = bitset_helper(handles, PWR_MGMT_1, SLEEP_MODE, MPU_DISABLE);
        STATUS_CHECK(status);
        status = bitset_helper(handles, PWR_MGMT_1, CYCLE_MODE, MPU_ENABLE);
        STATUS_CHECK(status);
        break;

    default:
        return HAL_ERROR;
    }

    return HAL_OK;
}


/**
  * @brief  Turn on/off Sleep Mode
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  state MPU_ENABLE to enable, MPU_DISABLE to disable.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_Set_Sleep(MPU6050_t *handles, MPU_6050_state_t state) {
    MEM_CHECK(handles);
    return bitset_helper(handles, PWR_MGMT_1, SLEEP_MODE, state);
}


/**
  * @brief  Configure wake-up frequency for accelerometer low-power cycle mode.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  freq    Low-power wake-up frequency selection (MPU_6050_lp_freq_t).
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_Set_Lp_Wakeup_Freq(MPU6050_t *handles, MPU_6050_lp_freq_t freq) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t reg;
    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                              PWR_MGMT_2, MPU6050_REG_SIZE,
                              &reg, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);

    reg &= ~(LP_WAKE_CTRL);
    reg |= (uint8_t)(freq << LP_WAKE_CTRL_POS);
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                               PWR_MGMT_2, MPU6050_REG_SIZE,
                               &reg, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);

    return HAL_OK;
}


/**
  * @brief  Enable or disable selected measurement channel.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  ch      Measurement channel to configure (accel, gyro axis or temperature).
  * @param  state   MPU_ENABLE to enable, MPU_DISABLE to disable.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_Set_Channel_State(MPU6050_t *handles, MPU_6050_meas_channel_t ch, MPU_6050_state_t state) {
    MEM_CHECK(handles);
    MPU_6050_state_t real_state = (state == MPU_ENABLE) ? MPU_DISABLE : MPU_ENABLE;

    switch (ch)
    {
    case ACCEL_X_CH:
        return bitset_helper(handles, PWR_MGMT_2, ACCEL_X_STANDBY, real_state);

    case ACCEL_Y_CH:
        return bitset_helper(handles, PWR_MGMT_2, ACCEL_Y_STANDBY, real_state);

    case ACCEL_Z_CH:
        return bitset_helper(handles, PWR_MGMT_2, ACCEL_Z_STANDBY, real_state);

    case GYRO_X_CH:
        return bitset_helper(handles, PWR_MGMT_2, GYRO_X_STANDBY, real_state);

    case GYRO_Y_CH:
        return bitset_helper(handles, PWR_MGMT_2, GYRO_Y_STANDBY, real_state);

    case GYRO_Z_CH:
        return bitset_helper(handles, PWR_MGMT_2, GYRO_Z_STANDBY, real_state);

    case TEMP_CH:
        return bitset_helper(handles, PWR_MGMT_1, TEMP_DIS, real_state);

    default:
        return HAL_ERROR;
    }
}


/**
  * @brief  Reset MPU6050 FIFO buffer and re-enable FIFO operation.
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_FIFO_Reset(MPU6050_t *handles) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t reg;

    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                              USER_CTRL_REG, MPU6050_REG_SIZE,
                              &reg, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);
    reg &= ~(FIFO_ENABLE);
    reg |= FIFO_RESET;
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                               USER_CTRL_REG, MPU6050_REG_SIZE,
                               &reg, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);
    status = mpu_delay(handles, 2);
    STATUS_CHECK(status);
    reg |= FIFO_ENABLE;
    reg &= ~(FIFO_RESET);
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                               USER_CTRL_REG, MPU6050_REG_SIZE,
                               &reg, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);

    return HAL_OK;
}


/**
  * @brief  Perform built-in self-test procedure of MPU6050.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  result  Pointer to structure storing self-test results (in %).
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_Self_Test(MPU6050_t *handles, MPU6050_selftest_t *result) {
    MEM_CHECK(handles);
    MEM_CHECK(result);
    HAL_StatusTypeDef status = HAL_OK;

    
    uint8_t power_mgmt1_original;
    uint8_t power_mgmt2_original;

    /* turn off cycle/sleep mode*/
    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                              PWR_MGMT_1, MPU6050_REG_SIZE,
                              &power_mgmt1_original, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);                       
    status = bitset_helper(handles, PWR_MGMT_1,
                           (uint8_t)(SLEEP_MODE | CYCLE_MODE), MPU_DISABLE);
    STATUS_CHECK(status);

    /* turn on all accel and gyro axis if any were disabled before */
    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                              PWR_MGMT_2, MPU6050_REG_SIZE,
                              &power_mgmt2_original, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);                       
    status = bitset_helper(handles, PWR_MGMT_2,
                           (uint8_t)(ACCEL_X_STANDBY | ACCEL_Y_STANDBY | ACCEL_Z_STANDBY | GYRO_X_STANDBY | GYRO_Y_STANDBY | GYRO_Z_STANDBY),
                           MPU_DISABLE);
    STATUS_CHECK(status);

    /* store original measurement range */
    enum {GYRO = 0, ACCEL = 1};
    uint8_t original_config[2];
    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                              GYRO_CONFIG_REG, MPU6050_REG_SIZE,
                              original_config, 2, I2C_TIMEOUT);
    STATUS_CHECK(status);

    /* set gyroscope range to +-250dps and accelerometer range to +-8g for selftest */
    uint8_t test_config[2] = {0U, (uint8_t)(1U << 4)};

    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                               GYRO_CONFIG_REG, MPU6050_REG_SIZE,
                               test_config, 2, I2C_TIMEOUT);
    STATUS_CHECK(status);
    status = mpu_delay(handles, 50);
    STATUS_CHECK(status);
    
    uint8_t test_raw[4];
    uint8_t gyro_test[3];
    uint8_t accel_test[3];

    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                              SELF_TEST_X, MPU6050_REG_SIZE,
                              test_raw, 4, I2C_TIMEOUT);
    STATUS_CHECK(status);

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

    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                              ACCEL_XOUT_H, MPU6050_REG_SIZE,
                              test_dis_data_raw, SELFTEST_PAYLOAD/2, I2C_TIMEOUT);
    STATUS_CHECK(status);
    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                              GYRO_XOUT_H, MPU6050_REG_SIZE,
                              test_dis_data_raw + SELFTEST_PAYLOAD/2, SELFTEST_PAYLOAD/2, I2C_TIMEOUT);
    STATUS_CHECK(status);

    test_config[GYRO] |= (uint8_t)(7U << 5);
    test_config[ACCEL] |= (uint8_t)(7U << 5);

    /* enable selftest */
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                               GYRO_CONFIG_REG, MPU6050_REG_SIZE,
                               test_config, 2, I2C_TIMEOUT);
    STATUS_CHECK(status);
    status = mpu_delay(handles, 50);
    STATUS_CHECK(status);

    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                              ACCEL_XOUT_H, MPU6050_REG_SIZE,
                              test_en_data_raw, SELFTEST_PAYLOAD/2, I2C_TIMEOUT);
    STATUS_CHECK(status);
    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                              GYRO_XOUT_H, MPU6050_REG_SIZE,
                              test_en_data_raw + SELFTEST_PAYLOAD/2, SELFTEST_PAYLOAD/2, I2C_TIMEOUT);
    STATUS_CHECK(status);

    /* disable selftest, restore previous scale */
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                               GYRO_CONFIG_REG, MPU6050_REG_SIZE,
                               original_config, 2, I2C_TIMEOUT);
    STATUS_CHECK(status);

    /* restore previous PWR_MGMT registers */
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                               PWR_MGMT_1, MPU6050_REG_SIZE,
                               &power_mgmt1_original, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);

    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                               PWR_MGMT_2, MPU6050_REG_SIZE,
                               &power_mgmt2_original, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);

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


/**
  * @details
  * If a user-defined delay callback is provided in the handle structure,
  * it is executed with the requested delay time. If the callback pointer
  * is NULL, the function returns immediately without performing any delay.
  */
static HAL_StatusTypeDef mpu_delay(MPU6050_t *handles, uint32_t ms) {
    MEM_CHECK(handles);
    if(handles->delay_ms_wrapper == NULL) {
        return HAL_OK;
    }
    handles->delay_ms_wrapper(ms);

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


/**
  * @brief  Set gyroscope full-scale range.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  range   Gyroscope range selection (DPS_250, DPS_500, DPS_1000, DPS_2000).
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_Set_Gyro_Range(MPU6050_t *handles, MPU_6050_gyro_range_t range) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t reg = 0;

    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                              GYRO_CONFIG_REG, MPU6050_REG_SIZE,
                              &reg, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);
    reg &= ~(GYRO_RANGE);
    reg |= (uint8_t)(range << GYRO_RANGE_POS);
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                               GYRO_CONFIG_REG, MPU6050_REG_SIZE,
                               &reg, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);
    handles->gyro_scale = range;
    status = gyro_path_reset(handles);
    STATUS_CHECK(status);

    return HAL_OK;
}


/**
  * @brief  Set accelerometer full-scale range.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  range   Accelerometer range selection (G_2, G_4, G_8, G_16).
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_Set_Accel_Range(MPU6050_t *handles, MPU_6050_accel_range_t range) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t reg = 0;

    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                              ACCEL_CONFIG_REG, MPU6050_REG_SIZE,
                              &reg, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);
    reg &= ~(ACCEL_RANGE);
    reg |= (uint8_t)(range << ACCEL_RANGE_POS);
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                               ACCEL_CONFIG_REG, MPU6050_REG_SIZE,
                               &reg, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);
    handles->accel_scale = range;
    status = accel_path_reset(handles);
    STATUS_CHECK(status);

    return HAL_OK;
}


static inline HAL_StatusTypeDef gyro_path_reset(MPU6050_t *handles) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t reg = 4U;
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                               SIGNAL_PATH_RESET, MPU6050_REG_SIZE,
                               &reg, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);

    return HAL_OK;
}


static inline HAL_StatusTypeDef accel_path_reset(MPU6050_t *handles) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t reg = 2U;
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                               SIGNAL_PATH_RESET, MPU6050_REG_SIZE,
                               &reg, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);

    return HAL_OK;
}


/**
  * @brief  Enable or disable selected sensor data in FIFO buffer.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  content FIFO content mask (combination of MPU_6050_fifo_content_t values).
  * @param  state   MPU_ENABLE to set bits, MPU_DISABLE to clear bits.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_Set_FIFO_Content(MPU6050_t *handles, MPU_6050_fifo_content_t content, MPU_6050_state_t state) {
    MEM_CHECK(handles);
    return bitset_helper(handles, FIFO_EN_REG, (uint8_t)content, state);
}


/**
  * @brief  Start single DMA read of sensor measurement payload.
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_Single_Read(MPU6050_t *handles) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    
    status = HAL_I2C_Mem_Read_DMA(handles->hi2c, I2C_ADDRESS_HAL,
                                  ACCEL_XOUT_H, MPU6050_REG_SIZE,
                                  handles->rx_buffer, FULL_PAYLOAD_SIZE);
    STATUS_CHECK(status);

    return HAL_OK;
}


/**
  * @brief  Extracts current FIFO buffer count.
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_Read_FIFO_Cnt(MPU6050_t *handles) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_I2C_Mem_Read_DMA(handles->hi2c, I2C_ADDRESS_HAL,
                                  FIFO_COUNT_H, MPU6050_REG_SIZE,
                                  handles->fifo_counter_raw, 2);
    STATUS_CHECK(status);

    return HAL_OK;
}


/**
  * @brief  Converts raw fifo_counter to uint16_t fifo_counter
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  raw two bytes of burst_count [high, low]
  * 
  * @retval None
  */
HAL_StatusTypeDef MPU_6050_Process_Burst_Cnt(MPU6050_t *handles) {
    MEM_CHECK(handles);
    handles->fifo_counter = ((uint16_t)handles->fifo_counter_raw[0] << 8) | (uint16_t)handles->fifo_counter_raw[1];

    return HAL_OK;
}


/**
  * @brief  Start burst DMA read from FIFO.
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_Burst_Read(MPU6050_t *handles) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    
    status = HAL_I2C_Mem_Read_DMA(handles->hi2c, I2C_ADDRESS_HAL,
                                  FIFO_R_W, MPU6050_REG_SIZE,
                                  handles->rx_buffer, handles->burst_count);
    STATUS_CHECK(status);

    return HAL_OK;
}


static inline int16_t conv_to_i16(uint8_t msb, uint8_t lsb) {
    uint16_t u = ((uint16_t)msb << 8) | (uint16_t)lsb;
    return (int16_t)u;
}


/**
  * @brief  Convert raw 14-byte payload into 16-bit signed values.
  * @param  raw   Pointer to raw sensor payload (14 bytes).
  * @param  inter Pointer to output buffer for converted values (7 x int16_t).
  *
  * @retval None.
  */
void MPU_6050_parse_payload(const uint8_t raw[14], int16_t *inter) {
    for(size_t i = 0; i < 7; i++){
        *inter++ = conv_to_i16(raw[2*i], raw[2*i+1]);
    }
}


/**
  * @brief  Convert parsed raw data to scaled physical units.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  payload Pointer to converted raw measurement array (7 x int16_t).
  *
  * @retval MPU6050_data_t structure with scaled values.
  */
MPU6050_data_t MPU_6050_payload_to_readable(MPU6050_t *handles, const int16_t payload[7]) {
    MPU6050_data_t readable;
    float accel_div = 16384.0f / (1U << handles->accel_scale);
    float gyro_div = 131.072f / (1U << handles->gyro_scale); 
    readable.accel_x = payload[0]/accel_div;
    readable.accel_y = payload[1]/accel_div;
    readable.accel_z = payload[2]/accel_div;
    readable.temp = (payload[3]/340.0f) + 35.0f;
    readable.gyro_x = payload[4]/gyro_div;
    readable.gyro_y = payload[5]/gyro_div;
    readable.gyro_z = payload[6]/gyro_div;

    return readable;
}


static HAL_StatusTypeDef bitset_helper(MPU6050_t *handles, uint8_t reg_address, uint8_t mask, MPU_6050_state_t state) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t reg;

    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                              (uint16_t)reg_address, MPU6050_REG_SIZE,
                              &reg, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);
    if(state == MPU_ENABLE) {
        reg |= mask;
    }
    else {
        reg &= ~(mask);
    }

    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                               (uint16_t)reg_address, MPU6050_REG_SIZE,
                               &reg, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);

    return HAL_OK;
}