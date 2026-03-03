#ifndef MPU_6050_H
#define MPU_6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t *tx_buffer;
    uint8_t *rx_buffer;
    void (*delay_ms_wrapper)(uint32_t);
    uint16_t burst_count;
    uint8_t fifo_counter_raw[2];
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
    MPU_LOWPOWER_CYCLE_MODE = 2U
} MPU_6050_mode_t;

typedef enum {
    F_1_25HZ = 0U,
    F_5HZ = 1U,
    F_20HZ = 2U,
    F_40HZ = 3U
} MPU_6050_lp_freq_t;

typedef enum {
    ACCEL_X_CH = 0U,
    ACCEL_Y_CH = 1U,
    ACCEL_Z_CH = 2U,
    TEMP_CH = 3U,
    GYRO_X_CH = 4U,
    GYRO_Y_CH = 5U,
    GYRO_Z_CH = 6U
} MPU_6050_meas_channel_t;


/**
  * @brief  Initialize MPU6050 sensor with default configuration.
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @details
  * Writes a set of default register values (power management, sample rate divider,
  * low-pass filter config, FIFO enable, interrupt enable) included in init_registers[].
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_Init(MPU6050_t *handles);

/**
  * @brief  Configure MPU6050 power/streaming mode (single sample, FIFO burst, or low-power cycle).
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  mode    Selected mode: MPU_SINGLE_MODE, MPU_BURST_MODE, or MPU_LOWPOWER_CYCLE_MODE.
  *
  * @details
  * The function configures interrupt source, FIFO usage and power-management bits
  * to match the selected operating mode:
  *
  * MPU_SINGLE_MODE:
  *  - Enables Data Ready interrupt (INT_ENABLE: DATA_RDY_EN).
  *  - Disables FIFO streaming (USER_CTRL: FIFO_EN = 0).
  *  - Forces normal operation: CYCLE = 0, SLEEP = 0.
  *  - Enables temperature sensor and all accel/gyro axes (standby bits cleared).
  *
  * MPU_BURST_MODE:
  *  - Enables FIFO Overflow interrupt (INT_ENABLE: FIFO_OFLOW_EN).
  *  - Enables FIFO streaming (USER_CTRL: FIFO_EN = 1).
  *  - Forces normal operation: CYCLE = 0, SLEEP = 0.
  *  - Enables temperature sensor and all accel/gyro axes (standby bits cleared).
  *
  * MPU_LOWPOWER_CYCLE_MODE:
  *  - Enables Data Ready interrupt (INT_ENABLE: DATA_RDY_EN).
  *  - Disables FIFO streaming (USER_CTRL: FIFO_EN = 0).
  *  - Configures accel-only low power cycling:
  *      * TEMP disabled (PWR_MGMT_1: TEMP_DIS = 1)
  *      * Gyro placed in standby (PWR_MGMT_2: gyro standby bits = 1)
  *      * Accel enabled (PWR_MGMT_2: accel standby bits = 0)
  *      * CYCLE enabled, SLEEP disabled (PWR_MGMT_1: CYCLE = 1, SLEEP = 0)
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_Set_Mode(MPU6050_t *handles, MPU_6050_mode_t mode);

/**
  * @brief  Turn on/off Sleep Mode
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  state MPU_ENABLE to enable, MPU_DISABLE to disable.
  * 
  * @details In sleep mode all onboard sensor and internal clock source are turned off.
  * This mode can be used in low power applications.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_Set_Sleep(MPU6050_t *handles, MPU_6050_state_t state);

/**
  * @brief  Configure wake-up frequency for accelerometer low-power cycle mode.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  freq    Low-power wake-up frequency selection (MPU_6050_lp_freq_t).
  *
  * @details
  * Updates LP_WAKE_CTRL[7:6] bits in the PWR_MGMT_2 register.
  * These bits define the internal wake-up rate used in
  * MPU_LOWPOWER_CYCLE_MODE (accelerometer-only duty-cycled operation).
  *
  * The function performs a read-modify-write operation to preserve
  * unrelated configuration bits in PWR_MGMT_2.
  *
  * @note
  * This setting has effect only when CYCLE mode is enabled
  * (PWR_MGMT_1: CYCLE = 1). In normal continuous measurement
  * modes, this field does not affect sensor output rate.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_Set_Lp_Wakeup_Freq(MPU6050_t *handles, MPU_6050_lp_freq_t freq);

/**
  * @brief  Enable or disable selected measurement channel.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  ch      Measurement channel to configure (accel, gyro axis or temperature).
  * @param  state   MPU_ENABLE to enable, MPU_DISABLE to disable.
  *
  * @details
  * For accelerometer and gyroscope axes, the function modifies the corresponding
  * standby bits in PWR_MGMT_2 register. Setting a standby bit disables the axis,
  * clearing it enables measurement.
  *
  * For temperature sensor, the function modifies TEMP_DIS bit in PWR_MGMT_1 register.
  * Setting TEMP_DIS disables temperature measurement, clearing it enables it.
  *
  * @attention Logic is inversed within this function so that passing ENABLE argument
  * resulted in turning selected sensor channel ON and DISABLE argument in turning OFF
  * 
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_Set_Channel_State(MPU6050_t *handles, MPU_6050_meas_channel_t ch, MPU_6050_state_t state);

/**
  * @brief  Reset MPU6050 FIFO buffer and re-enable FIFO operation.
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @details
  * Clears FIFO enable bit, triggers FIFO reset bit, then re-enables FIFO.
  * If handles->delay_ms_wrapper is provided, it may be used to wait a short time
  * between reset and re-enable.
  * 
  * @attention function is to be used when FIFO_OVERFLOW interrupt occurs
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_FIFO_Reset(MPU6050_t *handles);

/**
  * @brief  Perform built-in self-test procedure of MPU6050.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  result  Pointer to structure storing self-test results (in %).
  *
  * @details
  * The device should remain stationary during the test. The function:
  *  - Saves current PWR_MGMT1 and PWR_MGMT2.
  *  - Saves current GYRO_CONFIG and ACCEL_CONFIG.
  *  - Applies test ranges and reads self-test factory trim codes.
  *  - Reads sensor outputs with self-test disabled and enabled.
  *  - Restores original configuration.
  *  - Computes self-test response ratio in percent.
  *
  * A delay callback (handles->delay_ms_wrapper) may be required for stable results.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_Self_Test(MPU6050_t *handles, MPU6050_selftest_t *result);

/**
  * @brief  Set gyroscope full-scale range.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  range   Gyroscope range selection (DPS_250, DPS_500, DPS_1000, DPS_2000).
  *
  * @details
  * Updates FS_SEL bits in GYRO_CONFIG register. Afterwards, the gyro signal path
  * reset is triggered to ensure internal filtering state is refreshed.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_Set_Gyro_Range(MPU6050_t *handles, MPU_6050_gyro_range_t range);

/**
  * @brief  Set accelerometer full-scale range.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  range   Accelerometer range selection (G_2, G_4, G_8, G_16).
  *
  * @details
  * Updates AFS_SEL bits in ACCEL_CONFIG register. Afterwards, the accel signal path
  * reset is triggered to ensure internal filtering state is refreshed.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_Set_Accel_Range(MPU6050_t *handles, MPU_6050_accel_range_t range);

/**
  * @brief  Enable or disable selected sensor data in FIFO buffer.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  content FIFO content mask (combination of MPU_6050_fifo_content_t values).
  * @param  state   MPU_ENABLE to set bits, MPU_DISABLE to clear bits.
  *
  * @details
  * Modifies FIFO_EN register to select which sensor outputs are written into FIFO.
  * This function only configures FIFO content selection; FIFO must be enabled in
  * USER_CTRL separately.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_Set_FIFO_Content(MPU6050_t *handles, MPU_6050_fifo_content_t content, MPU_6050_state_t state);

/**
  * @brief  Start single DMA read of sensor measurement payload.
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @details
  * Starts a DMA read from ACCEL_XOUT_H for FULL_PAYLOAD_SIZE bytes:
  * accel (6), temperature (2), gyro (6) = 14 bytes total.
  * Completion is reported via I2C/DMA callbacks (HAL layer).
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_Single_Read(MPU6050_t *handles);

/**
  * @brief  Extracts current FIFO buffer count.
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @details
  * Reads FIFO_COUNT_H and FIFO_COUNT_L registers (16-bit) to determine the number of
  * bytes currently stored in the FIFO buffer.
  *
  * @note According to the datasheet, reading FIFO_COUNT_H latches the value of both
  * FIFO count registers. Therefore the count must be read in high-then-low order
  * to obtain a coherent snapshot.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_Read_FIFO_Cnt(MPU6050_t *handles);

/**
  * @brief  Converts raw fifo_counter to uint16_t fifo_counter
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  raw two bytes of burst_count [high, low]
  * 
  * @retval None
  */
HAL_StatusTypeDef MPU_6050_Process_Burst_Cnt(MPU6050_t *handles);

/**
  * @brief  Start burst DMA read from FIFO.
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @details
  * Starts a DMA read from FIFO_R_W register for handles->burst_count bytes.
  * FIFO must be properly configured and enabled before calling this function.
  * handles->burst_count must be a multiple of the current payload size in bytes
  * Make sure to provide DMA buffer >= handles->burst_count.
  * 
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_Burst_Read(MPU6050_t *handles);

/**
  * @brief  Convert raw 14-byte payload into 16-bit signed values.
  * @param  raw   Pointer to raw sensor payload (14 bytes).
  * @param  inter Pointer to output buffer for converted values (7 x int16_t).
  *
  * @details
  * Output order:
  *  [0..2] accel XYZ, [3] temperature, [4..6] gyro XYZ.
  *
  * @retval None.
  */
void MPU_6050_parse_payload(const uint8_t raw[14], int16_t *inter);

/**
  * @brief  Convert parsed raw data to scaled physical units.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  payload Pointer to converted raw measurement array (7 x int16_t).
  *
  * @details
  * Uses currently selected ranges to scale:
  *  - accelerometer to g,
  *  - gyroscope to deg/s,
  * and converts temperature to degrees Celsius.
  *
  * Note: scaling depends on internal range state used by this driver.
  *
  * @retval MPU6050_data_t structure with scaled values.
  */
MPU6050_data_t MPU_6050_payload_to_readable(MPU6050_t *handles, const int16_t payload[7]);

#ifdef __cplusplus
}
#endif

#endif