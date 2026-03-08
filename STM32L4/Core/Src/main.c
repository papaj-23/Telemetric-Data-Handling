#include "stm32l4xx_hal.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "init.h"
#include "rtos_init.h"
#include "mpu-6050.h"

#define BURST_COUNT 600

static void Tester_handler(void*);
static void MPU6050_data_receive_handler(void*);
static void RTOS_Init(void);

/* global variables */
uint8_t dma_i2c_rx_buf[14];

int16_t intermediate_rx_buf[7];

MPU_6050_t mpu_handle;
MPU_6050_selftest_t selftest_results;
MPU_6050_data_t current_data;

/* Idle task */
static StaticTask_t IdleTaskTCB;
static StackType_t IdleTaskStack[configMINIMAL_STACK_SIZE];

/* Tester task */
#define TESTER_STACK_SIZE 128
static StackType_t tester_stack[TESTER_STACK_SIZE];
static StaticTask_t tester_tcb;
static TaskHandle_t tester_handle = NULL;
static const task_init_t tester_init = {
    .entry = Tester_handler,
    .name = "tester",
    .stack_size = TESTER_STACK_SIZE,
    .args = NULL,
    .priority = 2,
    .stack = tester_stack,
    .tcb = &tester_tcb
};

/* MPU6050 data transfer task */
#define MPU6050_DATA_RECEIVE_STACK_SIZE 512
static StackType_t mpu6050_data_receive_stack[MPU6050_DATA_RECEIVE_STACK_SIZE];
static StaticTask_t mpu6050_data_receive_tcb;
static TaskHandle_t mpu6050_data_receive_handle = NULL;
static const task_init_t mpu6050_data_receive_init = {
    .entry = MPU6050_data_receive_handler,
    .name = "MPU6050_data_receive",
    .stack_size = MPU6050_DATA_RECEIVE_STACK_SIZE,
    .args = NULL,
    .priority = 3,
    .stack = mpu6050_data_receive_stack,
    .tcb = &mpu6050_data_receive_tcb
};

static void delay_wrapper_ms(uint32_t ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

static void RTOS_Init(void) {
    tester_handle = create_static_task(&tester_init);
    mpu6050_data_receive_handle = create_static_task(&mpu6050_data_receive_init);
}

static void Tester_handler(void* pvParameters) {
    for(;;) {
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void MPU6050_data_receive_handler(void* pvParameters) {

    HAL_TIM_Base_Start_IT(&htim6);

    mpu_handle = (MPU_6050_t) {
        .hi2c = &hi2c1,
        .rx_buffer = dma_i2c_rx_buf,
        .delay_ms_wrapper = delay_wrapper_ms,
        .burst_count = BURST_COUNT
    };
    uint32_t events = 0U;

    MPU_6050_Init(&mpu_handle);
    MPU_6050_Self_Test(&mpu_handle, &selftest_results);

    MPU_6050_Set_Mode(&mpu_handle, MPU_SINGLE_MODE);
    

    //MPU_6050_Set_Channel_State(&mpu_handle, GYRO_X_CH, MPU_DISABLE);
    //MPU_6050_Set_Channel_State(&mpu_handle, GYRO_Y_CH, MPU_DISABLE);
    //MPU_6050_Set_Channel_State(&mpu_handle, GYRO_Z_CH, MPU_DISABLE);
    //MPU_6050_Set_Channel_State(&mpu_handle, TEMP_CH, MPU_DISABLE);

    for(;;) {
        check_registers(&mpu_handle);
        xTaskNotifyWait(0U, 0xFFFFFFFFU, &events, portMAX_DELAY);

        /* external interrupt from MPU6050 */
        if(events & 0x01U) {
            MPU_6050_Interrupt_Handler(&mpu_handle);
        }

        /* i2c dma transfer complete */
        if(events & 0x02U) {
            MPU_6050_parse_payload(dma_i2c_rx_buf, intermediate_rx_buf);
            current_data = MPU_6050_payload_to_readable(&mpu_handle, intermediate_rx_buf);
            //MPU_6050_Process_Burst_Cnt(&mpu_handle);
            if(mpu_handle.fifo_counter >= BURST_COUNT) {
                //MPU_6050_Burst_Read(&mpu_handle);
                mpu_handle.fifo_counter -= BURST_COUNT;
            }
        }

        /* trigger fifo_count read */
        if(events & 0x04) {
            //MPU_6050_Read_FIFO_Cnt(&mpu_handle);
        }


    }
}

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
    *ppxIdleTaskTCBBuffer = &IdleTaskTCB;
    *ppxIdleTaskStackBuffer = IdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

int main(void)
{
    HAL_Init();
    System_Init();
    RTOS_Init();
    vTaskStartScheduler();
    for(;;) {
        __WFI();
    }
}

/* isr callbacks */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIM6) {
        if((mpu6050_data_receive_handle != NULL) && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)) {
            BaseType_t hpw = pdFALSE;
            xTaskNotifyFromISR(mpu6050_data_receive_handle, 0x04U, eSetBits, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
    }

    if(htim->Instance == TIM7) {
        HAL_IncTick();
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if(hi2c == &hi2c1) {
        if((mpu6050_data_receive_handle != NULL) && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)) {
            BaseType_t hpw = pdFALSE;
            xTaskNotifyFromISR(mpu6050_data_receive_handle, 0x02U, eSetBits, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == GPIO_PIN_12) {
        if((mpu6050_data_receive_handle != NULL) && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)) {
            BaseType_t hpw = pdFALSE;
            xTaskNotifyFromISR(mpu6050_data_receive_handle, 0x01U, eSetBits, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
    }
}

/*
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){
  if(hi2c == &hi2c1)

}
  */