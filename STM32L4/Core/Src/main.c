#include "stm32l4xx_hal.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "init.h"
#include "rtos_init.h"
#include "mpu-6050.h"

static void Tester_handler(void*);
static void MPU6050_data_transfer_handler(void*);
static void RTOS_Init(void);

uint8_t dma_i2c_tx_buf[10];
uint8_t dma_i2c_rx_buf[14];

int16_t intermediate_rx_buf[7];
MPU6050_data_t current_data;

// Idle task
static StaticTask_t IdleTaskTCB;
static StackType_t IdleTaskStack[configMINIMAL_STACK_SIZE];

// Tester task
#define TESTER_STACK_SIZE 128
static StackType_t tester_stack[TESTER_STACK_SIZE];
static StaticTask_t tester_tcb;
static const task_init_t tester_init = {
    .entry = Tester_handler,
    .name = "tester",
    .stack_size = TESTER_STACK_SIZE,
    .args = NULL,
    .priority = 2,
    .stack = tester_stack,
    .tcb = &tester_tcb
};

// MPU6050 data transfer task
#define MPU6050_DATA_TRANSFER_STACK_SIZE 512
static StackType_t mpu6050_data_transfer_stack[MPU6050_DATA_TRANSFER_STACK_SIZE];
static StaticTask_t mpu6050_data_transfer_tcb;
static const task_init_t mpu6050_data_transfer_init = {
    .entry = MPU6050_data_transfer_handler,
    .name = "MPU6050_data_transfer",
    .stack_size = MPU6050_DATA_TRANSFER_STACK_SIZE,
    .args = NULL,
    .priority = 3,
    .stack = mpu6050_data_transfer_stack,
    .tcb = &mpu6050_data_transfer_tcb
};

// Semaphores / mutexes
SemaphoreHandle_t read_data_sem = NULL;
SemaphoreHandle_t data_ready_sem = NULL;

static void Tester_handler(void* pvParameters) {
    extern uint32_t cnt;
    for(;;) {
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        if(cnt > 1000)
            cnt = 0;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void MPU6050_data_transfer_handler(void* pvParameters) {
    MPU6050_t mpu_handle = {&hi2c1, dma_i2c_tx_buf, dma_i2c_rx_buf};
    MPU_6050_Init(&mpu_handle);
    for(;;) {
        xSemaphoreTake(read_data_sem, portMAX_DELAY);
        MPU_6050_Single_Read(&mpu_handle);
        xSemaphoreTake(data_ready_sem, portMAX_DELAY);
        MPU_6050_parse_payload(dma_i2c_rx_buf, intermediate_rx_buf);
        current_data = MPU6050_payload_to_readable(intermediate_rx_buf);
    }
}

static void RTOS_Init(void) {
    create_static_task(&tester_init);
    create_static_task(&mpu6050_data_transfer_init);
    read_data_sem = xSemaphoreCreateBinary();
    data_ready_sem = xSemaphoreCreateBinary();
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