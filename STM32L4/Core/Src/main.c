#include "stm32l4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "init.h"
#include "rtos_init.h"
#include "mpu-6050.h"

static void Tester_handler(void*);
static void MPU6050_data_transfer_handler(void*);
static void RTOS_Init(void);

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
    .priority = 2,
    .stack = mpu6050_data_transfer_stack,
    .tcb = &mpu6050_data_transfer_tcb
};

static void Tester_handler(void* pvParameters) {
    for(;;){
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void MPU6050_data_transfer_handler(void* pvParameters) {
    ;
}

static void RTOS_Init(void) {
    create_static_task(&tester_init);
    create_static_task(&mpu6050_data_transfer_init);
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