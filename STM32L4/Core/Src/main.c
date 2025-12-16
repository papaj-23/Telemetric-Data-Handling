#include "stm32l4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "init.h"

#define STACK_SIZE 128

static StaticTask_t IdleTaskTCB;
static StackType_t IdleTaskStack[configMINIMAL_STACK_SIZE];

static const char Tester_task_name[] = "tester";
volatile uint32_t counter = 0;
static StackType_t stack_buf[STACK_SIZE];
static StaticTask_t tester_TCB;

void vApplicationIdleHook(void);
void Tester_task();

static void System_Init() {
    SystemClock_Config();
    GPIO_Init();
    I2C1_Init();
    USART2_UART_Init();
}

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
    *ppxIdleTaskTCBBuffer = &IdleTaskTCB;
    *ppxIdleTaskStackBuffer = IdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

static void RTOS_Init() {
    TaskHandle_t testerHandle = NULL;

    testerHandle = xTaskCreateStatic(
        Tester_task,
        Tester_task_name,
        STACK_SIZE,
        NULL,
        2,
        stack_buf,
        &tester_TCB
    );

    configASSERT(testerHandle);
}

void Tester_task() {
    for(;;){
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void vApplicationIdleHook() {
    counter++;
}

void vApplicationTickHook() {

}

int main(void)
{
    System_Init();
    RTOS_Init();
    vTaskStartScheduler();
    for(;;) {
        __WFI();
    }
}