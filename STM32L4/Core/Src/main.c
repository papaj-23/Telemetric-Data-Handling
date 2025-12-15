#include "stm32l432xx.h"
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
        GPIOB->ODR ^= (1U << 3);
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