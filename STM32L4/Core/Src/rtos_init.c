#include "FreeRTOS.h"
#include "task.h"
#include "rtos_init.h"

TaskHandle_t create_static_task(const task_init_t *t)
{
    TaskHandle_t h = xTaskCreateStatic(
        t->entry,
        t->name,
        t->stack_size,
        t->args,
        t->priority,
        t->stack,
        t->tcb
    );
    configASSERT(h);
    return h;
}

