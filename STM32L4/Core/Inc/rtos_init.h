#ifndef RTOS_INIT_H
#define RTOS_INIT_H

typedef struct {
    TaskFunction_t entry;
    const char *name;
    uint32_t stack_size;
    void *args;
    UBaseType_t priority;
    StackType_t *stack;
    StaticTask_t *tcb;
} task_init_t;

TaskHandle_t create_static_task(const task_init_t *);

#endif