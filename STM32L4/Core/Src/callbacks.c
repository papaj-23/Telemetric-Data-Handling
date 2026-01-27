#include "init.h"
#include "FreeRTOS.h"
#include "semphr.h"

extern SemaphoreHandle_t read_data_sem;
extern SemaphoreHandle_t data_ready_sem;
uint32_t cnt = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if (htim->Instance == TIM6){
    BaseType_t hpw = pdFALSE;
    xSemaphoreGiveFromISR(read_data_sem, NULL);
    portYIELD_FROM_ISR(hpw);

  }

  if(htim->Instance == TIM7){
    HAL_IncTick();
  }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
  if(hi2c == &hi2c1){
    BaseType_t hpw = pdFALSE;
    xSemaphoreGiveFromISR(data_ready_sem, NULL);
    portYIELD_FROM_ISR(hpw);
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){
  if(hi2c == &hi2c1)
    cnt++;
}