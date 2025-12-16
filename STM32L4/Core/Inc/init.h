#ifndef INIT_H
#define INIT_H

#define LD3_GPIO_Port GPIOB
#define LD3_Pin  3U

void SystemClock_Config(void);
void GPIO_Init(void);
void USART2_UART_Init(void);
void I2C1_Init(void);
void Error_Handler(void);

#endif