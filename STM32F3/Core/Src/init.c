#include "init.h"
#include "stm32f303xe.h"

#define LED_PORT GPIOA
#define LED_PIN  5U

#define BTN_PORT GPIOB
#define BTN_PIN  13U

void SystemClock_Config(void)
{
    
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0U);

    FLASH->ACR = FLASH_ACR_LATENCY_2 | FLASH_ACR_PRFTBE;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_2);

    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |=  RCC_CFGR_PPRE1_DIV2;

    RCC->CR &= ~RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) != 0U);

    RCC->CFGR2 &= ~(RCC_CFGR2_PREDIV);
    RCC->CFGR2 |= RCC_CFGR2_PREDIV_DIV1;

    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL);
    RCC->CFGR |=  RCC_CFGR_PLLSRC_HSI_PREDIV;
    RCC->CFGR |= RCC_CFGR_PLLMUL9;

    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0U);

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |=  RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    SystemCoreClockUpdate();
}

void GPIO_Init(void)
{
    /* GPIOA + GPIOB clock enable */
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
    (void)RCC->AHBENR;

    /* --- LED: PA9 jako wyjście push-pull --- */
    LED_PORT->MODER &= ~(0x3U << (LED_PIN * 2U));
    LED_PORT->MODER |=  (0x1U << (LED_PIN * 2U));     /* 01 = output */

    LED_PORT->OTYPER &= ~(1U << LED_PIN);             /* push-pull */
    LED_PORT->OSPEEDR &= ~(0x3U << (LED_PIN * 2U));
    LED_PORT->OSPEEDR |=  (0x1U << (LED_PIN * 2U));   /* medium */
    LED_PORT->PUPDR &= ~(0x3U << (LED_PIN * 2U));     /* no pull */

    /* Ustal stan początkowy LED (tu: zgaś -> reset bit) */
    LED_PORT->BRR = (1U << LED_PIN);

    /* --- BTN: PB13 jako wejście, bez pull (bo masz zewnętrzny pull-up) --- */
    BTN_PORT->MODER &= ~(0x3U << (BTN_PIN * 2U));     /* 00 = input */
    BTN_PORT->PUPDR &= ~(0x3U << (BTN_PIN * 2U));     /* no pull */
}