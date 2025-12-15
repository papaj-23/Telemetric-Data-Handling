#include "init.h"
#include "stm32l432xx.h"

#define LED_PORT GPIOB
#define LED_PIN  3U

void SystemClock_Config(void)
{
    /* 1. Włącz HSI (domyślnie włączone, ale dla pewności) */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {
        /* wait */
    }

    /* 2. Włącz zasilanie PWR i ustaw VOS = Range 1 */
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;

    /* VOS = 01 → Range 1 */
    PWR->CR1 &= ~PWR_CR1_VOS_Msk;
    PWR->CR1 |= (1U << PWR_CR1_VOS_Pos);

    while (PWR->SR2 & PWR_SR2_VOSF) {
        /* wait: stabilizacja regulatora */
    }

    /* 3. Flash wait states na 80 MHz */
    FLASH->ACR = FLASH_ACR_LATENCY_4WS |
                 FLASH_ACR_PRFTEN      |
                 FLASH_ACR_ICEN        |
                 FLASH_ACR_DCEN;

    while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != FLASH_ACR_LATENCY_4WS) {
        /* wait */
    }

    /* 4. Preskalery = 1 */
    RCC->CFGR &= ~(RCC_CFGR_HPRE  |
                   RCC_CFGR_PPRE1 |
                   RCC_CFGR_PPRE2);

    /* 5. PLL: source = HSI, M=2, N=20, R=2 */

    /* Wyłącz PLL jeśli aktualnie chodzi */
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) {
        /* wait */
    }

    /* Wyczyść konfigurację */
    RCC->PLLCFGR = 0;

    /* PLLSRC = HSI */
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI;

    /* PLLM = 2 → zapisujemy (M-1) */
    RCC->PLLCFGR |= ((2U - 1U) << RCC_PLLCFGR_PLLM_Pos);

    /* PLLN = 20 */
    RCC->PLLCFGR |= (20U << RCC_PLLCFGR_PLLN_Pos);

    /* PLLR = 2 → 00 */
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLR_Msk;

    /* włącz wyjście PLLR */
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;

    /* 6. Włącz PLL */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {
        /* wait */
    }

    /* 7. Ustaw PLL jako SYSCLK */
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
        /* wait */
    }
    
    SystemCoreClockUpdate();
    /* → SYSCLK = 80 MHz */
}

void GPIO_Init(void){
     /* 1. Włącz zegar dla GPIOB */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    (void)RCC->AHB2ENR; // mały „dummy read” – czas na start zegara

    /* 2. PB3 jako wyjście push-pull, bez pull-up/down, średnia prędkość */

    /* MODER[2*3+1:2*3] = 01 (Output) */
    LED_PORT->MODER &= ~(0x3U << (LED_PIN * 2));
    LED_PORT->MODER |=  (0x1U << (LED_PIN * 2));

    /* OTYPER bit 3 = 0 (push-pull) */
    LED_PORT->OTYPER &= ~(1U << LED_PIN);

    /* OSPEEDR[2*3+1:2*3] = 01 (Medium speed) */
    LED_PORT->OSPEEDR &= ~(0x3U << (LED_PIN * 2));
    LED_PORT->OSPEEDR |=  (0x1U << (LED_PIN * 2));

    /* PUPDR[2*3+1:2*3] = 00 (no pull) */
    LED_PORT->PUPDR &= ~(0x3U << (LED_PIN * 2));

    /* Na start zgaś LED (zakładam stan wysoki = świeci, ale to zależy od HW) */
    LED_PORT->BSRR = (1U << (LED_PIN + 16U));  // reset
}