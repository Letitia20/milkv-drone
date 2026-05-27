#include "regs.h"

/* External functions */
extern int main(void);
extern void SystemInit(void);

/* Weak default handlers */
void Default_Handler(void) { while (1); }

void Reset_Handler(void);
void NMI_Handler(void)          __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)    __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void)    __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)          __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)       __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)      __attribute__((weak, alias("Default_Handler")));

/* Peripheral interrupt handlers — weak defaults */
void USART1_IRQHandler(void)    __attribute__((weak, alias("Default_Handler")));
void USART2_IRQHandler(void)    __attribute__((weak, alias("Default_Handler")));
void TIM2_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void EXTI0_IRQHandler(void)     __attribute__((weak, alias("Default_Handler")));
void ADC1_2_IRQHandler(void)    __attribute__((weak, alias("Default_Handler")));

/* Stack top declared in linker script */
extern uint32_t _stack_top;

/* Vector table */
__attribute__((section(".isr_vector")))
void (* const g_vector_table[])(void) = {
    (void(*)(void))&_stack_top,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0, 0, 0, 0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
    /* IRQ 0-15 */
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    /* IRQ 16-31 */
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    /* IRQ 32-47 */
    0, 0, 0,
    USART1_IRQHandler,
    USART2_IRQHandler,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0,
    ADC1_2_IRQHandler,
    0,
    EXTI0_IRQHandler,
    0, 0, 0,
};

/* SystemInit: HSE + PLL → 72 MHz */
void SystemInit(void) {
    /* Flash: 2 wait states for 72 MHz */
    FLASH.ACR |= FLASH_ACR_LATENCY_2;

    /* Enable HSE */
    RCC.CR |= RCC_CR_HSEON;
    while (!(RCC.CR & RCC_CR_HSERDY));

    /* PLL source = HSE, PLL × 9 = 72 MHz */
    RCC.CFGR |= RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMUL_9;

    /* APB1 prescaler /2 → 36 MHz (max allowed) */
    RCC.CFGR |= RCC_CFGR_PPRE1_DIV2;

    /* PLL on */
    RCC.CR |= RCC_CR_PLLON;
    while (!(RCC.CR & RCC_CR_PLLRDY));

    /* Switch system clock to PLL */
    RCC.CFGR = (RCC.CFGR & ~0x3) | RCC_CFGR_SW_PLL;
    while ((RCC.CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);

    /* Enable DWT cycle counter (uses CPU clock) */
    CoreDebug.DEMCR |= (1 << 24);
    DWT.CYCCNT = 0;
    DWT.CTRL |= (1 << 0);

    /* 1ms SysTick @ 72 MHz */
    SysTick.LOAD = 72000 - 1;
    SysTick.VAL = 0;
    SysTick.CTRL = SysTick_CTRL_ENABLE | SysTick_CTRL_TICKINT | SysTick_CTRL_CLKSOURCE;
}

/* Reset handler: copy .data, zero .bss, call main */
extern uint32_t _etext, _sdata, _edata, _sbss, _ebss;

void __attribute__((noreturn)) Reset_Handler(void) {
    /* Copy .data from flash to RAM */
    uint32_t *src = &_etext;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) {
        *dst++ = *src++;
        __asm__ volatile ("" ::: "memory");
    }

    /* Zero .bss */
    dst = &_sbss;
    while (dst < &_ebss) {
        *dst++ = 0;
        __asm__ volatile ("" ::: "memory");
    }

    SystemInit();
    main();
    while (1) { __asm__ volatile ("wfi"); }
}
