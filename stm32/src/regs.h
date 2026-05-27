#pragma once
#include <stdint.h>

/* --- ARM Cortex-M3 core peripherals --- */
typedef struct {
    volatile uint32_t CPUID;
    volatile uint32_t ICSR;
    volatile uint32_t VTOR;
    volatile uint32_t AIRCR;
    volatile uint32_t SCR;
    volatile uint32_t CCR;
    volatile uint32_t SHPR[3];
    volatile uint32_t SHCSR;
    volatile uint32_t CFSR;
    volatile uint32_t HFSR;
    volatile uint32_t DFSR;
    volatile uint32_t MMFAR;
    volatile uint32_t BFAR;
    volatile uint32_t AFSR;
} SCB_Type;

typedef struct {
    volatile uint32_t CTRL;
    volatile uint32_t LOAD;
    volatile uint32_t VAL;
    volatile uint32_t CALIB;
} SysTick_Type;

typedef struct {
    volatile uint32_t ISER[3];
    uint32_t _reserved0[29];
    volatile uint32_t ICER[3];
    uint32_t _reserved1[29];
    volatile uint32_t ISPR[3];
    uint32_t _reserved2[29];
    volatile uint32_t ICPR[3];
    uint32_t _reserved3[29];
    volatile uint32_t IABR[3];
    uint32_t _reserved4[61];
    volatile uint8_t  IP[84];
    uint32_t _reserved5[643];
    volatile uint32_t STIR;
} NVIC_Type;

typedef struct {
    volatile uint32_t CTRL;
    volatile uint32_t CYCCNT;
    volatile uint32_t CPICNT;
    volatile uint32_t EXCCNT;
    volatile uint32_t SLEEPCNT;
    volatile uint32_t LSUCNT;
    volatile uint32_t FOLDCNT;
    volatile uint32_t PCSR;
} DWT_Type;

typedef struct {
    volatile uint32_t DEMCR;
} CoreDebug_Type;

#define SCB_BASE        ((SCB_Type *)0xE000ED00)
#define SysTick_BASE    ((SysTick_Type *)0xE000E010)
#define NVIC_BASE       ((NVIC_Type *)0xE000E100)
#define DWT_BASE        ((DWT_Type *)0xE0001000)
#define CoreDebug_BASE  ((CoreDebug_Type *)0xE000EDF0)

#define SCB         (*SCB_BASE)
#define SysTick     (*SysTick_BASE)
#define NVIC        (*NVIC_BASE)
#define DWT         (*DWT_BASE)
#define CoreDebug   (*CoreDebug_BASE)

/* --- STM32F103 peripherals --- */

/* RCC */
typedef struct {
    volatile uint32_t CR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t APB1RSTR;
    volatile uint32_t AHBENR;
    volatile uint32_t APB2ENR;
    volatile uint32_t APB1ENR;
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
} RCC_Type;

#define RCC_BASE        ((RCC_Type *)0x40021000)
#define RCC             (*RCC_BASE)

/* FLASH */
typedef struct {
    volatile uint32_t ACR;
    volatile uint32_t KEYR;
    volatile uint32_t OPTKEYR;
    volatile uint32_t SR;
    volatile uint32_t CR;
    volatile uint32_t AR;
    volatile uint32_t RESERVED;
    volatile uint32_t OBR;
    volatile uint32_t WRPR;
} FLASH_Type;

#define FLASH_BASE      ((FLASH_Type *)0x40022000)
#define FLASH           (*FLASH_BASE)

#define FLASH_ACR_LATENCY_2 (2 << 0)

/* GPIO */
typedef struct {
    volatile uint32_t CRL;
    volatile uint32_t CRH;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t BRR;
    volatile uint32_t LCKR;
} GPIO_Type;

#define GPIOA_BASE      ((GPIO_Type *)0x40010800)
#define GPIOB_BASE      ((GPIO_Type *)0x40010C00)
#define GPIOC_BASE      ((GPIO_Type *)0x40011000)
#define GPIOA           (*GPIOA_BASE)
#define GPIOB           (*GPIOB_BASE)
#define GPIOC           (*GPIOC_BASE)

/* AFIO */
typedef struct {
    volatile uint32_t EVCR;
    volatile uint32_t MAPR;
    volatile uint32_t EXTICR[4];
    uint32_t _reserved;
    volatile uint32_t MAPR2;
} AFIO_Type;

#define AFIO_BASE       ((AFIO_Type *)0x40010000)
#define AFIO            (*AFIO_BASE)

/* EXTI */
typedef struct {
    volatile uint32_t IMR;
    volatile uint32_t EMR;
    volatile uint32_t RTSR;
    volatile uint32_t FTSR;
    volatile uint32_t SWIER;
    volatile uint32_t PR;
} EXTI_Type;

#define EXTI_BASE       ((EXTI_Type *)0x40010400)
#define EXTI            (*EXTI_BASE)

/* USART */
typedef struct {
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
} USART_Type;

#define USART1_BASE     ((USART_Type *)0x40013800)
#define USART2_BASE     ((USART_Type *)0x40004400)
#define USART1          (*USART1_BASE)
#define USART2          (*USART2_BASE)

/* TIM */
typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t _reserved1;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    volatile uint32_t _reserved2;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;
} TIM_Type;

#define TIM2_BASE       ((TIM_Type *)0x40000000)
#define TIM3_BASE       ((TIM_Type *)0x40000400)
#define TIM2            (*TIM2_BASE)
#define TIM3            (*TIM3_BASE)

/* ADC */
typedef struct {
    volatile uint32_t SR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMPR1;
    volatile uint32_t SMPR2;
    volatile uint32_t JOFR1;
    volatile uint32_t JOFR2;
    volatile uint32_t JOFR3;
    volatile uint32_t JOFR4;
    volatile uint32_t HTR;
    volatile uint32_t LTR;
    volatile uint32_t SQR1;
    volatile uint32_t SQR2;
    volatile uint32_t SQR3;
    volatile uint32_t JSQR;
    volatile uint32_t JDR1;
    volatile uint32_t JDR2;
    volatile uint32_t JDR3;
    volatile uint32_t JDR4;
    volatile uint32_t DR;
} ADC_Type;

#define ADC1_BASE       ((ADC_Type *)0x40012400)
#define ADC1            (*ADC1_BASE)

/* --- Bit definitions --- */

/* RCC CR */
#define RCC_CR_HSEON        (1 << 16)
#define RCC_CR_HSERDY       (1 << 17)
#define RCC_CR_PLLON        (1 << 24)
#define RCC_CR_PLLRDY       (1 << 25)

/* RCC CFGR */
#define RCC_CFGR_PLLMUL_9   (7 << 18)

/* RCC APB2ENR */
#define RCC_APB2ENR_AFIOEN  (1 << 0)
#define RCC_APB2ENR_IOPAEN  (1 << 2)
#define RCC_APB2ENR_IOPBEN  (1 << 3)
#define RCC_APB2ENR_IOPCEN  (1 << 4)
#define RCC_APB2ENR_ADC1EN  (1 << 9)
#define RCC_APB2ENR_TIM1EN  (1 << 11)
#define RCC_APB2ENR_USART1EN (1 << 14)

/* RCC APB1ENR */
#define RCC_APB1ENR_TIM2EN  (1 << 0)
#define RCC_APB1ENR_TIM3EN  (1 << 1)
#define RCC_APB1ENR_USART2EN (1 << 17)

/* GPIO CRL/CRH mode bits */
#define GPIO_MODE_INPUT     0x0
#define GPIO_MODE_OUT10     0x1
#define GPIO_MODE_OUT2      0x2
#define GPIO_MODE_OUT50     0x3
#define GPIO_CNF_ANALOG     0x0
#define GPIO_CNF_IN_FLOAT   0x1
#define GPIO_CNF_IN_PUPD    0x2
#define GPIO_CNF_OUT_PP     0x0
#define GPIO_CNF_OUT_OD     0x1
#define GPIO_CNF_ALT_PP     0x2
#define GPIO_CNF_ALT_OD     0x3

/* USART CR1 */
#define USART_CR1_UE        (1 << 13)
#define USART_CR1_RE        (1 << 2)
#define USART_CR1_TE        (1 << 3)
#define USART_CR1_RXNEIE    (1 << 5)

/* USART SR */
#define USART_SR_TXE        (1 << 7)
#define USART_SR_RXNE       (1 << 5)

/* TIM CR1 */
#define TIM_CR1_CEN         (1 << 0)
#define TIM_CR1_ARPE        (1 << 7)

/* TIM CCMR1/2 OC mode */
#define TIM_CCMR_PWM1       (6 << 4)
#define TIM_CCMR_OC1PE      (1 << 3)

/* TIM CCER */
#define TIM_CCER_CC1E       (1 << 0)
#define TIM_CCER_CC2E       (1 << 4)
#define TIM_CCER_CC3E       (1 << 8)
#define TIM_CCER_CC4E       (1 << 12)

/* TIM EGR */
#define TIM_EGR_UG          (1 << 0)

/* ADC CR1 */
#define ADC_CR1_EOCIE       (1 << 5)

/* ADC CR2 */
#define ADC_CR2_ADON        (1 << 0)
#define ADC_CR2_CAL         (1 << 2)
#define ADC_CR2_RSTCAL      (1 << 3)
#define ADC_CR2_SWSTART     (1 << 22)
#define ADC_CR2_EXTTRIG     (1 << 20)
#define ADC_CR2_EXTSEL_SW   (7 << 17)

/* ADC SR */
#define ADC_SR_EOC          (1 << 1)

/* RCC CFGR */
#define RCC_CFGR_SW_HSI     0
#define RCC_CFGR_SW_PLL     (2 << 0)
#define RCC_CFGR_SWS_Msk     (3 << 2)
#define RCC_CFGR_SWS_PLL     (2 << 2)
#define RCC_CFGR_PLLSRC_HSE  (1 << 16)
#define RCC_CFGR_PPRE1_DIV2  (4 << 8)

/* SysTick */
#define SysTick_CTRL_ENABLE     (1 << 0)
#define SysTick_CTRL_TICKINT    (1 << 1)
#define SysTick_CTRL_CLKSOURCE  (1 << 2)
