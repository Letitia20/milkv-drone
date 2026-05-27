#include "pwm.h"
#include "regs.h"

/* TIM3: CH1=PA6, CH2=PA7, CH3=PB0, CH4=PB1 */
/* Timer clock = 72 MHz, prescaler = 72 → 1 MHz → 1 us tick */
/* ARR = 20000 - 1 for 50 Hz (20 ms period) */

#define TIM_PSC     72
#define TIM_ARR     (PWM_PERIOD_US - 1)

void pwm_init(void) {
    /* Enable clocks */
    RCC.APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
    RCC.APB1ENR |= RCC_APB1ENR_TIM3EN;

    /* PA6 = TIM3_CH1 (alt push-pull, 50 MHz) */
    {
        uint32_t crl = GPIOA.CRL;
        crl &= ~(0xF << (6 * 4));
        crl |= (GPIO_MODE_OUT50 | GPIO_CNF_ALT_PP) << (6 * 4);
        GPIOA.CRL = crl;
    }

    /* PA7 = TIM3_CH2 (alt push-pull, 50 MHz) */
    {
        uint32_t crl = GPIOA.CRL;
        crl &= ~(0xF << (7 * 4));
        crl |= (GPIO_MODE_OUT50 | GPIO_CNF_ALT_PP) << (7 * 4);
        GPIOA.CRL = crl;
    }

    /* PB0 = TIM3_CH3 (alt push-pull, 50 MHz) */
    {
        uint32_t crl = GPIOB.CRL;
        crl &= ~(0xF << (0 * 4));
        crl |= (GPIO_MODE_OUT50 | GPIO_CNF_ALT_PP) << (0 * 4);
        GPIOB.CRL = crl;
    }

    /* PB1 = TIM3_CH4 (alt push-pull, 50 MHz) */
    {
        uint32_t crl = GPIOB.CRL;
        crl &= ~(0xF << (1 * 4));
        crl |= (GPIO_MODE_OUT50 | GPIO_CNF_ALT_PP) << (1 * 4);
        GPIOB.CRL = crl;
    }

    /* Configure TIM3 */
    TIM3.PSC = TIM_PSC - 1;
    TIM3.ARR = TIM_ARR;
    TIM3.CNT = 0;

    /* PWM mode 1 on all 4 channels, preload enabled */
    TIM3.CCMR1 |= TIM_CCMR_PWM1 | TIM_CCMR_OC1PE;  /* CH1 */
    TIM3.CCMR1 |= (TIM_CCMR_PWM1 | TIM_CCMR_OC1PE) << 8;  /* CH2 */
    TIM3.CCMR2 |= TIM_CCMR_PWM1 | TIM_CCMR_OC1PE;  /* CH3 */
    TIM3.CCMR2 |= (TIM_CCMR_PWM1 | TIM_CCMR_OC1PE) << 8;  /* CH4 */

    /* Enable outputs, all at minimum first */
    TIM3.CCR1 = PWM_MIN_US;
    TIM3.CCR2 = PWM_MIN_US;
    TIM3.CCR3 = PWM_MIN_US;
    TIM3.CCR4 = PWM_MIN_US;
    TIM3.CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    /* Generate update to load shadow registers, then start timer */
    TIM3.EGR = TIM_EGR_UG;
    TIM3.CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

static uint16_t clamp_pulse(uint16_t pulse_us) {
    if (pulse_us < PWM_MIN_US) return PWM_MIN_US;
    if (pulse_us > PWM_MAX_US) return PWM_MAX_US;
    return pulse_us;
}

void pwm_set(uint8_t channel, uint16_t pulse_us) {
    pulse_us = clamp_pulse(pulse_us);
    switch (channel) {
        case 1: TIM3.CCR1 = pulse_us; break;
        case 2: TIM3.CCR2 = pulse_us; break;
        case 3: TIM3.CCR3 = pulse_us; break;
        case 4: TIM3.CCR4 = pulse_us; break;
    }
}

void pwm_set_all(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
    TIM3.CCR1 = clamp_pulse(m1);
    TIM3.CCR2 = clamp_pulse(m2);
    TIM3.CCR3 = clamp_pulse(m3);
    TIM3.CCR4 = clamp_pulse(m4);
}

void pwm_stop(void) {
    TIM3.CCR1 = PWM_MIN_US;
    TIM3.CCR2 = PWM_MIN_US;
    TIM3.CCR3 = PWM_MIN_US;
    TIM3.CCR4 = PWM_MIN_US;
}
