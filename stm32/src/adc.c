#include "adc.h"
#include "regs.h"

/* PA4 = ADC_IN4
 * Assumes resistor divider: R1 (top) = 10k, R2 (bottom) = 2.2k
 * Divider ratio: Vout = Vin * R2 / (R1 + R2) = Vin * 2.2 / 12.2 ≈ Vin * 0.18
 * For 3S battery (11.1V nominal): Vout = 11.1 * 0.18 = 2.0V (safe for 3.3V ADC)
 * Vin = Vout * (R1 + R2) / R2 = ADC_voltage * 12.2 / 2.2
 */

#define DIVIDER_RATIO_NUM   122   /* (10 + 2.2) * 10 for integer math */
#define DIVIDER_RATIO_DEN   22    /* 2.2 * 10 */

static uint16_t g_last_mv = 0;

void adc_init(void) {
    /* Enable clocks */
    RCC.APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN;

    /* PA4 as analog input */
    {
        uint32_t crl = GPIOA.CRL;
        crl &= ~(0xF << (4 * 4));
        GPIOA.CRL = crl;  /* 0000 = analog mode */
    }

    /* ADC calibration */
    ADC1.CR1 = 0;
    ADC1.CR2 = ADC_CR2_ADON | ADC_CR2_EXTSEL_SW;
    ADC1.SMPR2 = (5 << 12);  /* CH4 sample time = 55.5 cycles */

    /* Calibrate */
    ADC1.CR2 |= ADC_CR2_RSTCAL;
    while (ADC1.CR2 & ADC_CR2_RSTCAL);
    ADC1.CR2 |= ADC_CR2_CAL;
    while (ADC1.CR2 & ADC_CR2_CAL);

    /* Select CH4 as the first regular channel, 1 conversion */
    ADC1.SQR3 = 4;
    ADC1.SQR1 = 0;
}

void adc_start_conversion(void) {
    ADC1.CR2 |= ADC_CR2_ADON;
    ADC1.CR2 |= ADC_CR2_SWSTART;
}

uint16_t adc_read_mv(void) {
    if (!(ADC1.SR & ADC_SR_EOC)) {
        return g_last_mv;
    }

    uint16_t raw = (uint16_t)ADC1.DR;
    /* Vref = 3.3V, 12-bit ADC: V = raw * 3300 / 4096 */
    uint32_t vout_mv = (uint32_t)raw * 3300 / 4096;
    /* Vin = Vout * divider_ratio = Vout * 12.2 / 2.2 */
    uint32_t vin_mv = vout_mv * DIVIDER_RATIO_NUM / DIVIDER_RATIO_DEN;

    g_last_mv = (uint16_t)vin_mv;
    return g_last_mv;
}
