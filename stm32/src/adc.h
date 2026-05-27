#pragma once
#include <stdint.h>

/* Battery voltage via resistor divider → PA4 (ADC_IN4) */

void adc_init(void);

/* Returns battery voltage in millivolts, 0 if not yet converted */
uint16_t adc_read_mv(void);

/* Start a new conversion (non-blocking, result available next loop) */
void adc_start_conversion(void);
