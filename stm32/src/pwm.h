#pragma once
#include <stdint.h>

/* ESC PWM: 4 channels on TIM3, 50 Hz (20 ms period), 1000-2000 us pulse */

#define PWM_MIN_US  1000
#define PWM_MAX_US  2000
#define PWM_PERIOD_US 20000

void pwm_init(void);

/* Set pulse width for channel 1-4 (values 1000-2000, clamped). Channel is 1-indexed. */
void pwm_set(uint8_t channel, uint16_t pulse_us);

/* Set all 4 channels at once */
void pwm_set_all(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

/* Force all outputs to minimum (1000 us) — emergency stop */
void pwm_stop(void);
