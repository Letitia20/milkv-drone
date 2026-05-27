#include "ibus.h"
#include "uart.h"
#include "pwm.h"
#include "adc.h"
#include "regs.h"

/* Minimal my_strncmp — avoid pulling in libc */
static int my_strncmp(const char *a, const char *b, int n) {
    for (int i = 0; i < n; i++) {
        if (a[i] != b[i]) return (unsigned char)a[i] - (unsigned char)b[i];
        if (a[i] == '\0') return 0;
    }
    return 0;
}

/* ===== Pin assignments =====
 * PA0     iBUS signal (GPIO input, floating)
 * PA1     emergency stop button (GPIO input, pull-up, active low)
 * PA2     USART2_TX → Milk-V
 * PA3     USART2_RX → Milk-V
 * PA4     battery ADC (ADC_IN4)
 * PA6     TIM3_CH1 → Motor 1
 * PA7     TIM3_CH2 → Motor 2
 * PB0     TIM3_CH3 → Motor 3
 * PB1     TIM3_CH4 → Motor 4
 * PC13    status LED (active low)
 */

/* ===== Constants ===== */
#define MILKV_BAUDRATE      115200
#define MILKV_HEARTBEAT_MS  500
#define IBUS_TIMEOUT_MS     500
#define TELEMETRY_PERIOD_MS 100
#define BATTERY_PERIOD_MS   500

/* Emergency stop: active low (ground = stop) */
#define ESTOP_ACTIVE()      (!((GPIOA.IDR >> 1) & 1))

/* LED */
#define LED_ON()            (GPIOC.BRR = (1 << 13))
#define LED_OFF()           (GPIOC.BSRR = (1 << 13))
#define LED_TOGGLE()        (GPIOC.ODR ^= (1 << 13))

/* ===== Global state ===== */
static volatile uint32_t g_ticks_ms = 0;

static uint32_t g_last_milkv_ms = 0;
static uint8_t  g_armed = 0;
static uint16_t g_motors[4] = {PWM_MIN_US, PWM_MIN_US, PWM_MIN_US, PWM_MIN_US};

static uint32_t g_next_telemetry_ms = 0;
static uint32_t g_next_battery_ms = 0;
static uint32_t g_next_led_ms = 0;

/* ===== SysTick ===== */
void SysTick_Handler(void) {
    g_ticks_ms++;
}

/* ===== Simple itoa (avoiding sprintf) ===== */
static int itoa_simple(int val, char *buf) {
    if (val < 0) {
        *buf++ = '-';
        val = -val;
    }
    if (val == 0) {
        *buf++ = '0';
        *buf = '\0';
        return 1;
    }
    char tmp[12];
    int pos = 0;
    while (val > 0) {
        tmp[pos++] = '0' + (val % 10);
        val /= 10;
    }
    int len = pos;
    for (int i = len; i > 0; i--) buf[i - 1] = tmp[len - i];
    buf += len;
    *buf = '\0';
    return len;
}

/* ===== Protocol parsing (avoiding sscanf) ===== */

/* Parse "MOT,m1,m2,m3,m4" */
static int parse_mot(const char *line, uint16_t motors[4]) {
    if (my_strncmp(line, "MOT,", 4) != 0) return 0;
    line += 4;

    for (int i = 0; i < 4; i++) {
        int val = 0;
        while (*line >= '0' && *line <= '9') {
            val = val * 10 + (*line - '0');
            line++;
        }
        if (val < 900 || val > 2100) return 0;
        motors[i] = (uint16_t)val;
        if (i < 3) {
            if (*line != ',') return 0;
            line++;
        }
    }
    return 1;
}

static int parse_arm(const char *line) {
    return my_strncmp(line, "ARM", 3) == 0 && (line[3] == '\0' || line[3] == '\r');
}

static int parse_disarm(const char *line) {
    return my_strncmp(line, "DISARM", 6) == 0 && (line[6] == '\0' || line[6] == '\r');
}

/* ===== GPIO init ===== */
static void gpio_init(void) {
    /* Enable GPIO clocks */
    RCC.APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;

    /* PA1 = emergency stop (input with pull-up) */
    {
        uint32_t crl = GPIOA.CRL;
        crl &= ~(0xF << (1 * 4));
        crl |= ((GPIO_CNF_IN_PUPD << 2) | GPIO_MODE_INPUT) << (1 * 4);
        GPIOA.CRL = crl;
        GPIOA.ODR |= (1 << 1);  /* pull-up */
    }

    /* PC13 = status LED (output, push-pull, 2 MHz) */
    {
        uint32_t crh = GPIOC.CRH;
        crh &= ~(0xF << ((13 - 8) * 4));
        crh |= ((GPIO_CNF_OUT_PP << 2) | GPIO_MODE_OUT2) << ((13 - 8) * 4);
        GPIOC.CRH = crh;
        LED_OFF();
    }
}

/* ===== Safety check ===== */
static int safety_ok(int ibus_ok, int milkv_timeout) {
    if (ESTOP_ACTIVE()) return 0;
    if (milkv_timeout)   return 0;
    if (!ibus_ok)        return 0;
    return 1;
}

/* ===== Main ===== */
int main(void) {
    gpio_init();
    ibus_init();
    uart_init(MILKV_BAUDRATE);
    pwm_init();
    adc_init();

    /* Blink LED quickly (200 ms) on startup */
    for (int i = 0; i < 10; i++) {
        LED_TOGGLE();
        for (volatile int d = 0; d < 500000; d++);
    }
    LED_OFF();

    char line_buf[64];
    int milkv_timeout = 0;
    int ibus_ok = 0;
    uint16_t battery_mv = 0;

    adc_start_conversion();

    for (;;) {
        uint32_t now_ms = g_ticks_ms;

        /* --- Poll iBUS --- */
        ibus_poll();
        {
            const ibus_data_t *ibus = ibus_get_data();
            ibus_ok = !ibus->failsafe;
            /* Clear fresh flag; we'll encode RC data below */
        }

        /* --- Read Milk-V UART --- */
        while (uart_read_line(line_buf, sizeof(line_buf))) {
            uint16_t motors[4];
            if (parse_mot(line_buf, motors)) {
                g_motors[0] = motors[0];
                g_motors[1] = motors[1];
                g_motors[2] = motors[2];
                g_motors[3] = motors[3];
                g_last_milkv_ms = now_ms;
            } else if (parse_arm(line_buf)) {
                g_armed = 1;
                g_last_milkv_ms = now_ms;
            } else if (parse_disarm(line_buf)) {
                g_armed = 0;
                g_last_milkv_ms = now_ms;
            }
        }

        /* Check Milk-V heartbeat timeout */
        milkv_timeout = ((int32_t)(now_ms - g_last_milkv_ms) > MILKV_HEARTBEAT_MS);

        /* --- Safety evaluation --- */
        int safe = safety_ok(ibus_ok, milkv_timeout);

        /* --- Update PWM --- */
        if (safe && g_armed) {
            pwm_set_all(g_motors[0], g_motors[1], g_motors[2], g_motors[3]);
        } else {
            pwm_stop();
            g_armed = 0;  /* safety trips disarm */
        }

        /* --- Send telemetry to Milk-V --- */
        if ((int32_t)(now_ms - g_next_telemetry_ms) >= 0) {
            const ibus_data_t *ibus = ibus_get_data();
            /* Format: RC,ch1,...,ch6,failsafe */
            char rc_buf[48];
            rc_buf[0] = 'R'; rc_buf[1] = 'C'; rc_buf[2] = ',';
            int pos = 3;
            for (int ch = 0; ch < 6; ch++) {
                pos += itoa_simple(ibus->channels[ch], rc_buf + pos);
                rc_buf[pos++] = ',';
            }
            pos += itoa_simple(ibus->failsafe ? 1 : 0, rc_buf + pos);
            rc_buf[pos] = '\0';
            uart_write_line(rc_buf);
            g_next_telemetry_ms = now_ms + TELEMETRY_PERIOD_MS;
        }

        /* --- Send battery voltage to Milk-V --- */
        if ((int32_t)(now_ms - g_next_battery_ms) >= 0) {
            adc_start_conversion();
            /* Small delay for conversion to complete */
            for (volatile int d = 0; d < 1000; d++);
            battery_mv = adc_read_mv();

            char bat_buf[16];
            bat_buf[0] = 'B'; bat_buf[1] = 'A'; bat_buf[2] = 'T'; bat_buf[3] = ',';
            int pos = 4;
            if (battery_mv > 0) {
                pos += itoa_simple(battery_mv, bat_buf + pos);
            } else {
                bat_buf[pos++] = '0';
                bat_buf[pos] = '\0';
            }
            bat_buf[pos] = '\0';
            uart_write_line(bat_buf);
            g_next_battery_ms = now_ms + BATTERY_PERIOD_MS;
        }

        /* --- Status LED --- */
        if ((int32_t)(now_ms - g_next_led_ms) >= 0) {
            if (!safe) {
                if (milkv_timeout) {
                    /* 200 ms blink = no Milk-V communication */
                    LED_TOGGLE();
                    g_next_led_ms = now_ms + 200;
                } else {
                    /* 600 ms blink = Milk-V connected, waiting for arm */
                    LED_TOGGLE();
                    g_next_led_ms = now_ms + 600;
                }
            } else if (g_armed) {
                /* 1 second blink = armed */
                LED_TOGGLE();
                g_next_led_ms = now_ms + 1000;
            } else {
                /* Safe but disarmed */
                LED_TOGGLE();
                g_next_led_ms = now_ms + 500;
            }
        }
    }

    return 0;
}
