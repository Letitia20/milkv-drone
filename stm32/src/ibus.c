#include "ibus.h"
#include "regs.h"

/* Connect iBUS signal to PA0 (inverted UART, idle=low, start=high) */
#define IBUS_GPIO       GPIOA
#define IBUS_PIN        0

static ibus_data_t g_ibus = { .failsafe = 1, .fresh = 0 };
static uint8_t  g_buf[IBUS_FRAME_SIZE];
static uint32_t g_last_frame_ticks = 0;

/* DWT-based microsecond delay (CPU @ 72 MHz) */
static void delay_cycles(uint32_t cycles) {
    uint32_t start = DWT.CYCCNT;
    while ((DWT.CYCCNT - start) < cycles);
}

static inline int ibus_pin_read(void) {
    return (IBUS_GPIO.IDR >> IBUS_PIN) & 1;
}

void ibus_init(void) {
    /* PA0 as floating input */
    uint32_t crl = GPIOA.CRL;
    crl &= ~(0xF << (IBUS_PIN * 4));
    crl |= (GPIO_CNF_IN_FLOAT << 2) << (IBUS_PIN * 4);
    GPIOA.CRL = crl;

    g_ibus.failsafe = 1;
    g_ibus.fresh = 0;
}

void ibus_poll(void) {
    /* Inverted UART: idle = low, start bit = high (rising edge) */
    if (ibus_pin_read() == 0) {
        return;  /* idle — nothing to read */
    }

    /* Start bit detected. Sample all 32 bytes with cycle-accurate timing.
     * 115200 baud → 8680 ns/bit → 625 CPU cycles at 72 MHz.
     * Wait 1.5 bits (937 cycles) to center on first data bit. */
    delay_cycles(937);

    for (int byte = 0; byte < IBUS_FRAME_SIZE; byte++) {
        uint8_t data = 0;
        for (int bit = 0; bit < 8; bit++) {
            if (ibus_pin_read()) data |= (1 << bit);
            delay_cycles(625);
        }
        g_buf[byte] = data;

        /* Check stop bit (should be low for inverted signal) */
        if (ibus_pin_read() != 0) {
            return;  /* framing error, discard frame */
        }
        /* Small gap between stop bit and next start bit */
        delay_cycles(200);
        if (byte < IBUS_FRAME_SIZE - 1) {
            /* Wait for next start bit (should be high) */
            int timeout = 1000;
            while (ibus_pin_read() == 0 && --timeout) delay_cycles(100);
            if (timeout == 0) return;  /* timeout */
            delay_cycles(937);  /* 1.5 bits to data center */
        }
    }

    /* Validate header */
    if (g_buf[0] != IBUS_HEADER_BYTE0 || g_buf[1] != IBUS_HEADER_BYTE1) {
        return;
    }

    /* Verify checksum (16-bit sum of bytes 0-29, little-endian at bytes 30-31) */
    uint16_t sum = 0;
    for (int i = 0; i < 30; i++) sum += g_buf[i];
    uint16_t expected = (uint16_t)g_buf[30] | ((uint16_t)g_buf[31] << 8);
    if (sum != expected) {
        return;
    }

    /* Parse channels (12-bit, little-endian) */
    for (int ch = 0; ch < 6; ch++) {
        g_ibus.channels[ch] = (uint16_t)g_buf[2 + ch * 2]
                            | ((uint16_t)g_buf[3 + ch * 2] << 8);
    }
    g_ibus.failsafe = 0;
    g_ibus.fresh = 1;
    g_last_frame_ticks = SysTick.VAL;
}

const ibus_data_t* ibus_get_data(void) {
    return &g_ibus;
}
