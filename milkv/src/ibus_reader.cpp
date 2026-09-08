/* Milk-V 软解码 iBUS 功能：
 * 通过 GP3 读取反相 UART，使用 RISC-V rdcycle 做位时间采样。
 * 位时间公式：1 bit = 1e9 / 115200 ≈ 8680ns。
 *
 * iBUS software reader for Milk-V Duo
 * Reads inverted UART (idle low) from GPIO pin using RISC-V cycle counter timing.
 * Outputs "RC,ch1,ch2,ch3,ch4,ch5,ch6,failsafe" lines on stdout at 10 Hz. */
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

/* GPIO register base (Milk-V Duo / SG2002) */
#define GPIO_BASE       0x03020000
#define GPIO_SWPORTA_DR 0x00   /* data register */
#define GPIO_EXT_PORTA  0x50   /* input value register */

#define IBUS_PIN        3      /* GP3 */
#define IBUS_BAUD       115200
#define IBUS_FRAME_LEN  32

static volatile uint32_t *gpio = nullptr;

static inline uint64_t cycles() {
    uint64_t cyc;
    __asm__ volatile ("rdcycle %0" : "=r"(cyc));
    return cyc;
}

static inline int pin_read() {
    return (gpio[GPIO_EXT_PORTA / 4] >> IBUS_PIN) & 1;
}

/* Wait approximately `ns` nanoseconds using cycle counter (1 GHz nominal) */
static inline void delay_ns(uint64_t ns) {
    uint64_t start = cycles();
    while ((cycles() - start) < ns);
}

/* Try to read one complete iBUS frame. Returns 1 on success. */
static int ibus_read_frame(uint8_t buf[IBUS_FRAME_LEN]) {
    /* Inverted UART: idle = low, start bit = high (rising edge)
     * Wait for rising edge with timeout (~10 ms) */
    int timeout = 200000;  /* ~10 ms at ~50ns per iteration */
    while (pin_read() == 0 && --timeout) {
        for (volatile int i = 0; i < 10; i++);
    }
    if (timeout == 0) return 0;

    /* Start bit found. 1 bit = 1e9 / 115200 = 8680 ns.
     * Wait 1.5 bits = 13020 ns to center on first data bit. */
    delay_ns(13020);

    for (int byte = 0; byte < IBUS_FRAME_LEN; byte++) {
        uint8_t data = 0;
        for (int bit = 0; bit < 8; bit++) {
            if (pin_read()) data |= (1 << bit);
            delay_ns(8680);
        }
        buf[byte] = data;

        /* Stop bit: should be low for inverted signal */
        delay_ns(2000);  /* center of stop bit */
        if (pin_read() != 0) {
            return 0;  /* framing error */
        }

        /* Gap to next start bit (if any) */
        if (byte < IBUS_FRAME_LEN - 1) {
            /* Wait for rising edge of next start bit */
            timeout = 10000;
            while (pin_read() == 0 && --timeout) delay_ns(1000);
            if (timeout == 0) return 0;
            delay_ns(13020);  /* 1.5 bits to data center */
        }
    }

    return 1;
}

static void ibus_parse(const uint8_t buf[IBUS_FRAME_LEN], uint16_t channels[6], int *failsafe) {
    /* Check header: 0x20 0x40 */
    if (buf[0] != 0x20 || buf[1] != 0x40) {
        *failsafe = 1;
        return;
    }

    /* Checksum: 16-bit sum of bytes 0-29 */
    uint16_t sum = 0;
    for (int i = 0; i < 30; i++) sum += buf[i];
    uint16_t expected = (uint16_t)buf[30] | ((uint16_t)buf[31] << 8);
    if (sum != expected) {
        *failsafe = 1;
        return;
    }

    /* Parse 6 channels (12-bit, little-endian) */
    for (int ch = 0; ch < 6; ch++) {
        channels[ch] = (uint16_t)buf[2 + ch * 2] | ((uint16_t)buf[3 + ch * 2] << 8);
    }
    *failsafe = 0;
}

int main() {
    /* Open /dev/mem for GPIO access */
    int fd = ::open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        std::perror("open /dev/mem");
        return 1;
    }

    gpio = (volatile uint32_t *)::mmap(
        nullptr, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO_BASE);
    if (gpio == MAP_FAILED) {
        std::perror("mmap GPIO");
        return 1;
    }

    /* Configure GP3 as input (0 = input, enable bit at index 4) */
    /* For SG2002/CV181x, GPIO direction registers are at offset 0x04 */
    {
        uint32_t dir = gpio[0x04 / 4];
        dir &= ~(1 << IBUS_PIN);
        gpio[0x04 / 4] = dir;
    }

    std::fprintf(stderr, "ibus_reader: reading GP%d at %d baud\n", IBUS_PIN, IBUS_BAUD);

    uint8_t frame[IBUS_FRAME_LEN];
    uint16_t channels[6] = {1500, 1500, 1000, 1500, 1000, 1000};
    int failsafe = 1;
    int valid_frames = 0;

    for (;;) {
        if (ibus_read_frame(frame)) {
            ibus_parse(frame, channels, &failsafe);
            if (!failsafe) {
                valid_frames++;
                /* Print RC telemetry line (10Hz max when frames arrive) */
                std::printf("RC,%u,%u,%u,%u,%u,%u,%d\n",
                    channels[0], channels[1], channels[2],
                    channels[3], channels[4], channels[5],
                    failsafe);
                std::fflush(stdout);
            }
        }

        if (valid_frames > 50) {
            valid_frames = 0;
        }
    }

    return 0;
}
