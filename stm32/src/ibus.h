#pragma once
#include <stdint.h>

/* iBUS frame: 32 bytes at 115200 baud, inverted UART */
#define IBUS_FRAME_SIZE     32
#define IBUS_HEADER_BYTE0   0x20
#define IBUS_HEADER_BYTE1   0x40

/* Parsed channel values (raw, 1000-2000 range) */
typedef struct {
    uint16_t channels[6];   /* ch0=roll, ch1=pitch, ch2=throttle, ch3=yaw, ch4=aux1, ch5=aux2 */
    uint8_t  failsafe;      /* 1 = receiver signal lost */
    uint8_t  fresh;         /* 1 = new frame received since last read */
} ibus_data_t;

/* Call from main loop at >500Hz. Non-blocking, idle-to-low for inverted signal. */
void ibus_init(void);
void ibus_poll(void);

/* Returns pointer to latest parsed data */
const ibus_data_t* ibus_get_data(void);
