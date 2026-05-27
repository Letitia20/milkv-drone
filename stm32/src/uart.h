#pragma once
#include <stdint.h>

/* UART to Milk-V: USART2, 115200 8N1, non-blocking */

void uart_init(int baudrate);

/* Non-blocking: returns 0 if no line available, 1 if a line was copied */
int uart_read_line(char *buf, int max_len);

/* Non-blocking: returns 1 on success, 0 if buffer full */
int uart_write_line(const char *line);

/* Returns 1 if UART is initialized and working */
int uart_is_ok(void);
