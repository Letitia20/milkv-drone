#include "uart.h"
#include "regs.h"

/* USART2: TX=PA2, RX=PA3 */
#define UART_RX_BUF_SIZE    256

static char     g_rx_buf[UART_RX_BUF_SIZE];
static int      g_rx_head = 0;
static int      g_rx_tail = 0;
static int      g_ok = 0;

void uart_init(int baudrate) {
    /* Enable clocks */
    RCC.APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC.APB1ENR |= RCC_APB1ENR_USART2EN;

    /* PA2 = USART2_TX (alt push-pull, 50 MHz) */
    {
        uint32_t crl = GPIOA.CRL;
        crl &= ~(0xF << (2 * 4));
        crl |= (GPIO_MODE_OUT50 | GPIO_CNF_ALT_PP) << (2 * 4);
        GPIOA.CRL = crl;
    }

    /* PA3 = USART2_RX (input floating) */
    {
        uint32_t crl = GPIOA.CRL;
        crl &= ~(0xF << (3 * 4));
        crl |= (GPIO_CNF_IN_FLOAT << 2) << (3 * 4);
        GPIOA.CRL = crl;
    }

    /* Baud rate: PCLK1 = 36 MHz (HCLK=72 / APB1 prescaler=2) */
    USART2.BRR = (36000000 + baudrate / 2) / baudrate;
    USART2.CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

    /* Enable USART2 interrupt in NVIC (IRQ 38) */
    NVIC.ISER[1] = 1 << (38 - 32);

    g_ok = 1;
}

int uart_is_ok(void) {
    return g_ok;
}

int uart_read_line(char *buf, int max_len) {
    if (max_len <= 0) return 0;

    int i = 0;
    while (g_rx_head != g_rx_tail && i < max_len - 1) {
        char c = g_rx_buf[g_rx_tail];
        g_rx_tail = (g_rx_tail + 1) % UART_RX_BUF_SIZE;
        buf[i++] = c;
        if (c == '\n') break;
    }
    buf[i] = '\0';
    return i > 0;
}

int uart_write_line(const char *line) {
    if (!g_ok) return 0;

    while (*line) {
        /* Wait for TXE with timeout */
        int timeout = 100000;
        while (!(USART2.SR & USART_SR_TXE) && --timeout);
        if (timeout == 0) { g_ok = 0; return 0; }
        USART2.DR = *line++;
    }

    /* Send newline */
    int timeout = 100000;
    while (!(USART2.SR & USART_SR_TXE) && --timeout);
    if (timeout == 0) { g_ok = 0; return 0; }
    USART2.DR = '\n';

    return 1;
}

/* USART2 interrupt handler — store received bytes into ring buffer */
void USART2_IRQHandler(void) {
    if (USART2.SR & USART_SR_RXNE) {
        char c = (char)USART2.DR;
        int next = (g_rx_head + 1) % UART_RX_BUF_SIZE;
        if (next != g_rx_tail) {
            g_rx_buf[g_rx_head] = c;
            g_rx_head = next;
        }
    }
}
