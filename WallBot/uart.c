#include "uart.h"
#include <avr/io.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

void uart_init(uint32_t baud) {
    /* UBRR = F_CPU / (16 * baud) - 1, rounded. */
    uint16_t ubrr = (uint16_t)((F_CPU + 8UL * baud) / (16UL * baud) - 1UL);
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr & 0xFF);

    /* Enable TX (and RX, harmless even if unused). */
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    /* 8 data bits, 1 stop, no parity (URSEL not needed on 328P). */
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_putc(char c) {
    while (!(UCSR0A & (1 << UDRE0))) { }
    UDR0 = (uint8_t)c;
}

void uart_puts(const char *s) {
    while (*s) uart_putc(*s++);
}

void uart_put_u16(uint16_t v) {
    char buf[6];                /* max "65535" + NUL */
    uint8_t i = 0;
    if (v == 0) { uart_putc('0'); return; }
    while (v && i < sizeof(buf)) {
        buf[i++] = (char)('0' + (v % 10));
        v /= 10;
    }
    while (i--) uart_putc(buf[i]);
}

void uart_put_i16(int16_t v) {
    if (v < 0) { uart_putc('-'); v = -v; }
    uart_put_u16((uint16_t)v);
}
