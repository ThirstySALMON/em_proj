/*
 * uart.h — minimal polled USART0 driver (TX + optional RX).
 * 8N1, blocking writes. Used for debug prints over USB-serial during
 * bring-up; the same pins drive the HC-05 in flight.
 */
#ifndef UART_H
#define UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void uart_init(uint32_t baud);
void uart_putc(char c);
void uart_puts(const char *s);
void uart_put_u16(uint16_t v);     /* unsigned decimal */
void uart_put_i16(int16_t v);      /* signed decimal */

#ifdef __cplusplus
}
#endif

#endif /* UART_H */
