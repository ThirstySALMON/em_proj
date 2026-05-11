/*
 * uart.h - interrupt-driven USART0 driver (TX + RX), 8N1.
 * Send functions enqueue bytes and return immediately. If the TX buffer
 * fills, extra bytes are dropped instead of blocking the control loop.
 */
#ifndef UART_H
#define UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void UART_init(unsigned long baud);
void UART_sendChar(char c);
void UART_sendString(const char* str);
void UART_sendInt(int n);
uint8_t UART_receiveChar(char* data);
uint8_t UART_receiveString(char* buf, uint8_t maxLen);

#ifdef __cplusplus
}
#endif

#endif /* UART_H */
