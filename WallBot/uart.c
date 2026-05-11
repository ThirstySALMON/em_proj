#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "uart.h"


#define UART_RX_BUFFER_SIZE 64
#define UART_TX_BUFFER_SIZE 256

static volatile char uart_rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint8_t uart_rx_head = 0;
static volatile uint8_t uart_rx_tail = 0;
static volatile uint8_t uart_rx_overflow = 0;

static volatile char uart_tx_buffer[UART_TX_BUFFER_SIZE];
static volatile uint8_t uart_tx_head = 0;
static volatile uint8_t uart_tx_tail = 0;
static volatile uint8_t uart_tx_overflow = 0;

void UART_init(unsigned long baud)
{
  unsigned int ubrr = F_CPU / 16 / baud - 1;

  // Set baud rate
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;

  // Enable transmitter, receiver, and RX interrupt. TX interrupt is enabled
  // only when bytes are queued.
  UCSR0B = (1 << RXCIE0) | (1 << TXEN0) | (1 << RXEN0);

  // Set frame format: 8 data bits, 1 stop bit
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void UART_sendChar(char c)
{
  uint8_t next = (uint8_t)((uart_tx_head + 1) % UART_TX_BUFFER_SIZE);

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if (next == uart_tx_tail)
    {
      uart_tx_overflow = 1;
    }
    else
    {
      uart_tx_buffer[uart_tx_head] = c;
      uart_tx_head = next;
      UCSR0B |= (1 << UDRIE0);
    }
  }
}

void UART_sendString(const char *str)
{
  while (*str)
  {
    UART_sendChar(*str++);
  }
}

void UART_sendInt(int n)
{
  if (n < 0)
  {
    UART_sendChar('-');
    n = -n;
  }
  if (n == 0)
  {
    UART_sendChar('0');
    return;
  }

  char digits[6];
  uint8_t i = 0;

  while (n > 0)
  {
    digits[i++] = '0' + (n % 10);
    n /= 10;
  }

  while (i--)
  {
    UART_sendChar(digits[i]);
  }
}

uint8_t UART_receiveChar(char *data)
{
  uint8_t ok = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if (uart_rx_head != uart_rx_tail)
    {
      *data = uart_rx_buffer[uart_rx_tail];
      uart_rx_tail = (uint8_t)((uart_rx_tail + 1) % UART_RX_BUFFER_SIZE);
      ok = 1;
    }
  }
  return ok;
}

uint8_t UART_receiveString(char *buf, uint8_t maxLen)
{
  static uint8_t i = 0;
  char c;

  if (maxLen == 0)
    return 0;

  while (UART_receiveChar(&c))
  {
    if (c == '\r' || c == '\n')
    {
      uint8_t len = i;
      if (len == 0)
        continue;
      buf[i] = '\0';
      i = 0;
      return len;
    }

    if (i < maxLen - 1)
    {
      buf[i++] = c;
    }
    else
    {
      buf[i] = '\0';
      i = 0;
      return maxLen - 1;
    }
  }

  buf[i] = '\0';
  return 0;
}

ISR(USART_RX_vect)
{
  char c = UDR0;
  uint8_t next = (uint8_t)((uart_rx_head + 1) % UART_RX_BUFFER_SIZE);
  if (next != uart_rx_tail)
  {
    uart_rx_buffer[uart_rx_head] = c;
    uart_rx_head = next;
  }
  else
  {
    uart_rx_overflow = 1;
  }
}

ISR(USART_UDRE_vect)
{
  if (uart_tx_head == uart_tx_tail)
  {
    UCSR0B &= ~(1 << UDRIE0);
  }
  else
  {
    UDR0 = uart_tx_buffer[uart_tx_tail];
    uart_tx_tail = (uint8_t)((uart_tx_tail + 1) % UART_TX_BUFFER_SIZE);
  }
}
