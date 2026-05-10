#include <avr/io.h>
#include <util/atomic.h>
#include "uart.h"


#define UART_BUFFER_SIZE 64

volatile char uart_buffer[UART_BUFFER_SIZE];
volatile uint8_t uart_head = 0;
volatile uint8_t uart_tail = 0;
volatile uint8_t uart_overflow = 0;

void UART_init(unsigned long baud)
{
  unsigned int ubrr = F_CPU / 16 / baud - 1;

  // Set baud rate
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;

  // Enable transmitter and receiver
  UCSR0B = (1 << RXCIE0) | (1 << TXEN0) | (1 << RXEN0);

  // Set frame format: 8 data bits, 1 stop bit
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

  sei(); // enable global interrupts
}

void UART_sendChar(char c)
{
  // Wait until buffer is empty
  while (!(UCSR0A & (1 << UDRE0)))
    ;

  // Put data into buffer
  UDR0 = c;
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
  uint8_t head;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    head = uart_head;
  }
  if (head == uart_tail)
    return 0;

  *data = uart_buffer[uart_tail];
  uart_tail = (uart_tail + 1) % UART_BUFFER_SIZE;
  return 1;
}

//Note: this function blocks CPU
uint8_t UART_receiveString(char *buf, uint8_t maxLen)
{
  uint8_t i = 0;
  char c;

  while (i < maxLen - 1)
  {
    if (UART_receiveChar(&c))
    {
      if (c == '\r' || c == '\n')
      {
        if (i == 0)
          continue; // skip leading \r or \n
        break;      // end of string
      }
      buf[i++] = c;
    }
  }

  buf[i] = '\0'; // null terminate
  return i;      // return number of chars received
}

ISR(USART_RX_vect)
{
  char c = UDR0;
  uint8_t next = (uart_head + 1) % UART_BUFFER_SIZE;
  if (next != uart_tail)
  {
    uart_buffer[uart_head] = c;
    uart_head = next;
  }
  else
  {
    uart_overflow = 1;
  }
}