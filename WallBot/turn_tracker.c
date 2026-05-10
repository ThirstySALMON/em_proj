#include "turn_tracker.h"
#include "uart.h"

#define MAX_TURNS 200

static char sequence[MAX_TURNS * 3 + 1];
static uint8_t turn_count = 0;
static uint8_t current_index = 0;

// private helper (not exposed)
static void addTurn(char direction)
{
    if (turn_count >= MAX_TURNS)
        return;

    if (turn_count > 0)
    {
        sequence[current_index++] = ',';
        sequence[current_index++] = ' ';
    }

    sequence[current_index++] = direction;
    sequence[current_index] = '\0';

    turn_count++;
}

void turns_init(void)
{
    turn_count = 0;
    current_index = 0;
    sequence[0] = '\0';
}

void turn_left(void)
{
    addTurn('L');
}

void turn_right(void)
{
    addTurn('R');
}

void sendReport(void)
{
    UART_sendString("Turns: ");
    UART_sendInt(turn_count);
    UART_sendString("\nSequence: ");
    UART_sendString(sequence);
    UART_sendChar('\n');
}