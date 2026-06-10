#include "timer1.h"
#include <avr/io.h>          // Timer registers
#include <avr/interrupt.h>   // ISR, sei, cli

/* Compare value for exactly 5 ms with Timer1 @ 16 MHz and prescaler 64 */
#define TIMER1_COMPARE_VALUE  1249U

/* 
 * Software tick counter:
 * incremented every compare match interrupt
 */
static volatile uint32_t g_timer1Ticks = 0;

/* Optional callback function pointer */
static TIMER1_Callback_t g_timer1Callback = 0;

/*
 * Initialize Timer1 in CTC mode
 */
TIMER1_Status_t TIMER1_Init(void)
{
    /* Stop timer first */
    TCCR1B = 0x00;

    /* Clear control registers */
    TCCR1A = 0x00;
    TCCR1B = 0x00;

    /* Reset current timer count */
    TCNT1 = 0;

    /*
     * Set CTC mode for Timer1
     * From your notes:
     * WGM13 = 0
     * WGM12 = 1
     * WGM11 = 0
     * WGM10 = 0
     */
    TCCR1B |= (1 << WGM12);

    /* Set compare value for 5 ms */
    OCR1A = TIMER1_COMPARE_VALUE;

    /* Clear pending compare flag if any */
    TIFR1 |= (1 << OCF1A);

    /* Enable Timer1 Compare Match A interrupt */
    TIMSK1 |= (1 << OCIE1A);

    return TIMER1_OK;
}

/*
 * Start Timer1 using prescaler 64
 */
TIMER1_Status_t TIMER1_Start(void)
{
    /* Reset current counter */
    TCNT1 = 0;

    /*
     * Timer1 prescaler = 64
     * CS12 = 0, CS11 = 1, CS10 = 1
     *
     * From Timer0/Timer1 prescaler table:
     * 011 -> prescaler 64
     */
    TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
    TCCR1B |=  ((1 << CS11) | (1 << CS10));

    /* Enable global interrupts */
    sei();

    return TIMER1_OK;
}

/*
 * Stop Timer1 by removing its clock source
 */
TIMER1_Status_t TIMER1_Stop(void)
{
    /*
     * CS12:0 = 000
     * No clock source -> timer stops
     */
    TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));

    return TIMER1_OK;
}

/*
 * Safely return the current software tick count
 */
uint32_t TIMER1_GetTicks(void)
{
    uint32_t ticksCopy;
    uint8_t sreg = SREG;   /* Save CPU status register */

    cli();                 /* Disable interrupts temporarily */
    ticksCopy = g_timer1Ticks;
    SREG = sreg;           /* Restore old interrupt state */

    return ticksCopy;
}

/*
 * Safely reset software tick counter
 */
void TIMER1_ResetTicks(void)
{
    uint8_t sreg = SREG;   /* Save CPU status register */

    cli();                 /* Disable interrupts temporarily */
    g_timer1Ticks = 0;
    SREG = sreg;           /* Restore old interrupt state */
}

/*
 * Store callback function pointer
 */
TIMER1_Status_t TIMER1_SetCallback(TIMER1_Callback_t callback)
{
    /* Null pointer check */
    if (callback == 0)
    {
        return TIMER1_NULL_POINTER;
    }

    g_timer1Callback = callback;
    return TIMER1_OK;
}

/*
 * Timer1 Compare Match A ISR
 * This runs automatically every 5 ms
 */
ISR(TIMER1_COMPA_vect)
{
    /* Increment software tick count */
    g_timer1Ticks++;

    /* Call user callback if registered */
    if (g_timer1Callback != 0)
    {
        g_timer1Callback();
    }
}