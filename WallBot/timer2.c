#include "timer2.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define TIMER2_COMPARE_VALUE   77U

/* software tick counter, increases every compare match interrupt */
static volatile uint32_t g_timerTicks = 0;

/* optional user callback function called from ISR */
static TIMER2_Callback_t g_timerCallback = 0;

// timer 2 , ctc mode, 5ms tick, interrupt on compare match, callback function to toggle an LED
// fixed to 16 MHz
// fixed to Timer2
// fixed to CTC
// fixed to 5 ms
// prescaler 1024, OCR2A = 77
// fixed to interrupt
TIMER2_Status_t TIMER2_Init(void)
{

    TCCR2B = 0x00; // stop timer

    // Clear control registers
    TCCR2A = 0x00;
    TCCR2B = 0x00;

    TCNT2 = 0; // reset counter

    /*
     * CTC mode for Timer2:
     * WGM22 = 0
     * WGM21 = 1
     * WGM20 = 0
     */
    TCCR2A |= (1 << WGM21);

    // Compare value for ~5 ms tick 
    OCR2A = TIMER2_COMPARE_VALUE;

    // Clear pending compare flag 
    TIFR2 |= (1 << OCF2A);

    // Enable Compare Match A interrupt 
    TIMSK2 |= (1 << OCIE2A);

    return TIMER2_OK;
}

TIMER2_Status_t TIMER2_Start(void)
{
    // Reset counter before starting 
    TCNT2 = 0;

    /*
     * Timer2 prescaler = 1024
     * CS22 = 1, CS21 = 1, CS20 = 1
     */
    TCCR2B &= ~((1 << CS22) | (1 << CS21) | (1 << CS20));
    TCCR2B |=  ((1 << CS22) | (1 << CS21) | (1 << CS20)); 

    // Enable global interrupts 
    sei();

    return TIMER2_OK;
}

TIMER2_Status_t TIMER2_Stop(void)
{
    /* No clock source -> timer stopped */
    TCCR2B &= ~((1 << CS22) | (1 << CS21) | (1 << CS20));

    return TIMER2_OK;
}

uint32_t TIMER2_GetTicks(void)
{
    uint32_t ticksCopy;
    uint8_t sreg = SREG; // save current interrupt state

    cli();               // disable interrupts temporarily
    ticksCopy = g_timerTicks;
    SREG = sreg;         // restore old interrupt state

    return ticksCopy;
}

void TIMER2_ResetTicks(void)
{
    uint8_t sreg = SREG; // save current interrupt state

    cli();               // disable interrupts temporarily
    g_timerTicks = 0;
    SREG = sreg;         // restore old interrupt state
}

TIMER2_Status_t TIMER2_SetCallback(TIMER2_Callback_t callback)
{
    /* make sure callback pointer is valid */
    if (callback == 0)
    {
        return TIMER2_NULL_POINTER;
    }

    g_timerCallback = callback;
    return TIMER2_OK;
}

/* ISR runs automatically every compare match */
ISR(TIMER2_COMPA_vect)
{
    g_timerTicks++; // count one more 5 ms tick

    if (g_timerCallback != 0)
    {
        g_timerCallback();
    }
}