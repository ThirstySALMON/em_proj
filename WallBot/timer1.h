#ifndef TIMER1_H_
#define TIMER1_H_

#include <stdint.h> 

/* One timer1 tick is 5 ms */
#define TIMER1_TICK_MS 5UL

/* Status returned by the driver functions */
typedef enum
{
    TIMER1_OK = 0,
    TIMER1_NULL_POINTER
} TIMER1_Status_t;

/* Function pointer type for optional callback */
typedef void (*TIMER1_Callback_t)(void);

/*
 * Timer1 fixed configuration:
 * - Timer1
 * - CTC mode
 * - Compare Match A interrupt
 * - 5 ms periodic tick @ 16 MHz
 * - Prescaler = 64
 * - OCR1A = 1249
 */
TIMER1_Status_t TIMER1_Init(void);
TIMER1_Status_t TIMER1_Start(void);
TIMER1_Status_t TIMER1_Stop(void);
uint32_t        TIMER1_GetTicks(void);
void            TIMER1_ResetTicks(void);
TIMER1_Status_t TIMER1_SetCallback(TIMER1_Callback_t callback);

#endif