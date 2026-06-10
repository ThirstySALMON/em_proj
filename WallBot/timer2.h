#include <stdint.h>
#ifndef TIMER2_H_
#define TIMER2_H_

#define TIMER2_TICK_MS 5UL

typedef enum
{
    TIMER2_OK = 0,
    TIMER2_NULL_POINTER
} TIMER2_Status_t;

typedef void (*TIMER2_Callback_t)(void);

/*
 * Fixed project timer:
 * - Timer2
 * - CTC mode
 * - Compare Match A interrupt
 * - ~5 ms periodic tick @ 16 MHz
 * - Prescaler = 1024
 * - OCR2A = 77
 * - User callback function called from ISR on each tick
 * fixed to interrupt
 */

TIMER2_Status_t TIMER2_Init(void);
TIMER2_Status_t TIMER2_Start(void);
TIMER2_Status_t TIMER2_Stop(void);
uint32_t       TIMER2_GetTicks(void);
void           TIMER2_ResetTicks(void);
TIMER2_Status_t TIMER2_SetCallback(TIMER2_Callback_t callback);

#endif