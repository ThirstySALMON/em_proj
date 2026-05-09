#include "motors.h"
#include "pinmap.h"
#include <avr/io.h>

static inline uint8_t mag8(int16_t v) {
    if (v >  255) return 255;
    if (v < -255) return 255;
    return (uint8_t)(v < 0 ? -v : v);
}

void motors_init(void) {
    /* Direction + PWM pins as outputs, low. */
    DDRD |= (1 << MOT_AIN1_BIT) | (1 << MOT_AIN2_BIT) | (1 << MOT_PWMA_BIT);
    DDRB |= (1 << MOT_BIN1_BIT) | (1 << MOT_BIN2_BIT) | (1 << MOT_PWMB_BIT);
    PORTD &= ~((1 << MOT_AIN1_BIT) | (1 << MOT_AIN2_BIT) | (1 << MOT_PWMA_BIT));
    PORTB &= ~((1 << MOT_BIN1_BIT) | (1 << MOT_BIN2_BIT) | (1 << MOT_PWMB_BIT));

    /* STBY: start LOW so the driver is in standby until motors_enable(1). */
    MOT_STBY_DDR  |=  (1 << MOT_STBY_BIT);
    MOT_STBY_PORT &= ~(1 << MOT_STBY_BIT);

    /* Timer2: Phase-Correct PWM, TOP=0xFF (WGM2:0 = 001).
     * Non-inverting on both OC2A and OC2B.
     * Prescaler 1 (CS = 001) -> f_pwm = 16 MHz / (1 * 510) ≈ 31.37 kHz. */
    TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM20);
    TCCR2B = (1 << CS20);
    OCR2A = 0;
    OCR2B = 0;
}

void motors_enable(uint8_t on) {
    if (on) MOT_STBY_PORT |=  (1 << MOT_STBY_BIT);
    else    MOT_STBY_PORT &= ~(1 << MOT_STBY_BIT);
}

/* Direction helpers: set IN1/IN2 per channel. */
static void left_dir(int16_t s) {
    if (s > 0) {        /* forward: 1,0 */
        PORTD |=  (1 << MOT_AIN1_BIT);
        PORTD &= ~(1 << MOT_AIN2_BIT);
    } else if (s < 0) { /* reverse: 0,1 */
        PORTD &= ~(1 << MOT_AIN1_BIT);
        PORTD |=  (1 << MOT_AIN2_BIT);
    } else {            /* short brake: 1,1 */
        PORTD |=  (1 << MOT_AIN1_BIT) | (1 << MOT_AIN2_BIT);
    }
}

static void right_dir(int16_t s) {
    if (s > 0) {
        PORTB |=  (1 << MOT_BIN1_BIT);
        PORTB &= ~(1 << MOT_BIN2_BIT);
    } else if (s < 0) {
        PORTB &= ~(1 << MOT_BIN1_BIT);
        PORTB |=  (1 << MOT_BIN2_BIT);
    } else {
        PORTB |=  (1 << MOT_BIN1_BIT) | (1 << MOT_BIN2_BIT);
    }
}

void motors_set(int16_t left, int16_t right) {
    left_dir(left);
    right_dir(right);
    /* PWMA -> OC2B -> D3 -> LEFT.   PWMB -> OC2A -> D11 -> RIGHT. */
    OCR2B = mag8(left);
    OCR2A = mag8(right);
}

void motors_stop(void) { motors_set(0, 0); }

void motors_coast(void) {
    PORTD &= ~((1 << MOT_AIN1_BIT) | (1 << MOT_AIN2_BIT));
    PORTB &= ~((1 << MOT_BIN1_BIT) | (1 << MOT_BIN2_BIT));
    OCR2A = 0;
    OCR2B = 0;
}
