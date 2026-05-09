#include "ultrasonic.h"
#include "pinmap.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include <util/delay.h>

/* Timer1: prescaler 8 -> 0.5 us/tick, overflow every 32768 us. */
#define US_TICKS_PER_OVF      0xFFFFu
#define US_ECHO_TIMEOUT_TICKS 50000u   /* ~25 ms ≈ 4.3 m round-trip */

/* HC-SR04 datasheet: distance_cm = pulse_us / 58.
 * With 0.5 us/tick: distance_mm = ticks * 5 / 58. Use uint32 multiply
 * to avoid 16-bit overflow (max ~25000 ticks * 5 = 125000 fits in u32). */
static inline uint16_t ticks_to_mm(uint16_t ticks) {
    return (uint16_t)(((uint32_t)ticks * 5UL) / 58UL);
}

/* Per-sensor state. Index by us_sensor_t. */
static volatile uint16_t echo_start[US_COUNT];
static volatile uint16_t distance_mm[US_COUNT];
static volatile uint8_t  pending[US_COUNT];     /* 1 if waiting for falling edge */
static volatile uint16_t pending_start_t1[US_COUNT]; /* TCNT1 at trigger time */

/* Round-robin scheduler driven by Timer1 overflow. */
static volatile uint8_t  next_sensor;
static volatile uint8_t  ovf_tick;              /* set by ISR, cleared by service */
static volatile uint16_t ovf_count;             /* monotonic, public via getter */

/* ---- pin helpers ------------------------------------------------------- */

static inline void trig_pulse_front(void) {
    TRIG_FRONT_PORT |=  (1 << TRIG_FRONT_BIT);
    _delay_us(10);
    TRIG_FRONT_PORT &= ~(1 << TRIG_FRONT_BIT);
}
static inline void trig_pulse_right(void) {
    TRIG_RIGHT_PORT |=  (1 << TRIG_RIGHT_BIT);
    _delay_us(10);
    TRIG_RIGHT_PORT &= ~(1 << TRIG_RIGHT_BIT);
}
static inline void trig_pulse_left(void) {
    TRIG_LEFT_PORT  |=  (1 << TRIG_LEFT_BIT);
    _delay_us(10);
    TRIG_LEFT_PORT  &= ~(1 << TRIG_LEFT_BIT);
}

static void fire_sensor(uint8_t s) {
    pending[s] = 1;
    pending_start_t1[s] = TCNT1;
    switch (s) {
        case US_FRONT: trig_pulse_front(); break;
        case US_RIGHT: trig_pulse_right(); break;
        case US_LEFT:  trig_pulse_left();  break;
    }
}

/* ---- public API -------------------------------------------------------- */

void us_init(void) {
    /* TRIG pins as outputs, low. */
    TRIG_FRONT_DDR |= (1 << TRIG_FRONT_BIT);
    TRIG_RIGHT_DDR |= (1 << TRIG_RIGHT_BIT);
    TRIG_LEFT_DDR  |= (1 << TRIG_LEFT_BIT);
    TRIG_FRONT_PORT &= ~(1 << TRIG_FRONT_BIT);
    TRIG_RIGHT_PORT &= ~(1 << TRIG_RIGHT_BIT);
    TRIG_LEFT_PORT  &= ~(1 << TRIG_LEFT_BIT);

    /* ECHO pins as inputs, no pull-up (sensor drives push-pull). */
    ECHO_FRONT_DDR &= ~(1 << ECHO_FRONT_BIT);
    ECHO_RIGHT_DDR &= ~(1 << ECHO_RIGHT_BIT);
    ECHO_LEFT_DDR  &= ~(1 << ECHO_LEFT_BIT);

    /* Timer1: normal mode, prescaler 8 (0.5 us/tick at 16 MHz). */
    TCCR1A = 0;
    TCCR1B = (1 << CS11);
    TCCR1C = 0;
    TCNT1  = 0;
    TIMSK1 = (1 << TOIE1);          /* overflow used as scheduler tick */

    /* Pin-change interrupts: one echo per group. */
    PCMSK0 = (1 << PCINT1);         /* PB1 — right echo  */
    PCMSK1 = (1 << PCINT11);        /* PC3 — left echo   */
    PCMSK2 = (1 << PCINT22);        /* PD6 — front echo  */
    PCICR  = (1 << PCIE0) | (1 << PCIE1) | (1 << PCIE2);

    for (uint8_t i = 0; i < US_COUNT; i++) {
        distance_mm[i] = US_NO_READING;
        pending[i] = 0;
    }
    next_sensor = 0;
    ovf_tick    = 0;
    ovf_count   = 0;
}

uint16_t us_overflow_count(void) {
    uint16_t v;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { v = ovf_count; }
    return v;
}

void us_service(void) {
    /* Time out any echo we've been waiting on too long, so a missing
     * sensor or an out-of-range target doesn't strand pending[]. */
    uint16_t now = TCNT1;
    for (uint8_t i = 0; i < US_COUNT; i++) {
        if (pending[i]) {
            uint16_t elapsed = now - pending_start_t1[i];   /* wraps OK */
            if (elapsed > US_ECHO_TIMEOUT_TICKS) {
                pending[i] = 0;
                distance_mm[i] = US_NO_READING;
            }
        }
    }

    /* On each Timer1 overflow (~33 ms), trigger the next sensor in the
     * rotation. Full cycle ≈ 100 ms → ~10 Hz per sensor. */
    if (ovf_tick) {
        ovf_tick = 0;
        uint8_t s = next_sensor;
        if (!pending[s]) fire_sensor(s);   /* skip if previous still pending */
        next_sensor = (s + 1) % US_COUNT;
    }
}

uint16_t us_distance_mm(us_sensor_t s) {
    uint16_t v;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { v = distance_mm[s]; }
    return v;
}

/* ---- ISRs -------------------------------------------------------------- */

ISR(TIMER1_OVF_vect) {
    ovf_tick = 1;
    ovf_count++;
}

/* Each PCINT ISR captures TCNT1 *first* to keep the timestamp tight,
 * then decides edge by re-reading the pin. */

ISR(PCINT0_vect) {                  /* PORTB — right echo on PB1 */
    uint16_t t = TCNT1;
    if (ECHO_RIGHT_PIN & (1 << ECHO_RIGHT_BIT)) {
        echo_start[US_RIGHT] = t;
    } else if (pending[US_RIGHT]) {
        distance_mm[US_RIGHT] = ticks_to_mm((uint16_t)(t - echo_start[US_RIGHT]));
        pending[US_RIGHT] = 0;
    }
}

ISR(PCINT1_vect) {                  /* PORTC — left echo on PC3 */
    uint16_t t = TCNT1;
    if (ECHO_LEFT_PIN & (1 << ECHO_LEFT_BIT)) {
        echo_start[US_LEFT] = t;
    } else if (pending[US_LEFT]) {
        distance_mm[US_LEFT] = ticks_to_mm((uint16_t)(t - echo_start[US_LEFT]));
        pending[US_LEFT] = 0;
    }
}

ISR(PCINT2_vect) {                  /* PORTD — front echo on PD6 */
    uint16_t t = TCNT1;
    if (ECHO_FRONT_PIN & (1 << ECHO_FRONT_BIT)) {
        echo_start[US_FRONT] = t;
    } else if (pending[US_FRONT]) {
        distance_mm[US_FRONT] = ticks_to_mm((uint16_t)(t - echo_start[US_FRONT]));
        pending[US_FRONT] = 0;
    }
}
