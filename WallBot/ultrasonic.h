/*
 * ultrasonic.h — register-level HC-SR04 driver for 3 sensors.
 *
 * Strategy:
 *   - Timer1 runs free at 0.5 us/tick (prescaler 8). It is the timebase
 *     for both echo-width measurement and the round-robin trigger
 *     scheduler (one trigger per Timer1 overflow ~33 ms).
 *   - Each ECHO line lives on a different PCINT group, so each ISR
 *     handles exactly one sensor: rising edge captures TCNT1 as start,
 *     falling edge captures TCNT1 as end and marks the reading ready.
 *   - Distances are stored in millimetres. 0xFFFF means "no reading"
 *     (out of range or echo timed out).
 */
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    US_FRONT = 0,
    US_RIGHT = 1,
    US_LEFT  = 2,
    US_COUNT = 3
} us_sensor_t;

#define US_NO_READING 0xFFFFu

/* One-time init: pins, Timer1, PCINTs. Caller must sei() afterwards. */
void us_init(void);

/* Run from main loop as often as possible. Handles trigger scheduling
 * and echo timeouts. Non-blocking. */
void us_service(void);

/* Latest distance in millimetres for a sensor, or US_NO_READING.
 * Atomic read of a 16-bit volatile. */
uint16_t us_distance_mm(us_sensor_t s);

/* Count of Timer1 overflows since us_init(). One overflow ≈ 32.768 ms.
 * Useful as a coarse non-blocking timebase in main(). Wraps at 65535. */
uint16_t us_overflow_count(void);

#ifdef __cplusplus
}
#endif

#endif /* ULTRASONIC_H */
