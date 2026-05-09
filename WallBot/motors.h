/*
 * motors.h — register-level TB6612FNG dual-motor driver.
 *
 * Speed control: Timer2 Phase-Correct PWM, TOP=255, prescaler 1
 *                -> ~31.37 kHz (above audible whine band).
 *                OC2B = D3  -> PWMA -> LEFT motor
 *                OC2A = D11 -> PWMB -> RIGHT motor
 *
 * Direction logic (per channel, IN1 / IN2):
 *   1,0 -> forward       0,1 -> reverse
 *   1,1 -> short brake   0,0 -> coast (free-wheel)
 *
 * Speeds are signed [-255 .. +255]; sign chooses direction, magnitude
 * is the duty cycle. 0 issues a short brake. Use motors_coast() to let
 * the wheels free-wheel instead.
 */
#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void motors_init(void);             /* pins, Timer2, STBY held LOW */
void motors_enable(uint8_t on);     /* 1 -> STBY HIGH (driver active) */
void motors_set(int16_t left, int16_t right);
void motors_stop(void);             /* short brake both */
void motors_coast(void);            /* both channels free-wheel */

#ifdef __cplusplus
}
#endif

#endif /* MOTORS_H */
