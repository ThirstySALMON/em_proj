/*
 * WallBot — Step 4: PID corridor centering.
 *
 * Strict register-level. No Arduino library calls.
 *
 *   error = right_mm - left_mm     (positive => closer to LEFT wall)
 *   turn  = Kp*error + Kd*(error-prev) + Ki*sum
 *   left  = base + turn            right = base - turn
 *
 * Tunables are at the top of this file. Speeds are expressed as 0..100 %
 * of the 8-bit PWM full scale (255), so MAX_SPEED_PCT actually limits the
 * current the motors can pull.
 *
 * Test procedure:
 *   1. Lift wheels OR place robot in a 45 cm corridor with both side
 *      walls present and the front clear.
 *   2. HC-05 still unplugged; debug over USB-serial @ 9600.
 *   3. Watch the printed line:
 *        L=<mm>  R=<mm>  F=<mm>  err=<mm>  outL=<pwm>  outR=<pwm>
 *      "----" = sensor saw nothing (out of range or timeout).
 *      Both side readings must be valid for PID to steer.
 *   4. With Kp only, expect oscillation. Add Kd to damp. Leave Ki=0
 *      for now — it tends to wind up on this kind of geometry.
 *   5. Front-stop kicks in if anything is closer than FRONT_STOP_MM.
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "pinmap.h"
#include "uart.h"
#include "ultrasonic.h"
#include "motors.h"

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

/* ---- TUNABLES --------------------------------------------------------- */

/* Speed caps as % of 8-bit PWM full scale (0..100). */
#define MAX_SPEED_PCT     100
#define BASE_SPEED_PCT    75      /* forward speed when centred */

/* Per-motor trim, signed % of full PWM. Compensates for the fact that
 * two "identical0" DC motors never spin at the same speed for the same
 * PWM. Positive value boosts LEFT and cuts RIGHT (i.e. steers RIGHT,
 * fights drift toward the left wall). Tune in steps of 2..3. */
#define MOTOR_TRIM_PCT     6      /* try 5 first if drifting left */

/* PID gains stored as gain * 100 (fixed-point, no floats on AVR). */
#define KP_X100           18      /* Kp = 0.30 */
#define KD_X100           68      /* Kd = 0.50 */
#define KI_X100            0      /* start with PD only */

#define INTEG_LIMIT     4000      /* anti-windup clamp */

/* Front-sensor emergency stop. Robot brakes if anything is closer. */
#define FRONT_STOP_MM    120

/* PID update cadence: every N Timer1 overflows. 1 ovf ≈ 32.768 ms. */
#define PID_PERIOD_OVFS    1      /* ~30 Hz */
#define PRINT_PERIOD_OVFS  8      /* ~262 ms */

/* ---- DERIVED ---------------------------------------------------------- */

#define PCT_TO_PWM(p)     ((int16_t)(((int32_t)(p) * 255) / 100))
#define MAX_SPEED         PCT_TO_PWM(MAX_SPEED_PCT)
#define BASE_SPEED        PCT_TO_PWM(BASE_SPEED_PCT)

/* ---- STATE ------------------------------------------------------------ */

static int16_t prev_error = 0;
static int32_t integ      = 0;
static int16_t last_outL  = 0;
static int16_t last_outR  = 0;
static int16_t last_error = 0;
static uint8_t front_braking = 0;

/* ---- HELPERS ---------------------------------------------------------- */

static inline int16_t clamp16(int32_t v, int16_t lim) {
    if (v >  lim) return  lim;
    if (v < -lim) return -lim;
    return (int16_t)v;
}

static void print_field(const char *label, int16_t v, uint8_t is_signed) {
    uart_puts(label);
    if (is_signed) uart_put_i16(v);
    else           uart_put_u16((uint16_t)v);
    uart_putc(' ');
}

static void print_dist(const char *label, uint16_t mm) {
    uart_puts(label);
    if (mm == US_NO_READING) uart_puts("----");
    else                     uart_put_u16(mm);
    uart_putc(' ');
}

/* ---- PID -------------------------------------------------------------- */

static void pid_step(void) {
    uint16_t L = us_distance_mm(US_LEFT);
    uint16_t R = us_distance_mm(US_RIGHT);
    uint16_t F = us_distance_mm(US_FRONT);

    /* 1) Front-sensor safety: hard brake on obstacle. */
    if (F != US_NO_READING && F < FRONT_STOP_MM) {
        motors_stop();
        last_outL = last_outR = 0;
        front_braking = 1;
        return;
    }
    front_braking = 0;

    /* 2) Need both side walls to do PID. If either is missing, crawl
     *    forward in a straight line at half base speed and hold the
     *    integrator (don't accumulate error against unknown geometry). */
    if (L == US_NO_READING || R == US_NO_READING) {
        int16_t crawl = BASE_SPEED / 2;
        motors_set(crawl, crawl);
        last_outL = last_outR = crawl;
        last_error = 0;
        return;
    }

    /* 3) PID on lateral error. */
    int16_t error = (int16_t)R - (int16_t)L;
    int16_t deriv = error - prev_error;
    integ += error;
    if (integ >  INTEG_LIMIT) integ =  INTEG_LIMIT;
    if (integ < -INTEG_LIMIT) integ = -INTEG_LIMIT;

    int32_t turn = ((int32_t)KP_X100 * error
                  + (int32_t)KD_X100 * deriv
                  + (int32_t)KI_X100 * integ) / 100;

    /* Cap differential so PID alone can't reverse a wheel. */
    int16_t turn_pwm = clamp16(turn, BASE_SPEED);

    int16_t left  = (int16_t)((int32_t)BASE_SPEED + turn_pwm);
    int16_t right = (int16_t)((int32_t)BASE_SPEED - turn_pwm);

    /* Static motor trim — applied AFTER PID so the controller and the
     * imbalance are decoupled. Positive trim => left faster, right slower. */
    int16_t trim_pwm = PCT_TO_PWM(MOTOR_TRIM_PCT);
    left  += trim_pwm;
    right -= trim_pwm;

    /* Floor each wheel at 0 — no spinning in place during centering.
     * If you want true 90° turns, that goes in Step 5 (turn execute). */
    if (left  < 0) left  = 0;
    if (right < 0) right = 0;
    if (left  > MAX_SPEED) left  = MAX_SPEED;
    if (right > MAX_SPEED) right = MAX_SPEED;

    motors_set(left, right);

    prev_error = error;
    last_outL  = left;
    last_outR  = right;
    last_error = error;
}

/* ---- ENTRY POINTS ----------------------------------------------------- */

void setup(void) {
    LED_DDR  |=  (1 << LED_BIT);
    LED_PORT &= ~(1 << LED_BIT);

    uart_init(9600);
    us_init();
    motors_init();
    sei();

    uart_puts("\r\nWallBot: PID corridor centering\r\n");
    uart_puts("MAX="); uart_put_u16(MAX_SPEED_PCT);
    uart_puts("%  BASE="); uart_put_u16(BASE_SPEED_PCT);
    uart_puts("%  TRIM="); uart_put_i16(MOTOR_TRIM_PCT);
    uart_puts("%  Kp*100="); uart_put_u16(KP_X100);
    uart_puts(" Kd*100=");   uart_put_u16(KD_X100);
    uart_puts(" Ki*100=");   uart_put_u16(KI_X100);
    uart_puts("\r\n");

    motors_enable(1);
}

void loop(void) {
    static uint16_t last_print_ovf = 0;
    static uint16_t last_pid_ovf   = 0;

    us_service();

    uint16_t ovf = us_overflow_count();

    if ((uint16_t)(ovf - last_pid_ovf) >= PID_PERIOD_OVFS) {
        last_pid_ovf = ovf;
        pid_step();
    }

    if ((uint16_t)(ovf - last_print_ovf) >= PRINT_PERIOD_OVFS) {
        last_print_ovf = ovf;
        print_dist ("L=",    us_distance_mm(US_LEFT));
        print_dist ("R=",    us_distance_mm(US_RIGHT));
        print_dist ("F=",    us_distance_mm(US_FRONT));
        print_field("err=",  last_error, 1);
        print_field("oL=",   last_outL,  1);
        print_field("oR=",   last_outR,  1);
        if (front_braking) uart_puts("[STOP]");
        uart_puts("\r\n");
        LED_PORT ^= (1 << LED_BIT);
    }
}
