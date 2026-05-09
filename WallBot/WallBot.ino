/*
 * WallBot — Step 3 bring-up: TB6612FNG motor test (sensors still live).
 *
 * Strict register-level. No Arduino library calls (digitalWrite, delay,
 * Serial.print, etc.). The .ino is here only because the Arduino IDE
 * needs an entry point; setup() and loop() are used as plain functions.
 *
 * Test procedure:
 *   1. *** Lift the robot so wheels spin free, OR remove wheels. ***
 *   2. Power motors from the battery (VM). USB alone won't drive them.
 *   3. HC-05 still unplugged from D0/D1; debug over USB-serial.
 *   4. Upload (Arduino Nano, ATmega328P / Old Bootloader if needed).
 *   5. Open Serial Monitor at 9600 baud.
 *   6. The sketch runs an automatic 7-step demo on a ~2 s cadence:
 *
 *        STEP   ACTION                EXPECTED
 *        ----   --------------------  ------------------------------
 *        0      idle (STBY low)       wheels free, no current
 *        1      forward 50 % both     both wheels turn FORWARD
 *        2      brake                 wheels stop hard
 *        3      reverse 50 % both     both wheels turn BACKWARD
 *        4      brake                 wheels stop hard
 *        5      spin LEFT (in place)  L back, R fwd  (CCW from above)
 *        6      spin RIGHT (in place) L fwd, R back  (CW  from above)
 *        7      coast                 wheels free-wheel; stays here
 *
 *      Each line printed is:
 *        [step] F=<mm> R=<mm> L=<mm>
 *
 *   7. If a wheel spins the wrong way for its step, swap that motor's
 *      two leads at the TB6612 output (AO1<->AO2 or BO1<->BO2). Do NOT
 *      change pin macros — the convention is "positive speed = forward".
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

#define DEMO_SPEED      128             /* ~50 % duty */
#define STEP_OVFS       60              /* 60 * 32.768 ms ≈ 1.97 s per step */

static void print_reading(const char *label, uint16_t mm) {
    uart_puts(label);
    if (mm == US_NO_READING) uart_puts("----");
    else                     uart_put_u16(mm);
    uart_putc(' ');
}

static void apply_step(uint8_t step) {
    switch (step) {
        case 0: motors_enable(0); motors_coast();                   break;
        case 1: motors_enable(1); motors_set( DEMO_SPEED,  DEMO_SPEED); break;
        case 2: motors_enable(1); motors_stop();                    break;
        case 3: motors_enable(1); motors_set(-DEMO_SPEED, -DEMO_SPEED); break;
        case 4: motors_enable(1); motors_stop();                    break;
        case 5: motors_enable(1); motors_set(-DEMO_SPEED,  DEMO_SPEED); break;
        case 6: motors_enable(1); motors_set( DEMO_SPEED, -DEMO_SPEED); break;
        default:motors_enable(0); motors_coast();                   break; /* 7+ */
    }
}

void setup(void) {
    LED_DDR |= (1 << LED_BIT);
    LED_PORT &= ~(1 << LED_BIT);

    uart_init(9600);
    us_init();
    motors_init();                  /* STBY held LOW until step 1 */
    sei();

    uart_puts("\r\nWallBot bring-up: motor demo (step 0..7)\r\n");
    apply_step(0);
}

void loop(void) {
    static uint16_t last_print_ovf = 0;
    static uint16_t step_start_ovf = 0;
    static uint8_t  step = 0;

    us_service();

    uint16_t ovf = us_overflow_count();

    /* Advance demo step every ~2 s, then latch on the final step. */
    if (step <= 6 && (uint16_t)(ovf - step_start_ovf) >= STEP_OVFS) {
        step_start_ovf = ovf;
        step++;
        apply_step(step);
    }

    /* Print sensor + step state every ~262 ms. */
    if ((uint16_t)(ovf - last_print_ovf) >= 8) {
        last_print_ovf = ovf;
        uart_putc('[');
        uart_put_u16(step);
        uart_puts("] ");
        print_reading("F=", us_distance_mm(US_FRONT));
        print_reading("R=", us_distance_mm(US_RIGHT));
        print_reading("L=", us_distance_mm(US_LEFT));
        uart_puts("\r\n");
        LED_PORT ^= (1 << LED_BIT);
    }
}
