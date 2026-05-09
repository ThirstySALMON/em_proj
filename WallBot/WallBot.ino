/*
 * WallBot — Step 2 bring-up: HC-SR04 sensor test.
 *
 * Strict register-level. No Arduino library calls (digitalWrite, delay,
 * Serial.print, etc.). The .ino is here only because the Arduino IDE
 * needs an entry point; setup() and loop() are used as plain functions.
 *
 * Test procedure:
 *   1. HC-05 unplugged from D0/D1.
 *   2. Upload via Arduino IDE (Tools → Board: Arduino Nano,
 *      Processor: ATmega328P (Old Bootloader) if needed).
 *   3. Open Serial Monitor at 9600 baud, "No line ending".
 *   4. Output every ~250 ms:
 *        F=<mm> R=<mm> L=<mm>
 *      "----" means out of range / no echo within timeout.
 *   5. Wave a hand in front of each sensor, confirm only that sensor's
 *      reading changes. Front near 100 mm should print ~100.
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "pinmap.h"
#include "uart.h"
#include "ultrasonic.h"

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

static void print_reading(const char *label, uint16_t mm) {
    uart_puts(label);
    if (mm == US_NO_READING) uart_puts("----");
    else                     uart_put_u16(mm);
    uart_putc(' ');
}

void setup(void) {
    /* Status LED. */
    LED_DDR |= (1 << LED_BIT);
    LED_PORT &= ~(1 << LED_BIT);

    uart_init(9600);
    us_init();
    sei();

    uart_puts("\r\nWallBot bring-up: HC-SR04 x3\r\n");
}

void loop(void) {
    static uint16_t last_print_ovf = 0;

    us_service();

    /* Print every 8 Timer1 overflows ≈ 262 ms. Unsigned subtraction
     * means the comparison still works after ovf_count wraps at 65535. */
    uint16_t ovf = us_overflow_count();
    if ((uint16_t)(ovf - last_print_ovf) >= 8) {
        last_print_ovf = ovf;
        print_reading("F=", us_distance_mm(US_FRONT));
        print_reading("R=", us_distance_mm(US_RIGHT));
        print_reading("L=", us_distance_mm(US_LEFT));
        uart_puts("\r\n");
        LED_PORT ^= (1 << LED_BIT);   /* heartbeat */
    }
}
