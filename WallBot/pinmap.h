/*
 * pinmap.h — single source of truth for pin assignments.
 * Must match docs/PINOUT.md. Edit both together.
 */
#ifndef PINMAP_H
#define PINMAP_H

#include <avr/io.h>

/* ---- TB6612FNG motor driver ---- */
#define MOT_STBY_PORT  PORTD
#define MOT_STBY_DDR   DDRD
#define MOT_STBY_BIT   PD2

#define MOT_PWMA_BIT   PD3   /* OC2B */
#define MOT_AIN1_BIT   PD4
#define MOT_AIN2_BIT   PD7

#define MOT_BIN2_BIT   PB0
#define MOT_BIN1_BIT   PB4
#define MOT_PWMB_BIT   PB3   /* OC2A */

/* ---- HC-SR04 trig (outputs) ---- */
#define TRIG_FRONT_PORT PORTD
#define TRIG_FRONT_DDR  DDRD
#define TRIG_FRONT_BIT  PD5

#define TRIG_RIGHT_PORT PORTB
#define TRIG_RIGHT_DDR  DDRB
#define TRIG_RIGHT_BIT  PB2

#define TRIG_LEFT_PORT  PORTC
#define TRIG_LEFT_DDR   DDRC
#define TRIG_LEFT_BIT   PC0

/* ---- HC-SR04 echo (inputs, each on its own PCINT group) ---- */
#define ECHO_FRONT_PIN  PIND
#define ECHO_FRONT_DDR  DDRD
#define ECHO_FRONT_BIT  PD6   /* PCINT22 — PCI2 */

#define ECHO_RIGHT_PIN  PINB
#define ECHO_RIGHT_DDR  DDRB
#define ECHO_RIGHT_BIT  PB1   /* PCINT1  — PCI0 */

#define ECHO_LEFT_PIN   PINC
#define ECHO_LEFT_DDR   DDRC
#define ECHO_LEFT_BIT   PC3   /* PCINT11 — PCI1 */

/* ---- Status LED (onboard) ---- */
#define LED_PORT  PORTB
#define LED_DDR   DDRB
#define LED_BIT   PB5

#endif /* PINMAP_H */
