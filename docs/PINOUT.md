# Pinout & Wiring Reference — Wall-Following Robot

**Board:** Arduino Nano (ATmega328P, 16 MHz, 5 V logic)
**Last updated:** 2026-05-09
**Status:** canonical wiring source. All firmware pin macros must match this document.

---

## 1. Pin assignment table

| Pin | Port/Bit  | Dir | Connected to              | Role / notes                                         |
|-----|-----------|-----|---------------------------|------------------------------------------------------|
| D0  | PD0 / RXD | In  | HC-05 **TX**              | USART0 RX                                            |
| D1  | PD1 / TXD | Out | HC-05 **RX**              | USART0 TX — needs 5V→3.3V divider (1 kΩ + 2 kΩ)      |
| D2  | PD2       | Out | TB6612FNG **STBY**        | HIGH = driver enabled                                |
| D3  | PD3 / OC2B| Out | TB6612FNG **PWMA**        | Left motor speed (Timer2 Fast PWM)                   |
| D4  | PD4       | Out | TB6612FNG **AIN1**        | Left motor direction bit 1                           |
| D5  | PD5       | Out | HC-SR04 **Front TRIG**    | 10 µs pulse                                          |
| D6  | PD6       | In  | HC-SR04 **Front ECHO**    | PCINT22 — PCI2 group (PORTD)                         |
| D7  | PD7       | Out | TB6612FNG **AIN2**        | Left motor direction bit 2                           |
| D8  | PB0       | Out | TB6612FNG **BIN2**        | Right motor direction bit 2 (ICP1 left unused)       |
| D9  | PB1       | In  | HC-SR04 **Right ECHO**    | PCINT1 — PCI0 group (PORTB)                          |
| D10 | PB2       | Out | HC-SR04 **Right TRIG**    | 10 µs pulse                                          |
| D11 | PB3 / OC2A| Out | TB6612FNG **PWMB**        | Right motor speed (Timer2 Fast PWM)                  |
| D12 | PB4       | Out | TB6612FNG **BIN1**        | Right motor direction bit 1                          |
| D13 | PB5       | Out | Onboard LED               | Status / heartbeat                                   |
| A0  | PC0       | Out | HC-SR04 **Left TRIG**     | Used as digital out                                  |
| A1  | PC1       | —   | (free)                    | Spare                                                |
| A2  | PC2       | —   | (free)                    | Spare                                                |
| A3  | PC3       | In  | HC-SR04 **Left ECHO**     | PCINT11 — PCI1 group (PORTC)                         |
| A4  | PC4 / SDA | —   | Reserved                  | Future I²C                                           |
| A5  | PC5 / SCL | —   | Reserved                  | Future I²C                                           |
| A6  | ADC6      | —   | (free, analog-only)       | e.g. battery voltage divider                         |
| A7  | ADC7      | —   | (free, analog-only)       | Spare                                                |

---

## 2. Peripheral allocation

| Peripheral        | Owner                          | Configuration                                              |
|-------------------|--------------------------------|------------------------------------------------------------|
| Timer0 (8-bit)    | unused                         | Reserved for future system tick                            |
| Timer1 (16-bit)   | HC-SR04 echo width measurement | Free-running, prescaler 8 → 0.5 µs/tick (~32.7 ms range)   |
| Timer2 (8-bit)    | Motor PWM                      | Fast PWM mode, prescaler 1 → ~31.25 kHz, OC2A=D11, OC2B=D3 |
| PCI0 (PCINT0..7)  | Right echo on PB1              | PCMSK0 = (1<<PCINT1)                                       |
| PCI1 (PCINT8..14) | Left echo on PC3               | PCMSK1 = (1<<PCINT11)                                      |
| PCI2 (PCINT16..23)| Front echo on PD6              | PCMSK2 = (1<<PCINT22)                                      |
| USART0            | HC-05 Bluetooth                | 9600 8N1 (verify HC-05 module's stored baud first)         |

Each ECHO line lives on a different PCINT group so each ISR handles exactly one
sensor — no in-ISR pin disambiguation, cleaner round-trip timing.

---

## 3. Why these pins (rationale)

- **UART (D0/D1)** is the only hardware serial on the 328P, so HC-05 claims it.
- **PWMA=D3, PWMB=D11** are the two Timer2 output-compare pins, letting both
  motor PWMs share a single timer with identical frequency/phase. Timer1 is
  *not* used for motor PWM because it is reserved for echo timing, and Timer0
  is left untouched in case a millis-equivalent tick is added later.
- **Echo pins (D6, D9, A3)** were chosen one-per-PCINT-group. Each ISR can read
  the corresponding PIN register bit and stamp Timer1's TCNT1 without deciding
  which sensor fired.
- **TRIG pins (D5, D10, A0)** are pure GPIO outputs and can sit on any free pin;
  they were placed near their ECHO partner to keep the wire harness short.
- **D8 (ICP1)** is used as a plain digital output for BIN2; we deliberately do
  *not* use input capture, since echo timing is handled by PCINT + TCNT1 reads.
- **D13** keeps its onboard LED role — useful as a heartbeat/fault indicator.
- **A4/A5 (I²C)** stay free for a future IMU or expander.
- **A6/A7** are analog-only on the Nano; reserved for a battery voltage
  divider so we can detect brown-out before the motors start glitching.

---

## 4. Power & wiring notes

### Rails

```
Battery (e.g. 7.4 V Li-ion, 6 V NiMH)
   │
   ├──► TB6612FNG VM   (motor power, 2.7–13.5 V)
   │
   └──► 5 V BEC / regulator ──► Nano VIN  (or 5V pin if regulator output is clean 5 V)
                              ──► HC-SR04 ×3 VCC  (5 V)
                              ──► HC-05 VCC       (most modules accept 3.6–6 V)
                              ──► TB6612FNG VCC   (logic, 2.7–5.5 V)

GND: common across battery −, Nano GND, motor driver GND, HC-05 GND, all sensors GND.
```

- The Nano's onboard regulator can sustain ~400 mA; three HC-SR04s + an HC-05
  are borderline. Prefer a separate 5 V BEC for sensors when on battery.
- **Never** power motors from the Nano's 5 V rail. VM is its own line.
- Decoupling: 100 nF ceramic across each sensor VCC/GND, plus a bulk
  100–470 µF electrolytic close to TB6612 VM.
- HC-05 RX is **not 5 V tolerant** on bare modules. Add a divider on the
  D1 → HC-05 RX line (1 kΩ in series, 2 kΩ to GND gives ~3.3 V). Most
  breakout boards already include this — confirm by inspecting the module.

### Signal-level summary

| Signal            | Voltage  | Direction          | Notes                            |
|-------------------|----------|--------------------|----------------------------------|
| HC-SR04 ECHO → D6/D9/A3 | 5 V | sensor → MCU        | Direct connect, MCU is 5 V       |
| HC-SR04 TRIG ← MCU | 5 V    | MCU → sensor       | Direct connect                   |
| TB6612 logic       | 5 V    | bi-dir on AIN/BIN/PWM/STBY | Direct connect          |
| HC-05 TX → D0      | 3.3 V  | module → MCU       | OK (5 V MCU reads 3.3 V as HIGH) |
| HC-05 RX ← D1      | 5 V→3.3 V | MCU → module     | Divider required if bare module  |

---

## 5. ASCII block diagram

```
              ┌──────────────────────────────────────┐
              │           Arduino Nano               │
              │           (ATmega328P)               │
              │                                      │
   HC-05 TX ──► D0 (PD0/RXD)               D13 (PB5) ───► Onboard LED (status)
   HC-05 RX ◄── D1 (PD1/TXD) [÷ to 3.3V]   D12 (PB4) ───► TB6612 BIN1
                D2 (PD2)     ─────────────► TB6612 STBY
                D3 (PD3/OC2B)─────────────► TB6612 PWMA   (left motor)
                D4 (PD4)     ─────────────► TB6612 AIN1
                D5 (PD5)     ─────────────► HC-SR04 Front TRIG
                D6 (PD6)     ◄──────────── HC-SR04 Front ECHO  (PCI2)
                D7 (PD7)     ─────────────► TB6612 AIN2
                D8 (PB0)     ─────────────► TB6612 BIN2
                D9 (PB1)     ◄──────────── HC-SR04 Right ECHO  (PCI0)
                D10(PB2)     ─────────────► HC-SR04 Right TRIG
                D11(PB3/OC2A)─────────────► TB6612 PWMB   (right motor)
                                                          ┌─────────────────┐
                A0 (PC0)     ─────────────► HC-SR04 Left TRIG               │
                A3 (PC3)     ◄──────────── HC-SR04 Left ECHO   (PCI1)       │
                A4/A5        ── reserved I²C                                │
                A6/A7        ── analog spare (battery sense)                │
              └──────────────────────────────────────┘                      │
                       │                          │                         │
                       │ 5V                       │ GND ◄── common ground ──┤
                       ▼                          ▼                         │
              ┌─────────────────┐        ┌─────────────────┐                │
              │  5 V BEC / reg  │        │   Battery (−)   │                │
              └────────┬────────┘        └────────┬────────┘                │
                       │                          │                         │
              ┌────────▼─────────────────┐    ┌───▼────────────┐            │
              │ HC-SR04 ×3, HC-05, Nano  │    │ Battery (+)    │            │
              │ TB6612 logic VCC         │    └───┬────────────┘            │
              └──────────────────────────┘        │                         │
                                                  │ VM                      │
                                          ┌───────▼─────────┐               │
                                          │   TB6612FNG     │── AO1/AO2 ──► Left motor
                                          │  motor driver   │── BO1/BO2 ──► Right motor
                                          └─────────────────┘               │
                                                                            │
              All GNDs (Nano, sensors, HC-05, TB6612 GND, battery −) ───────┘
```

---

## 6. Suggested firmware pin macro header (for Step 2 onward)

```c
/* pinmap.h — single source of truth, must match docs/PINOUT.md */

/* HC-05 (USART0) — D0/D1, no port macros needed */

/* TB6612FNG */
#define MOT_STBY_PORT  PORTD
#define MOT_STBY_DDR   DDRD
#define MOT_STBY_BIT   PD2

#define MOT_PWMA_BIT   PD3   /* OC2B */
#define MOT_AIN1_BIT   PD4
#define MOT_AIN2_BIT   PD7

#define MOT_BIN2_BIT   PB0
#define MOT_BIN1_BIT   PB4
#define MOT_PWMB_BIT   PB3   /* OC2A */

/* HC-SR04 trigs */
#define TRIG_FRONT_PORT PORTD
#define TRIG_FRONT_DDR  DDRD
#define TRIG_FRONT_BIT  PD5

#define TRIG_RIGHT_PORT PORTB
#define TRIG_RIGHT_DDR  DDRB
#define TRIG_RIGHT_BIT  PB2

#define TRIG_LEFT_PORT  PORTC
#define TRIG_LEFT_DDR   DDRC
#define TRIG_LEFT_BIT   PC0

/* HC-SR04 echoes (each on its own PCINT group) */
#define ECHO_FRONT_PIN  PIND
#define ECHO_FRONT_BIT  PD6   /* PCINT22 */
#define ECHO_RIGHT_PIN  PINB
#define ECHO_RIGHT_BIT  PB1   /* PCINT1  */
#define ECHO_LEFT_PIN   PINC
#define ECHO_LEFT_BIT   PC3   /* PCINT11 */

/* Status LED */
#define LED_PORT  PORTB
#define LED_DDR   DDRB
#define LED_BIT   PB5
```

---

## 7. Pre-power-on checklist

- [ ] Every Nano pin in the table is wired to exactly one device.
- [ ] No PWM signal on a non-PWM pin (PWMA on D3, PWMB on D11 — both Timer2).
- [ ] Each ECHO on a different PCINT group (D6 = PCI2, D9 = PCI0, A3 = PCI1).
- [ ] HC-05 RX line has a divider (or the module is a level-shifted breakout).
- [ ] Motor power (VM) and logic power (VCC) are separate rails.
- [ ] All GNDs tied together at one star point.
- [ ] STBY (D2) wired — driver outputs are Hi-Z until D2 is driven HIGH.
- [ ] Onboard LED on D13 free for use (no external load on it).
