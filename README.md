<a id="readme-top"></a>

# Wall-Following Autonomous Robot

<br />
<div align="center">
  <h3 align="center">Wall-Following Autonomous Robot</h3>
</div>

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
        <li><a href="#system-overview">System Overview</a></li>
      </ul>
    </li>
    <li><a href="#getting-started">Getting Started</a></li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#project-structure">Project Structure</a></li>
    <li><a href="#hardware-and-wiring">Hardware and Wiring</a></li>
    <li><a href="#license">License</a></li>
  </ol>
</details>

### <a id="about-the-project"></a>About The Project

This project implements a wall-following autonomous robot for a predefined indoor track with 90° turns only. The robot uses register-level AVR C programming and low-level drivers instead of Arduino high-level APIs. It detects left and right turns, counts them, and sends the turn report to a PC through Bluetooth UART.

The firmware is designed for real-time control using an FSM, ultrasonic range sensing, motor PWM control, and serial communication.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## <a id="built-with"></a>🛠️ Built With

[![C](https://img.shields.io/badge/C-00599C?style=for-the-badge&logo=c&logoColor=white)](<https://en.wikipedia.org/wiki/C_(programming_language)>)
[![AVR](https://img.shields.io/badge/AVR-FB8B24?style=for-the-badge&logo=arduino&logoColor=white)](https://www.microchip.com/design-centers/8-bit/avr-mcus)
[![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=arduino&logoColor=white)](https://www.arduino.cc/)
[![HC-SR04](https://img.shields.io/badge/HC--SR04-Ultrasonic-blue?style=for-the-badge&logo=raspberry-pi&logoColor=white)](https://en.wikipedia.org/wiki/Ultrasonic_sensor)
[![TB6612](https://img.shields.io/badge/TB6612FNG-Motor%20Driver-brightgreen?style=for-the-badge&logo=electronics&logoColor=white)](https://www.sparkfun.com/products/10167)
[![Bluetooth](https://img.shields.io/badge/Bluetooth-HC--05-2F5F9A?style=for-the-badge&logo=bluetooth&logoColor=white)](https://www.bluetooth.com/)

- **C / AVR** — register-level firmware for ATmega328P
- **Arduino Nano** — MCU platform
- **HC-SR04 ultrasonic sensors** — front, left, right wall detection
- **TB6612FNG motor driver** — dual DC motor control
- **HC-05 Bluetooth** — UART serial communication to PC
- **Timers and interrupts** — Timer1 for sensor timing, Timer2 for motor PWM, PCINTs for echo capture

## <a id="system-overview"></a>System Overview

- `WallBot/WallBot.ino` is the main firmware sketch.
- `WallBot/pinmap.h` defines the physical pin assignments and matches `docs/PINOUT.md`.
- `WallBot/ultrasonic.c` handles non-blocking sonar triggering and echo timing.
- `WallBot/motors.c` controls motor direction, PWM, and standby.
- `WallBot/turn_tracker.c` logs turns and sends the final report over UART.
- `WallBot/uart.c` provides buffered serial transmit/receive using USART0.
- `WallBot/completion_detector.c` detects when the robot exits the course.

The control firmware uses these states:

- `ST_CORRECTION` — wall following with PID correction
- `ST_PRE_TURN` — pre-turn detection/confirmation
- `ST_SPIN` — execute the 90° turn
- `ST_POST_TURN` — stabilize after the turn
- `ST_REVERSE_RECOVERY` — recover from collisions or being stuck
- `ST_COMPLETE` — stop and send final turn report

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## <a id="getting-started"></a>Getting Started

Follow these steps to build and run the robot firmware.

### Prerequisites

- [Arduino IDE](https://www.arduino.cc/en/software) or compatible AVR build environment
- Arduino Nano (ATmega328P, 16 MHz)
- HC-SR04 ultrasonic sensors
- TB6612FNG motor driver
- HC-05 Bluetooth module
- DC motors and chassis hardware

### Installation

1. Clone the repository

   ```sh
   git clone https://github.com/<your-username>/em_proj.git
   cd em_proj
   ```

2. Open `WallBot/WallBot.ino` in the Arduino IDE.
3. Select the board: `Arduino Nano` with `ATmega328P` and the correct bootloader.
4. Select the correct serial port and upload.
5. Power the robot from its battery pack, ensuring common ground between the Nano, motor driver, sensors, and HC-05.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## <a id="usage"></a>📖 Usage

- Place the robot at the start of the predefined corridor track.
- Ensure the walls are within the ultrasonic sensor range and the track contains only 90° turns.
- The robot will follow the wall autonomously, detect turns, and count left/right corners.
- After completing the track, it sends a report via Bluetooth in this format:

  ```txt
  Turns: 5
  Sequence: L, R, R, L, L
  ```

- Open a serial terminal at **9600 baud, 8N1** to receive the output from the HC-05 module.

### What it does

- Wall-following using three ultrasonic sensors
- Turn detection and classification as `L` or `R`
- Non-blocking real-time control loop with state machine
- Collision / stuck recovery using reverse and sensor re-evaluation
- Completion detection with final turn report over UART

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## <a id="project-structure"></a>Project Structure

```text
em_proj/
├── README.md
├── docs/
│   └── PINOUT.md
└── WallBot/
    ├── completion_detector.c
    ├── completion_detector.h
    ├── motors.c
    ├── motors.h
    ├── pinmap.h
    ├── turn_tracker.c
    ├── turn_tracker.h
    ├── uart.c
    ├── uart.h
    ├── ultrasonic.c
    ├── ultrasonic.h
    └── WallBot.ino
```

### Key modules

- `pinmap.h` — pin assignment source of truth
- `uart.c` — buffered serial communication for Bluetooth
- `ultrasonic.c` — scheduled HC-SR04 sensor polling and echo filtering
- `motors.c` — PWM speed control and motor direction handling
- `turn_tracker.c` — turn logging and formatted report output
- `completion_detector.c` — finish-line/open-space detection logic

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## <a id="hardware-and-wiring"></a>Hardware and Wiring

See `docs/PINOUT.md` for the full pin assignment and wiring reference.

### Sensors

- Front HC-SR04 for corridor forward distance.
- Left HC-SR04 and right HC-SR04 for wall tracking.

### Actuators

- Left and right DC motors driven by TB6612FNG.
- Motor PWM on Arduino pins D3 and D11.

### Communication

- HC-05 Bluetooth connected to Arduino pins D0 (RX) and D1 (TX).
- UART configured at `9600 8N1`.

### Important notes

- The project avoids Arduino high-level APIs such as `digitalWrite()` and `analogRead()`.
- Timer1 is reserved for ultrasonic timing and scheduling.
- Timer2 is used for high-frequency PWM to the motor driver.
- The firmware performs non-blocking control with interrupts and periodic state updates.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## <a id="license"></a>License

This project is intended for educational use and academic demonstration only.

<p align="right">(<a href="#readme-top">back to top</a>)</p>
