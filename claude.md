# Wall-Following Autonomous Robot - Project Documentation

## Project Overview

This is an embedded systems project requiring the design and implementation of an autonomous mobile robot capable of navigating a predefined track by following walls, detecting 90° turns, and communicating results to a PC via WiFi/Bluetooth.

**Team Size:** 5 members

---

## Core Learning Objectives

- Master low-level embedded C programming without high-level Arduino libraries
- Understand sensor interfacing and placement for accurate wall detection (IR or ultrasonic)
- Design and implement a Finite State Machine (FSM) for real-time control systems
- Develop real-time control system architecture
- Learn communication protocols (UART over WiFi or Bluetooth)
- Integrate mechanical, electrical, and software components into a cohesive system

---

## System Requirements

### Functional Requirements

- Robot must start autonomously and maintain wall-following behavior
- Accurately detect and handle 90° turns only
- Count all turns and classify each as Left (L) or Right (R)
- Transmit structured data to PC in the following format:
  ```
  Turns: 5
  Sequence: L, R, R, L, L
  ```

### Non-Functional Requirements

- **Register-level programming:** Must use register-level access and custom drivers
- **No high-level APIs:** Strictly avoid Arduino functions like `digitalWrite()`, `analogRead()`, etc.
- **Real-time response:** Blocking delays must be minimized; use interrupt-driven and non-blocking approaches

---

## Phase 1: Design & Validation

**Deadline:** Week 12

This phase focuses on comprehensive planning and design documentation before hardware implementation.

### Robot Design Constraints

| Parameter | Value |
|-----------|-------|
| Maximum Width | 20 cm |
| Maximum Length | 20 cm |
| Configuration | Must ensure stability and maneuverability |
| Sensor Placement | Maximize wall detection accuracy |

### Track Design Specifications

| Parameter | Value |
|-----------|-------|
| Corridor Width | 45 cm |
| Wall Height | ≥ 10 cm |
| Turn Type | 90° only (no curves) |

### Finite State Machine (FSM) Requirements

Your FSM design must include:

- **State diagram** with all possible states
- **Transition table** documenting what sensor inputs trigger each state change
- **Contingency handling** for scenarios such as:
  - Loss of wall signal
  - Detection of open spaces
  - Transition between wall-following and turn detection
  - Recovery from temporary sensor errors

#### Suggested FSM States

| State | Purpose | Transitions |
|-------|---------|-------------|
| **FOLLOW_WALL** | Maintain wall proximity while moving | → DETECT_TURN on opening detected |
| **DETECT_TURN** | Identify turn direction (L/R) | → EXECUTE_TURN after direction determined |
| **EXECUTE_TURN** | Perform 90° rotation and re-align | → FOLLOW_WALL after alignment |
| **LOST_WALL** | Emergency state when wall signal lost | → SEARCH_WALL or → FOLLOW_WALL if refound |
| **SEARCH_WALL** | Attempt to relocate wall signal | → FOLLOW_WALL when wall re-detected |

### Phase 1 Deliverables

#### 1. Mechanical Design Document
- Robot dimensions and geometry
- Wheel configuration and placement
- Motor placement for balanced torque distribution
- Sensor mounting locations with justification for placement
- Center of mass analysis for stability
- Material selections and weight estimates

#### 2. Electrical Design Document
- Block diagram showing all components and interconnections
- Microcontroller pin assignments
- Sensor circuit diagrams (power supply, signal conditioning)
- Motor driver circuits
- Communication module interface (WiFi/Bluetooth)
- Power distribution and battery selection
- Protection circuits (fuses, diodes, voltage regulators)

#### 3. FSM Design Document
- State diagram (visual representation)
- Transition table with:
  - Current state
  - Input conditions (sensor readings)
  - Next state
  - Output actions (motor commands, flags)
- Explanation of design rationale
- Handling of edge cases and error conditions

#### 4. Algorithm Description
- **Wall Following Algorithm:**
  - Sensor input processing
  - Distance calculation and error correction
  - Motor speed adjustment logic
  - PID control strategy (if applicable)
  
- **Turn Detection Algorithm:**
  - Criteria for identifying a turn
  - Direction classification logic
  - Turn counting mechanism
  - State transition triggers

- **Communication Protocol:**
  - Message format and structure
  - Error checking (checksums, acknowledgments)
  - Transmission timing and frequency

#### 5. Communication Plan
- **Selection Justification:** Why WiFi or Bluetooth?
- **Data Format:** Exact structure sent to PC
- **Protocol Details:**
  - Baud rate / transmission speed
  - Handshaking mechanism
  - Error handling and retransmission
  - PC-side receiving script outline

### Phase 1 Evaluation Criteria

| Criteria | Weight |
|----------|--------|
| Design Feasibility | 25% |
| FSM Correctness | 35% |
| Documentation Clarity | 20% |
| Engineering Reasoning | 20% |

**Advancement Requirement:** Teams must pass Phase 1 evaluation to proceed to Phase 2.

---

## Phase 2: Final Competition

**Deadline:** Week 14

### Rules

- Each team receives 2 trial runs
- Best trial score is counted toward final ranking
- Robot must autonomously complete the track and transmit turn data

### Scoring System

```
Final Score = Time (seconds) + Penalties (seconds)
Lower score is better
```

### Penalty Table

| Violation | Penalty |
|-----------|---------|
| Collision with wall | +2 sec |
| Losing track for >2 sec | +5 sec |
| Wrong turn detection | +3 sec |
| Missing turn count | +3 sec |
| Manual intervention | Disqualification |
| Failure to send data | +5 sec |

### Ranking Criteria

1. **Primary:** Lowest final score
2. **Tiebreaker 1:** Fastest raw completion time
3. **Tiebreaker 2:** Fewest penalties incurred

---

## Suggested Hardware

### Microcontroller Options

- **Tiva C (TM4C123GH6PM)** - Good ADC, plenty of I/O
- **ESP32** - Built-in WiFi, dual-core, good for communication
- **STM32** - High performance, extensive peripheral support
- **AVR (Arduino-compatible)** - Budget-friendly, well-documented

### Sensors

- **Ultrasonic Sensors:** Good range, less affected by surface color
- **IR Sensors:** Faster response, cheaper, but sensitive to surface reflectivity

**Recommended:** 3-4 sensors strategically placed for robust wall detection

### Motors

- **Type:** DC motors with encoders
- **Encoders:** For speed feedback and distance estimation
- **Motor Driver:** H-bridge or dedicated motor controller (L298N, DRV8835, etc.)

### Communication

- **WiFi:** ESP32 module (built-in or separate)
- **Bluetooth:** HC-05 or HC-06 Bluetooth module
- **Serial Interface:** UART for debugging and communication

### Power

- **Battery:** Lithium-ion or NiMH for extended runtime
- **Voltage Regulation:** Multiple regulators for microcontroller (3.3V/5V) and motors (6-12V)

---

## Documentation Guidelines

Your final report should follow this structure:

### Report Format

1. **Introduction**
   - Project objective
   - Problem statement
   - Scope and constraints

2. **System Overview**
   - High-level architecture
   - System components and their roles
   - Integration approach

3. **Mechanical Design**
   - Robot geometry and dimensions
   - Wheel configuration
   - Sensor mounting strategy
   - CAD drawings or detailed sketches
   - Material and weight analysis

4. **Electrical Design**
   - Component list with specifications
   - Block diagram
   - Schematic diagrams
   - Pin assignments
   - Power budget analysis

5. **FSM and Algorithm**
   - FSM state diagram with detailed explanation
   - Transition table
   - Wall-following algorithm with pseudocode
   - Turn detection logic
   - Communication protocol specification

6. **Testing Plan**
   - Unit testing strategy (individual sensors, motors)
   - Integration testing approach
   - Performance metrics and acceptance criteria
   - Expected challenges and mitigation strategies

7. **References and Appendices**
   - Datasheets
   - Code snippets
   - Additional diagrams

### Diagram Requirements

- **FSM Diagram:** State circles, labeled transitions with conditions
- **Track Layout:** Scaled drawing showing corridor width, turn positions
- **Robot Design:** Top view, side view, sensor placement diagrams
- **Electrical Schematic:** Clear component connections
- **Block Diagram:** System components and signal flow

### Explanation Standards

Write all explanations suitable for beginners:
- Define technical terms on first use
- Break down complex logic into steps
- Explain **why** decisions were made, not just **what** was built
- Include worked examples for sensor readings and motor control calculations

---

## Key Design Considerations

### Wall Following Strategy

1. **Sensor Placement:** Typically, side-facing sensors (perpendicular to wall) detect distance
2. **Control Law:** Proportional or PID control adjusts steering to maintain target distance
3. **Speed Management:** Reduce speed during turns; maintain steady speed on straight sections
4. **Hysteresis:** Apply threshold bands to prevent oscillation

### Turn Detection Strategy

1. **Opening Detection:** Sensor readings show sudden absence of wall
2. **Direction Classification:**
   - Left turn: Opening detected on left sensor
   - Right turn: Opening detected on right sensor
3. **Confirmation:** Verify turn detection before incrementing counter
4. **Post-Turn Alignment:** Re-align to wall after completing the turn

### Real-Time Constraints

- **Sensor Reading Frequency:** 100+ Hz for responsive control
- **State Machine Update:** ≥100 Hz to catch rapid changes
- **Motor Command:** ≥50 Hz for smooth motion
- **Non-Blocking I/O:** Use interrupts, timers, and state machines
- **Avoid:** `delay()`, busy-waiting, long blocking operations

---

## Success Criteria Checklist

### Phase 1

- [ ] Mechanical design is feasible within weight and size constraints
- [ ] Electrical design handles all components safely with proper power distribution
- [ ] FSM includes all necessary states and transitions
- [ ] Algorithm descriptions are clear and logical
- [ ] Documentation follows engineering report standards
- [ ] Team demonstrates understanding of design choices

### Phase 2

- [ ] Robot completes track without manual intervention
- [ ] Turn count matches actual count
- [ ] Turn sequence correctly classified as L/R
- [ ] Data successfully transmitted to PC
- [ ] Final score (time + penalties) is minimized
- [ ] Robot recovers from minor wall contact without disqualification

---

## Resources for Implementation

### Embedded C Programming

- Register-level GPIO and timer configuration
- UART communication setup
- Interrupt handling for sensor inputs
- Non-blocking state machine patterns

### Sensor Integration

- ADC (Analog-to-Digital Conversion) for analog sensors
- Digital signal processing for noise filtering
- Calibration procedures for sensor accuracy

### Control Systems

- Basic PID control theory
- Motor speed control via PWM (Pulse Width Modulation)
- Feedback systems and closed-loop control

### Communication

- UART protocol fundamentals
- WiFi/Bluetooth serial protocols
- Packet structure and error handling

---

## Timeline Overview

| Phase | Duration | Focus | Deliverable |
|-------|----------|-------|-------------|
| Design & Planning | Weeks 1-12 | Documentation and Design | Phase 1 Report |
| Component Testing | Weeks 9-12 | Early prototyping and validation | Working sensor/motor tests |
| Assembly | Weeks 12-13 | Build prototype, integrate subsystems | Functional prototype |
| Final Competition | Week 14 | Performance testing and optimization | Best score + running robot |

---

## Contact & Support

- **Instructor:** [Provided by course]
- **Office Hours:** [Provided by course]
- **Lab Resources:** [List available equipment and tools]

---

**Last Updated:** Spring 2026  
**Version:** 1.0
