/*
 * WallBot — PID centering + sensor-gated 90° turn execution.
 *
 * Strict register-level. No Arduino library calls.
 *
 * FSM:
 *   CORRECTION   PID-center between L and R walls. Opening predicate:
 *                front close (F <= F_DETECT_MAX_MM) AND one side wide
 *                (side >= SIDE_OPEN_MM), sustained for OPENING_CONFIRM
 *                cycles. On confirm, latch direction -> APPROACH.
 *   APPROACH    Direction is locked. Keep moving forward until the
 *                front wall closes to F_TURN_MM. PID is one-sided
 *                (only the wall that is NOT opening). No new opening
 *                checks, no front emergency stop. APPROACH_TIMEOUT_OVFS
 *                is a safety bound.
 *   SPIN         Tank-rotate in latched direction at TURN_SPIN_PCT for
 *                TURN_SPIN_OVFS. Pure open-loop, no sensor reads, no
 *                early exits. Calibrate TURN_SPIN_OVFS until ~90°.
 *   SETTLE       Brake briefly. Reset PID state. Record turn.
 *                -> CORRECTION.
 *
 * Output (every PRINT_PERIOD_OVFS):
 *   [STA] L=<mm> R=<mm> F=<mm> err=<mm> oL=<pwm> oR=<pwm> T=<count> seq=<L,R,...>
 *   STA in { COR, APP, SPN, SET }.
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

/* ---- TUNABLES: motion (your tuned values) ----------------------------- */

#define MAX_SPEED_PCT     100
#define BASE_SPEED_PCT     75      /* forward speed when centred */
#define MOTOR_TRIM_PCT      6      /* L+ / R- to fight leftward drift */

#define KP_X100            18      /* Kp = 0.18 */
#define KD_X100            68      /* Kd = 0.68 */
#define KI_X100             0
#define INTEG_LIMIT      4000

#define FRONT_STOP_MM     70      /* hard brake threshold (FOLLOW only) */

/* ---- TUNABLES: turn detection ----------------------------------------- */

/* Opening predicate requires BOTH conditions, sustained OPENING_CONFIRM
 * cycles. 1 overflow ~= 32.768 ms, so 3 cycles ~= 98 ms of debounce. */
#define F_DETECT_MAX_MM     350    /* front must be <= this (corner ahead) */
#define SIDE_OPEN_MM        400    /* a side counts as open at >= this     */
#define OPENING_CONFIRM       3    /* consecutive cycles before latching   */

/* ---- TUNABLES: turn execution ----------------------------------------- */

/* Commit to spin when the front wall is this close. Replaces the old
 * blind TURN_ENTRY_OVFS forward burst. */
#define F_TURN_MM           150

/* Safety net: if F never crosses F_TURN_MM (e.g. front sensor failure),
 * spin anyway after this long in APPROACH. ~2.0 s @ 32.768 ms/ovf. */
#define APPROACH_TIMEOUT_OVFS  60

/* Open-loop tank spin. Calibrate TURN_SPIN_OVFS empirically:
 *   45 ovfs ~= 1.47 s, 60 ovfs ~= 1.97 s — within the 1-2 s spec. */
#define TURN_SPIN_PCT        35    /* PWM during spin (0..100)             */
#define TURN_SPIN_OVFS       23    /* ~1.47 s — TUNE until 90° lands flat  */
#define TURN_SETTLE_OVFS      4    /* ~131 ms brake before PID resumes     */

/* Forward speed during APPROACH. Slightly slower than BASE so we don't
 * overshoot F_TURN_MM between cycles. */
#define APPROACH_SPEED_PCT   50

/* One-sided PID gain in APPROACH. The opening side is unreliable, so we
 * steer off the *other* wall only. Setpoint = half corridor width. */
#define APPROACH_SETPOINT_MM 225   /* corridor is 450 mm, half = 225 mm    */
#define APPROACH_KP_X100      20

/* Sequence buffer. 32 turns is more than any reasonable course. */
#define TURN_SEQ_MAX         32

/* ---- TUNABLES: scheduling ---------------------------------------------- */

#define PID_PERIOD_OVFS     1      /* control rate */
#define PRINT_PERIOD_OVFS   8      /* ~262 ms */

/* ---- DERIVED ---------------------------------------------------------- */

#define PCT_TO_PWM(p)     ((int16_t)(((int32_t)(p) * 255) / 100))
#define MAX_SPEED         PCT_TO_PWM(MAX_SPEED_PCT)
#define BASE_SPEED        PCT_TO_PWM(BASE_SPEED_PCT)
#define APPROACH_SPEED    PCT_TO_PWM(APPROACH_SPEED_PCT)
#define TURN_SPIN_PWM     PCT_TO_PWM(TURN_SPIN_PCT)

/* ---- STATE ------------------------------------------------------------ */

typedef enum { ST_CORRECTION = 0, ST_APPROACH, ST_SPIN, ST_SETTLE } state_t;

static state_t state = ST_CORRECTION;
static uint16_t state_start_ovf = 0;
static char     pending_dir = 'L';      /* direction latched at CORRECTION exit */

static uint8_t  open_count_L = 0;       /* consecutive "open" cycles, left  */
static uint8_t  open_count_R = 0;       /* consecutive "open" cycles, right */

static uint8_t  turn_count = 0;
static char     turn_seq[TURN_SEQ_MAX];

static int16_t  prev_error = 0;
static int32_t  integ      = 0;

static int16_t  last_outL  = 0;
static int16_t  last_outR  = 0;
static int16_t  last_error = 0;
static uint8_t  front_braking = 0;

/* ---- HELPERS ---------------------------------------------------------- */

static inline int16_t clamp16(int32_t v, int16_t lim) {
    if (v >  lim) return  lim;
    if (v < -lim) return -lim;
    return (int16_t)v;
}

static inline uint8_t side_is_open(uint16_t mm) {
    return (mm == US_NO_READING) || (mm >= SIDE_OPEN_MM);
}

static inline uint8_t front_is_close(uint16_t mm) {
    return (mm != US_NO_READING) && (mm <= F_DETECT_MAX_MM);
}

static void enter_state(state_t s, uint16_t now_ovf) {
    state = s;
    state_start_ovf = now_ovf;
}

static void reset_pid(void) {
    prev_error = 0;
    integ      = 0;
    open_count_L = open_count_R = 0;
}

static void record_turn(char dir) {
    if (turn_count < TURN_SEQ_MAX) turn_seq[turn_count] = dir;
    turn_count++;
    uart_puts("\r\n*** TURN #"); uart_put_u16(turn_count);
    uart_puts(" "); uart_putc(dir); uart_puts(" ***\r\n");
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

static const char *state_tag(void) {
    switch (state) {
        case ST_CORRECTION:  return "COR";
        case ST_APPROACH:    return "APP";
        case ST_SPIN:        return "SPN";
        case ST_SETTLE:      return "SET";
    }
    return "???";
}

/* ---- CORRECTION: PID + opening detection ------------------------------ */

static void correction_step(uint16_t ovf) {
    uint16_t L = us_distance_mm(US_LEFT);
    uint16_t R = us_distance_mm(US_RIGHT);
    uint16_t F = us_distance_mm(US_FRONT);

    /* Opening predicate: front close AND that side wide, sustained.
     * Both conditions must hold on the SAME cycle to advance the count. */
    uint8_t fclose = front_is_close(F);
    if (fclose && side_is_open(L)) {
        if (open_count_L < 255) open_count_L++;
    } else {
        open_count_L = 0;
    }
    if (fclose && side_is_open(R)) {
        if (open_count_R < 255) open_count_R++;
    } else {
        open_count_R = 0;
    }

    if (open_count_L >= OPENING_CONFIRM || open_count_R >= OPENING_CONFIRM) {
        /* Left wins on a tie (left-hand rule). */
        pending_dir = (open_count_L >= OPENING_CONFIRM) ? 'L' : 'R';
        front_braking = 0;
        enter_state(ST_APPROACH, ovf);
        return;
    }

    /* Front emergency stop — only honored in CORRECTION, never during a
     * committed turn. Once a turn is latched, APPROACH/SPIN/SETTLE drive
     * past the front-stop region on purpose. */
    if (F != US_NO_READING && F < FRONT_STOP_MM) {
        motors_stop();
        last_outL = last_outR = 0;
        front_braking = 1;
        return;
    }
    front_braking = 0;

    /* If a side is missing but no opening has confirmed, crawl forward.
     * Avoids latching a noisy NO_READING into an unwanted PID swing. */
    if (L == US_NO_READING || R == US_NO_READING) {
        int16_t crawl = BASE_SPEED / 2;
        motors_set(crawl, crawl);
        last_outL = last_outR = crawl;
        last_error = 0;
        return;
    }

    /* PID on lateral error. */
    int16_t error = (int16_t)R - (int16_t)L;
    int16_t deriv = error - prev_error;
    integ += error;
    if (integ >  INTEG_LIMIT) integ =  INTEG_LIMIT;
    if (integ < -INTEG_LIMIT) integ = -INTEG_LIMIT;

    int32_t turn = ((int32_t)KP_X100 * error
                  + (int32_t)KD_X100 * deriv
                  + (int32_t)KI_X100 * integ) / 100;

    int16_t turn_pwm = clamp16(turn, BASE_SPEED);
    int16_t left  = (int16_t)((int32_t)BASE_SPEED + turn_pwm);
    int16_t right = (int16_t)((int32_t)BASE_SPEED - turn_pwm);

    int16_t trim_pwm = PCT_TO_PWM(MOTOR_TRIM_PCT);
    left  += trim_pwm;
    right -= trim_pwm;

    if (left  < 0)         left  = 0;
    if (right < 0)         right = 0;
    if (left  > MAX_SPEED) left  = MAX_SPEED;
    if (right > MAX_SPEED) right = MAX_SPEED;

    motors_set(left, right);

    prev_error = error;
    last_outL  = left;
    last_outR  = right;
    last_error = error;
}

/* ---- APPROACH / SPIN / SETTLE ----------------------------------------- *
 * Direction is locked. No opening checks, no front-stop, no transitions
 * back to CORRECTION until SETTLE finishes. Ensures the turn cannot be
 * preempted by transient sensor readings.
 */

static void approach_step(uint16_t ovf) {
    uint16_t F = us_distance_mm(US_FRONT);

    /* Commit to spin once we are at the corner. */
    if (F != US_NO_READING && F <= F_TURN_MM) {
        enter_state(ST_SPIN, ovf);
        return;
    }
    /* Safety: front sensor failure must not leave us creeping forever. */
    if ((uint16_t)(ovf - state_start_ovf) >= APPROACH_TIMEOUT_OVFS) {
        enter_state(ST_SPIN, ovf);
        return;
    }

    /* One-sided P-only steering. The opening side is unreliable; use the
     * wall that is NOT opening. error > 0 means too far from that wall,
     * so steer toward it. */
    uint16_t hold;
    int16_t  err;
    if (pending_dir == 'L') {
        hold = us_distance_mm(US_RIGHT);
        if (hold == US_NO_READING) {
            err = 0;
        } else {
            err = (int16_t)APPROACH_SETPOINT_MM - (int16_t)hold;
        }
    } else {
        hold = us_distance_mm(US_LEFT);
        if (hold == US_NO_READING) {
            err = 0;
        } else {
            err = (int16_t)hold - (int16_t)APPROACH_SETPOINT_MM;
        }
    }

    int32_t correct = ((int32_t)APPROACH_KP_X100 * err) / 100;
    int16_t correct_pwm = clamp16(correct, APPROACH_SPEED);

    int16_t left  = (int16_t)((int32_t)APPROACH_SPEED + correct_pwm);
    int16_t right = (int16_t)((int32_t)APPROACH_SPEED - correct_pwm);

    int16_t trim_pwm = PCT_TO_PWM(MOTOR_TRIM_PCT);
    left  += trim_pwm;
    right -= trim_pwm;

    if (left  < 0)         left  = 0;
    if (right < 0)         right = 0;
    if (left  > MAX_SPEED) left  = MAX_SPEED;
    if (right > MAX_SPEED) right = MAX_SPEED;

    motors_set(left, right);
    last_outL  = left;
    last_outR  = right;
    last_error = err;
}

static void spin_step(uint16_t ovf) {
    /* Pure open-loop. No sensor reads, no early aborts. */
    if (pending_dir == 'L') {
        motors_set(-TURN_SPIN_PWM, +TURN_SPIN_PWM);
        last_outL = -TURN_SPIN_PWM; last_outR = +TURN_SPIN_PWM;
    } else {
        motors_set(+TURN_SPIN_PWM, -TURN_SPIN_PWM);
        last_outL = +TURN_SPIN_PWM; last_outR = -TURN_SPIN_PWM;
    }
    if ((uint16_t)(ovf - state_start_ovf) >= TURN_SPIN_OVFS) {
        enter_state(ST_SETTLE, ovf);
    }
}

static void settle_step(uint16_t ovf) {
    motors_stop();
    last_outL = last_outR = 0;
    if ((uint16_t)(ovf - state_start_ovf) >= TURN_SETTLE_OVFS) {
        record_turn(pending_dir);
        reset_pid();
        enter_state(ST_CORRECTION, ovf);
    }
}

/* ---- ENTRY POINTS ----------------------------------------------------- */

void setup(void) {
    LED_DDR  |=  (1 << LED_BIT);
    LED_PORT &= ~(1 << LED_BIT);

    uart_init(9600);
    us_init();
    motors_init();
    sei();

    uart_puts("\r\nWallBot: PID + sensor-gated 90 deg turn\r\n");
    uart_puts("MAX="); uart_put_u16(MAX_SPEED_PCT);
    uart_puts("%  BASE="); uart_put_u16(BASE_SPEED_PCT);
    uart_puts("%  TRIM="); uart_put_i16(MOTOR_TRIM_PCT);
    uart_puts("%  Kp*100="); uart_put_u16(KP_X100);
    uart_puts(" Kd*100=");   uart_put_u16(KD_X100);
    uart_puts(" Ki*100=");   uart_put_u16(KI_X100);
    uart_puts("\r\nF_DET<="); uart_put_u16(F_DETECT_MAX_MM);
    uart_puts(" SIDE>=");     uart_put_u16(SIDE_OPEN_MM);
    uart_puts(" CONFIRM=");   uart_put_u16(OPENING_CONFIRM);
    uart_puts(" F_TURN<=");   uart_put_u16(F_TURN_MM);
    uart_puts(" SPIN=");      uart_put_u16(TURN_SPIN_OVFS);
    uart_puts("@");           uart_put_u16(TURN_SPIN_PCT);
    uart_puts("%\r\n");

    motors_enable(1);
}

void loop(void) {
    static uint16_t last_print_ovf = 0;
    static uint16_t last_pid_ovf   = 0;

    us_service();

    uint16_t ovf = us_overflow_count();

    if ((uint16_t)(ovf - last_pid_ovf) >= PID_PERIOD_OVFS) {
        last_pid_ovf = ovf;
        switch (state) {
            case ST_CORRECTION: correction_step(ovf); break;
            case ST_APPROACH:   approach_step(ovf);   break;
            case ST_SPIN:       spin_step(ovf);       break;
            case ST_SETTLE:     settle_step(ovf);     break;
        }
    }

    if ((uint16_t)(ovf - last_print_ovf) >= PRINT_PERIOD_OVFS) {
        last_print_ovf = ovf;
        uart_putc('['); uart_puts(state_tag()); uart_puts("] ");
        print_dist ("L=",   us_distance_mm(US_LEFT));
        print_dist ("R=",   us_distance_mm(US_RIGHT));
        print_dist ("F=",   us_distance_mm(US_FRONT));
        print_field("err=", last_error, 1);
        print_field("oL=",  last_outL,  1);
        print_field("oR=",  last_outR,  1);
        print_field("T=",   (int16_t)turn_count, 0);
        if (turn_count) {
            uart_puts("seq=");
            uint8_t n = turn_count < TURN_SEQ_MAX ? turn_count : TURN_SEQ_MAX;
            for (uint8_t i = 0; i < n; i++) {
                if (i) uart_putc(',');
                uart_putc(turn_seq[i]);
            }
        }
        if (front_braking) uart_puts(" [STOP]");
        uart_puts("\r\n");
        LED_PORT ^= (1 << LED_BIT);
    }
}
