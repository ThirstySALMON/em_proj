/*
 * WallBot — PID centering + sensor-gated 90° turn execution.
 * ...same header comment as before...
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "pinmap.h"
#include "uart.h"
#include "ultrasonic.h"
#include "motors.h"
#include "turn_tracker.h"

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

/* ---- TUNABLES: motion (your tuned values) ----------------------------- */

#define MAX_SPEED_PCT     100
#define BASE_SPEED_PCT     75
#define MOTOR_TRIM_PCT      6

#define KP_X100            18
#define KD_X100            68
#define KI_X100             0
#define INTEG_LIMIT      4000

#define FRONT_STOP_MM      70

/* ---- TUNABLES: turn detection ----------------------------------------- */

/* Opening predicate requires BOTH conditions, sustained OPENING_CONFIRM
 * cycles. 1 overflow ~= 32.768 ms, so 3 cycles ~= 98 ms of debounce. */
#define F_DETECT_MAX_MM     475    /* front must be <= this (corner ahead) */
#define SIDE_OPEN_MM        350    /* a side counts as open at >= this     */
#define OPENING_CONFIRM       3    /* consecutive cycles before latching   */
#define CORRECTION_ENTRY_COOLDOWN_OVFS 10  /* grace after entering CORRECTION */

/* ---- TUNABLES: turn execution ----------------------------------------- */

#define F_TURN_MM           120
#define PRE_TURN_TIMEOUT_OVFS  60
#define TURN_SPIN_PCT        40
#define TURN_SPIN_OVFS       26
#define TURN_SETTLE_OVFS     5
#define TURN_SPEED_PCT       50
#define POST_TURN_SPEED_PCT  80   /* slight slowdown from MAX -- 25% stalled the motors */
#define TURN_SETPOINT_DEFAULT_MM  225
#define POST_BOTH_WALLS_MM   400
#define POST_TURN_TIMEOUT_OVFS 90
#define POST_EXIT_CONFIRM      3  /* consecutive cycles before POST_TURN exits */
#define WALL_HOLD_TRIM_PCT     3  /* gentler trim during wall-hold (vs 6% straight) */
#define SIDE_VALID_MIN_MM      80 /* min opposite-side mm to count Exit (b) as a real corner */

/* ---- TUNABLES: scheduling ---------------------------------------------- */

#define PID_PERIOD_OVFS     1
#define PRINT_PERIOD_OVFS   8
#define PRINT_STATE_CHANGES  1
#define PRINT_PERIODIC_STATUS 0

/* ---- DERIVED ---------------------------------------------------------- */

#define PCT_TO_PWM(p)     ((int16_t)(((int32_t)(p) * 255) / 100))
#define MAX_SPEED         PCT_TO_PWM(MAX_SPEED_PCT)
#define BASE_SPEED        PCT_TO_PWM(BASE_SPEED_PCT)
#define TURN_SPEED        PCT_TO_PWM(TURN_SPEED_PCT)
#define POST_TURN_SPEED   PCT_TO_PWM(POST_TURN_SPEED_PCT)
#define TURN_SPIN_PWM     PCT_TO_PWM(TURN_SPIN_PCT)

/* ---- STATE ------------------------------------------------------------ */

typedef enum {
    ST_CORRECTION = 0,
    ST_PRE_TURN,
    ST_SPIN,
    ST_POST_TURN
} state_t;

static state_t  state           = ST_CORRECTION;
static uint16_t state_start_ovf = 0;
static char     pending_dir     = 'L';

static uint8_t  open_count_L = 0;
static uint8_t  open_count_R = 0;

static uint8_t  post_both_count     = 0;
static uint8_t  post_front_count    = 0;
static uint8_t  post_corner_L_count = 0;
static uint8_t  post_corner_R_count = 0;

static uint8_t  turns_logged = 0;

static int16_t  prev_error = 0;
static int32_t  integ      = 0;
static uint8_t  seed_prev_error = 0;   /* on next PID cycle, set prev_error = err so deriv starts at 0 */

static int16_t  last_outL    = 0;
static int16_t  last_outR    = 0;
static int16_t  last_error   = 0;
static uint8_t  front_braking = 0;

static uint16_t turn_setpoint_mm = TURN_SETPOINT_DEFAULT_MM;
static uint8_t  sample_setpoint  = 0;

/* ---- HELPERS ---------------------------------------------------------- */

static const char *state_name(state_t s);

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
    state_t old_state = state;
    state = s;
    state_start_ovf = now_ovf;
    /* Sample the followed-wall distance ONLY at PRE_TURN entry. POST_TURN
     * inherits the same setpoint so the bot recovers to the pre-turn
     * corridor distance (taken while CORRECTION had us centered) instead
     * of locking in whatever misaligned distance the spin happened to
     * land on. Corridor width is fixed (45 cm) so the pre-turn target
     * is still correct after a 90 deg turn. */
    if (s == ST_PRE_TURN) {
        sample_setpoint = 1;
    }
    if (s == ST_POST_TURN) {
        post_both_count     = 0;
        post_front_count    = 0;
        post_corner_L_count = 0;
        post_corner_R_count = 0;
    }
#if PRINT_STATE_CHANGES
    if (old_state != s) {
        UART_sendString("\r\nSTATE ");
        UART_sendString(state_name(old_state));
        UART_sendString(" -> ");
        UART_sendString(state_name(s));
        UART_sendString("\r\n");
    }
#endif
}

/* Soft reset: clears wind-up (integ) and the CORRECTION opening counters,
 * but leaves prev_error alone -- instead, a seed flag tells the next PID
 * cycle to set prev_error = current err so that deriv starts at 0. This
 * removes the spurious KD kick at state transitions (which was causing
 * "overshoot and hit the wall" right after POST_TURN -> CORRECTION). */
static void reset_pid(void) {
    integ           = 0;
    open_count_L    = 0;
    open_count_R    = 0;
    seed_prev_error = 1;
}

static void record_turn(char dir) {
    if (dir == 'L') turn_left();
    else            turn_right();
    turns_logged++;

    UART_sendString("\r\n*** TURN #");
    UART_sendInt((int)turns_logged);
    UART_sendChar(' ');
    UART_sendChar(dir);
    UART_sendString(" ***\r\n");
}

static void print_field_signed(const char *label, int16_t v) {
    UART_sendString(label);
    UART_sendInt((int)v);
    UART_sendChar(' ');
}

static void print_field_unsigned(const char *label, uint16_t v) {
    UART_sendString(label);
    UART_sendInt((int)v);
    UART_sendChar(' ');
}

static void print_dist(const char *label, uint16_t mm) {
    UART_sendString(label);
    if (mm == US_NO_READING) UART_sendString("----");
    else                     UART_sendInt((int)mm);
    UART_sendChar(' ');
}

static const char *state_name(state_t s) {
    switch (s) {
        case ST_CORRECTION: return "MOVING_FORWARD";
        case ST_PRE_TURN:   return "PRE_TURN";
        case ST_SPIN:       return "TURNING";
        case ST_POST_TURN:  return "POST_TURN";
    }
    return "???";
}

static const char *state_tag(void) {
    return state_name(state);
}

static inline uint16_t followed_wall_mm(void) {
    return (pending_dir == 'L') ? us_distance_mm(US_RIGHT)
                                : us_distance_mm(US_LEFT);
}

static void wall_hold_drive(int16_t base) {
    uint16_t L = us_distance_mm(US_LEFT);
    uint16_t R = us_distance_mm(US_RIGHT);

    /* ALWAYS substitute the open side with the sampled setpoint -- the
     * opening direction is already known (PRE_TURN was triggered by
     * confirmed opening detection; POST_TURN is locked to the turn we
     * just executed). Trusting the sensor here was the bug: right after
     * a turn the "open" side often briefly reads the inside-corner wall
     * at 80-150mm, which is < SIDE_OPEN_MM, so the old gate failed to
     * substitute and the PID computed a huge spurious error on the
     * first cycle -> aggressive whip toward the real wall.
     *
     * The followed-wall NO_READING branch sets err=0 (go straight with
     * trim) instead of leaving a stale value. */
    if (pending_dir == 'L') {
        L = turn_setpoint_mm;                                /* virtual */
        if (R == US_NO_READING) R = turn_setpoint_mm;        /* safe   */
    } else {
        R = turn_setpoint_mm;                                /* virtual */
        if (L == US_NO_READING) L = turn_setpoint_mm;        /* safe   */
    }

    int16_t error = (int16_t)R - (int16_t)L;
    if (seed_prev_error) {
        prev_error      = error;   /* smooth transition: deriv=0 on first cycle */
        seed_prev_error = 0;
    }
    int16_t deriv = error - prev_error;
    integ += error;
    if (integ >  INTEG_LIMIT) integ =  INTEG_LIMIT;
    if (integ < -INTEG_LIMIT) integ = -INTEG_LIMIT;

    int32_t turn = ((int32_t)KP_X100 * error
                  + (int32_t)KD_X100 * deriv
                  + (int32_t)KI_X100 * integ) / 100;

    int16_t turn_pwm = clamp16(turn, base);
    int16_t left  = (int16_t)((int32_t)base + turn_pwm);
    int16_t right = (int16_t)((int32_t)base - turn_pwm);

    /* Gentler trim than CORRECTION's full MOTOR_TRIM_PCT. Steady-state
     * offset under P-dominant control is ~ trim_pwm*100/Kp mm, roughly
     * independent of base speed. Iterate down further if drift remains. */
    int16_t trim_pwm = PCT_TO_PWM(WALL_HOLD_TRIM_PCT);
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

static void sample_setpoint_if_pending(void) {
    if (!sample_setpoint) return;
    uint16_t hold    = followed_wall_mm();
    turn_setpoint_mm = (hold == US_NO_READING) ? TURN_SETPOINT_DEFAULT_MM
                                                : hold;
    sample_setpoint  = 0;
}

/* ---- CORRECTION: PID + opening detection ------------------------------ */

static void correction_step(uint16_t ovf) {
    uint16_t L = us_distance_mm(US_LEFT);
    uint16_t R = us_distance_mm(US_RIGHT);
    uint16_t F = us_distance_mm(US_FRONT);

    /* Cooldown after entering CORRECTION (especially right after POST_TURN):
     * suppress opening detection for a short grace window so transient
     * sensor geometry just after a turn does not retrigger another. */
    uint8_t in_cooldown = ((uint16_t)(ovf - state_start_ovf)
                           < CORRECTION_ENTRY_COOLDOWN_OVFS);

    if (in_cooldown) {
        open_count_L = open_count_R = 0;
    } else {
        uint8_t fclose = front_is_close(F);
        uint8_t l_open = side_is_open(L);
        uint8_t r_open = side_is_open(R);

        /* Require the OPPOSITE side to be reading a real corridor wall.
         * Without this, simultaneous dropouts on both sensors look like
         * an L opening (L checked first in the tie-break) and trigger a
         * spurious turn. */
        if (fclose && l_open && !r_open) {
            if (open_count_L < 255) open_count_L++;
        } else {
            open_count_L = 0;
        }
        if (fclose && r_open && !l_open) {
            if (open_count_R < 255) open_count_R++;
        } else {
            open_count_R = 0;
        }

        if (open_count_L >= OPENING_CONFIRM || open_count_R >= OPENING_CONFIRM) {
            pending_dir   = (open_count_L >= OPENING_CONFIRM) ? 'L' : 'R';
            front_braking = 0;
            reset_pid();  /* clean derivative state before virtual-wall PID */
            enter_state(ST_PRE_TURN, ovf);
            return;
        }
    }

    if (F != US_NO_READING && F < FRONT_STOP_MM) {
        motors_stop();
        last_outL = last_outR = 0;
        front_braking = 1;
        return;
    }
    front_braking = 0;

    if (L == US_NO_READING || R == US_NO_READING) {
        int16_t crawl = BASE_SPEED / 2;
        motors_set(crawl, crawl);
        last_outL  = last_outR = crawl;
        last_error = 0;
        return;
    }

    int16_t error = (int16_t)R - (int16_t)L;
    if (seed_prev_error) {
        prev_error      = error;   /* smooth transition: deriv=0 on first cycle */
        seed_prev_error = 0;
    }
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

/* ---- PRE_TURN / SPIN / POST_TURN -------------------------------------- */

static void pre_turn_step(uint16_t ovf) {
    sample_setpoint_if_pending();

    uint16_t F = us_distance_mm(US_FRONT);

    if (F != US_NO_READING && F <= F_TURN_MM) {
        enter_state(ST_SPIN, ovf);
        return;
    }
    if ((uint16_t)(ovf - state_start_ovf) >= PRE_TURN_TIMEOUT_OVFS) {
        enter_state(ST_SPIN, ovf);
        return;
    }

    wall_hold_drive(TURN_SPEED);
}

static void spin_step(uint16_t ovf) {
    if (pending_dir == 'L') {
        motors_set(-TURN_SPIN_PWM, +TURN_SPIN_PWM);
        last_outL = -TURN_SPIN_PWM; last_outR = +TURN_SPIN_PWM;
    } else {
        motors_set(+TURN_SPIN_PWM, -TURN_SPIN_PWM);
        last_outL = +TURN_SPIN_PWM; last_outR = -TURN_SPIN_PWM;
    }
    if ((uint16_t)(ovf - state_start_ovf) >= TURN_SPIN_OVFS) {
        record_turn(pending_dir);
        reset_pid();
        enter_state(ST_POST_TURN, ovf);
    }
}

static void post_turn_step(uint16_t ovf) {
    uint16_t since_entry = (uint16_t)(ovf - state_start_ovf);

    /* Folded settle: bleed off spin inertia before wall-holding so the
     * first sensor read isn't taken mid-rotation. */
    if (since_entry < TURN_SETTLE_OVFS) {
        motors_stop();
        last_outL = last_outR = 0;
        return;
    }

    uint16_t L = us_distance_mm(US_LEFT);
    uint16_t R = us_distance_mm(US_RIGHT);
    uint16_t F = us_distance_mm(US_FRONT);

    uint8_t both_now  = (L != US_NO_READING && R != US_NO_READING
                         && L <= POST_BOTH_WALLS_MM
                         && R <= POST_BOTH_WALLS_MM);
    uint8_t front_now = (F != US_NO_READING && F <= F_TURN_MM);

    /* Per-cycle corner-geometry tests: front close AND one side open
     * AND the other side reading a real corridor wall, all together.
     * Debouncing THIS (not just front_now) stops single-cycle flickers
     * from picking the spin direction. */
    uint8_t lo = side_is_open(L);
    uint8_t ro = side_is_open(R);
    uint8_t l_wall_ok = (L != US_NO_READING
                         && L >= SIDE_VALID_MIN_MM
                         && L <  SIDE_OPEN_MM);
    uint8_t r_wall_ok = (R != US_NO_READING
                         && R >= SIDE_VALID_MIN_MM
                         && R <  SIDE_OPEN_MM);
    uint8_t L_corner_now = front_now && lo && r_wall_ok;
    uint8_t R_corner_now = front_now && ro && l_wall_ok;

    if (both_now)     { if (post_both_count     < 255) post_both_count++;     } else post_both_count     = 0;
    if (front_now)    { if (post_front_count    < 255) post_front_count++;    } else post_front_count    = 0;
    if (L_corner_now) { if (post_corner_L_count < 255) post_corner_L_count++; } else post_corner_L_count = 0;
    if (R_corner_now) { if (post_corner_R_count < 255) post_corner_R_count++; } else post_corner_R_count = 0;

    /* Priority 1 -- Exit (a): both side walls visible. Wins outright
     * over Exit (b) when both_now is currently true (a flickery side
     * reading shouldn't let a corner trigger fire while we're clearly
     * inside a corridor). */
    if (post_both_count >= POST_EXIT_CONFIRM) {
        reset_pid();
        enter_state(ST_CORRECTION, ovf);
        return;
    }

    /* Priority 2 -- Exit (b): only checked when we DON'T currently see
     * both walls. Direction is locked in by 3 cycles of consistent
     * corner geometry, not just by a noisy front reading. */
    if (!both_now) {
        if (post_corner_L_count >= POST_EXIT_CONFIRM) {
            pending_dir = 'L';
            reset_pid();
            enter_state(ST_SPIN, ovf);
            return;
        }
        if (post_corner_R_count >= POST_EXIT_CONFIRM) {
            pending_dir = 'R';
            reset_pid();
            enter_state(ST_SPIN, ovf);
            return;
        }
        /* Front debounced close but corner geometry never confirmed --
         * hand off to CORRECTION. Its FRONT_STOP brake and debounced
         * opening detector will resolve whatever is actually ahead. */
        if (post_front_count >= POST_EXIT_CONFIRM) {
            reset_pid();
            enter_state(ST_CORRECTION, ovf);
            return;
        }
    }

    if (since_entry >= POST_TURN_TIMEOUT_OVFS) {
        reset_pid();
        enter_state(ST_CORRECTION, ovf);
        return;
    }

    wall_hold_drive(POST_TURN_SPEED);
}

/* ---- ENTRY POINTS ----------------------------------------------------- */

void setup(void) {
    LED_DDR  |=  (1 << LED_BIT);
    LED_PORT &= ~(1 << LED_BIT);

    UART_init(9600);
    us_init();
    motors_init();
    turns_init();
    sei();

    UART_sendString("\r\nWallBot: PID + sensor-gated 90 deg turn\r\n");
    UART_sendString("MAX="); UART_sendInt(MAX_SPEED_PCT);
    UART_sendString("%  BASE="); UART_sendInt(BASE_SPEED_PCT);
    UART_sendString("%  TRIM="); UART_sendInt(MOTOR_TRIM_PCT);
    UART_sendString("%  Kp*100="); UART_sendInt(KP_X100);
    UART_sendString(" Kd*100=");  UART_sendInt(KD_X100);
    UART_sendString(" Ki*100=");  UART_sendInt(KI_X100);
    UART_sendString("\r\nF_DET<="); UART_sendInt(F_DETECT_MAX_MM);
    UART_sendString(" SIDE>=");    UART_sendInt(SIDE_OPEN_MM);
    UART_sendString(" CONFIRM=");  UART_sendInt(OPENING_CONFIRM);
    UART_sendString(" F_TURN<=");  UART_sendInt(F_TURN_MM);
    UART_sendString(" SPIN=");     UART_sendInt(TURN_SPIN_OVFS);
    UART_sendString("@");          UART_sendInt(TURN_SPIN_PCT);
    UART_sendString("%  POST_BOTH<=");  UART_sendInt(POST_BOTH_WALLS_MM);
    UART_sendString("\r\n");

    /* ---- Press-to-start ---- */
    // UART_sendString("Send 's' to start...\r\n");
    // char ch;
    // while (1) {
    //     if (UART_receiveChar(&ch) && ch == 's') break;
    // }
    // UART_sendString("Starting!\r\n");
    /* ------------------------ */

    motors_enable(1);
}

void loop(void) {
    static uint16_t last_pid_ovf   = 0;
#if PRINT_PERIODIC_STATUS
    static uint16_t last_print_ovf = 0;
#endif

    us_service();

    uint16_t ovf = us_overflow_count();

    if ((uint16_t)(ovf - last_pid_ovf) >= PID_PERIOD_OVFS) {
        last_pid_ovf = ovf;
        switch (state) {
            case ST_CORRECTION: correction_step(ovf); break;
            case ST_PRE_TURN:   pre_turn_step(ovf);   break;
            case ST_SPIN:       spin_step(ovf);        break;
            case ST_POST_TURN:  post_turn_step(ovf);   break;
        }

        /* TODO: add end-of-track detection condition here.
         * When the track is finished, call sendReport() to transmit
         * the full turn summary over Bluetooth/UART, then stop motors.
         *
         * if (end_of_track_detected()) {
         *     motors_stop();
         *     sendReport();
         *     while (1);
         * }
         */
    }

#if PRINT_PERIODIC_STATUS
    if ((uint16_t)(ovf - last_print_ovf) >= PRINT_PERIOD_OVFS) {
        last_print_ovf = ovf;
        UART_sendChar('['); UART_sendString(state_tag()); UART_sendString("] ");
        print_dist("L=",   us_distance_mm(US_LEFT));
        print_dist("R=",   us_distance_mm(US_RIGHT));
        print_dist("F=",   us_distance_mm(US_FRONT));
        print_field_signed  ("err=", last_error);
        print_field_signed  ("oL=",  last_outL);
        print_field_signed  ("oR=",  last_outR);
        print_field_unsigned("T=",   (uint16_t)turns_logged);
        if (front_braking) UART_sendString(" [STOP]");
        UART_sendString("\r\n");
        LED_PORT ^= (1 << LED_BIT);
    }
#endif
}
