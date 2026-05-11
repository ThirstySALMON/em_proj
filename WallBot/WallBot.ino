
#include <avr/io.h>
#include <avr/interrupt.h>

#include "pinmap.h"
#include "uart.h"
#include "ultrasonic.h"
#include "motors.h"
#include "turn_tracker.h"
#include "completion_detector.h"

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

// ---- TUNABLES: motion ----------------------------- 

#define MAX_SPEED_PCT     100
#define BASE_SPEED_PCT     75
#define MOTOR_TRIM_PCT      6 // bcz of motor imbalance

// pid constants
#define KP_X100            18
#define KD_X100            68
#define KI_X100             0
#define INTEG_LIMIT      4000 // prevent integral from growing infinitely

#define FRONT_STOP_MM      70 // emergency stop (during correction)

// ---- TUNABLES: turn detection ---------------------
 // (during pre turn)
// Opening requires BOTH conditions
#define F_DETECT_MAX_MM     475    // front must be <=  (corner ahead) 
#define SIDE_OPEN_MM        380    // a side counts as open at >= this     

#define OPENING_CONFIRM       3    // consecutive cycles before committing   
#define CORRECTION_ENTRY_COOLDOWN_OVFS 10  // after a turn - ignore openings for this many cycles as they might be faulty

// ---- TUNABLES: turn execution ----------------------------------------- 

#define F_TURN_MM           120 // when to turn
#define PRE_TURN_TIMEOUT_OVFS  90 // timeout for when to turn ()
#define TURN_SPIN_PCT_L      45  // speed while spinning left
#define TURN_SPIN_PCT_R      39  // speed while spinning right
#define TURN_SPIN_OVFS       28 // timer for spinning
#define TURN_SETTLE_OVFS     5 // timer for settling after spin (cuz of inertia)
#define TURN_SPEED_PCT       50 // speed in pre-turn
#define POST_TURN_SPEED_PCT  80   // speed in post turn
#define TURN_SETPOINT_DEFAULT_MM  225  // default one wall setpoint if we fail to sample (bcz of no-reading or simultaneous open)
#define POST_BOTH_WALLS_MM   400 // distance to consider walls visible
#define POST_TURN_TIMEOUT_OVFS 90 // timeout 
#define POST_EXIT_CONFIRM      3  // consecutive cycles before POST_TURN exits
#define WALL_HOLD_TRIM_PCT     3  // gentler trim during wall-hold 
#define SIDE_VALID_MIN_MM      50 // min opposite-side mm to count exit (b) as a real corner 

// ---- TUNABLES: scheduling ---------------------------------------------- 

#define PID_PERIOD_OVFS     1 // overflows between PID cycles.
#define PRINT_PERIOD_OVFS   8
#define PRINT_STATE_CHANGES  1
#define PRINT_PERIODIC_STATUS 0

// ---- TUNABLES: run completion detector -------------------------------- 

// all three sensors open continuously for this many Timer1 overflows
#define COMPLETE_OPEN_MM          350 // what counts as open space
#define COMPLETE_CONFIRM_OVFS      62 // how long to remain open (2.5 secs rn)
#define COMPLETE_NO_READING_OPEN    1 // sets if no echo = open 1-> yes

// ---- TUNABLES: reverse recovery --------------------------------------- 

// Collision recovery is independent from navigation. Trigger only after
// both motors have been commanded stopped continuously. Then reverse at
// about 70% of normal speed until a valid turn or forward path is found. 
#define RECOVERY_STOPPED_CONFIRM_OVFS 76  // ~2.49 s
#define RECOVERY_IMPACT_FRONT_MM      30  // 2-3 cm: car is about to hit
#define RECOVERY_IMPACT_CONFIRM_OVFS   2  // short debounce for ultrasonic noise
#define RECOVERY_REVERSE_SPEED_PCT    55  // ~70% of BASE_SPEED_PCT
#define RECOVERY_MIN_REVERSE_OVFS      8  // ~262 ms before exit checks
#define RECOVERY_MAX_REVERSE_OVFS    120  // ~3.93 s safety fallback
#define RECOVERY_EXIT_CONFIRM_OVFS     3
#define RECOVERY_FRONT_CLEAR_MM      160
#define RECOVERY_COOLDOWN_OVFS       15

// ---- DERIVED ---------------------------------------------------------- 

#define PCT_TO_PWM(p)     ((int16_t)(((int32_t)(p) * 255) / 100))
#define MAX_SPEED         PCT_TO_PWM(MAX_SPEED_PCT)
#define BASE_SPEED        PCT_TO_PWM(BASE_SPEED_PCT)
#define TURN_SPEED        PCT_TO_PWM(TURN_SPEED_PCT)
#define POST_TURN_SPEED   PCT_TO_PWM(POST_TURN_SPEED_PCT)
#define TURN_SPIN_PWM_L   PCT_TO_PWM(TURN_SPIN_PCT_L)
#define TURN_SPIN_PWM_R   PCT_TO_PWM(TURN_SPIN_PCT_R)
#define RECOVERY_REVERSE_SPEED PCT_TO_PWM(RECOVERY_REVERSE_SPEED_PCT)

// ---- STATE ------------------------------------------------------------ 

typedef enum {
    ST_CORRECTION = 0,
    ST_PRE_TURN,
    ST_SPIN,
    ST_POST_TURN,
    ST_REVERSE_RECOVERY,
    ST_COMPLETE
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
static uint8_t  seed_prev_error = 0;   // on next PID cycle, set prev_error = err so deriv starts at 0

static int16_t  last_outL    = 0;
static int16_t  last_outR    = 0;
static int16_t  last_error   = 0;
static uint8_t  front_braking = 0;

static uint16_t turn_setpoint_mm = TURN_SETPOINT_DEFAULT_MM;
static uint8_t  sample_setpoint  = 0;
static uint8_t  completion_report_sent = 0;

static uint8_t  recovery_stopped_count = 0;
static uint8_t  recovery_impact_count = 0;
static uint8_t  recovery_clear_count = 0;
static uint8_t  recovery_corner_L_count = 0;
static uint8_t  recovery_corner_R_count = 0;
static uint8_t  recovery_cooldown_active = 0;
static uint16_t recovery_cooldown_start_ovf = 0;

static completion_detector_t completion_detector;
static const completion_detector_config_t completion_config = {
    COMPLETE_OPEN_MM,
    COMPLETE_CONFIRM_OVFS,
    COMPLETE_NO_READING_OPEN
};

// ---- HELPERS ---------------------------------------------------------- 

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
    state_start_ovf = now_ovf; // when did we enter this state

    // Sample the followed-wall distance ONLY at PRE_TURN entry. POST_TURN
    // inherits the same setpoint so the bot recovers to the pre-turn
    // corridor distance (taken while CORRECTION had us centered) instead
    // of locking in whatever misaligned distance the spin happened to
    // land on. Corridor width is fixed (45 cm) so the pre-turn target
    // is still correct after a 90 deg turn. 
    if (s == ST_PRE_TURN) {
        sample_setpoint = 1; // when entering PRE_TURN, sample the followed-wall distance
    }
    if (s == ST_POST_TURN) {
        // reset those to 0.. used to know which exit condition was hit.
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
        case ST_REVERSE_RECOVERY: return "REVERSING_RECOVERY";
        case ST_COMPLETE:   return "COMPLETE";
    }
    return "???";
}

static const char *state_tag(void) {
    return state_name(state);
}

// depending on which wall im following rn -> get its distance from ultrasonic
static inline uint16_t followed_wall_mm(void) {
    return (pending_dir == 'L') ? us_distance_mm(US_RIGHT)
                                : us_distance_mm(US_LEFT);
}

// pid drive straight looping while holding the wall at a certain distance. Used in PRE_TURN and POST_TURN.
static void wall_hold_drive(int16_t base) {

    // read both side sensors (raw values)
    uint16_t L = us_distance_mm(US_LEFT);
    uint16_t R = us_distance_mm(US_RIGHT);

    // if we are preparing a LEFT turn, left side becomes "virtual"
    // we replace it with the stored corridor distance so PID doesn't freak out
    if (pending_dir == 'L') {
        L = turn_setpoint_mm;                                // fake left wall distance

        // if right sensor is missing, also replace it so we don't get garbage values
        if (R == US_NO_READING) R = turn_setpoint_mm;        // safety fallback
    } 
    else {
        // same idea but mirrored for RIGHT turn
        R = turn_setpoint_mm;                                // fake right wall distance

        if (L == US_NO_READING) L = turn_setpoint_mm;        // safety fallback
    }

    // compute how off-center we are (right minus left)
    int16_t error = (int16_t)R - (int16_t)L;

    // first cycle after state change: prevent derivative spike
    if (seed_prev_error) {
        prev_error      = error;   // lock previous error to current
        seed_prev_error = 0;       // only do this once
    }

    // how fast the error is changing
    int16_t deriv = error - prev_error;

    // accumulate long-term drift
    integ += error;

    // clamp integral so it doesn't run away
    if (integ >  INTEG_LIMIT) integ =  INTEG_LIMIT;
    if (integ < -INTEG_LIMIT) integ = -INTEG_LIMIT;

    // PID output (scaled down by /100 because gains are *100)
    int32_t turn = ((int32_t)KP_X100 * error
                  + (int32_t)KD_X100 * deriv
                  + (int32_t)KI_X100 * integ) / 100;

    // limit how strong correction can be (based on base speed)
    int16_t turn_pwm = clamp16(turn, base);

    // apply correction to motors
    int16_t left  = (int16_t)((int32_t)base + turn_pwm);
    int16_t right = (int16_t)((int32_t)base - turn_pwm);

    // small fixed trim to cancel motor imbalance
    int16_t trim_pwm = PCT_TO_PWM(WALL_HOLD_TRIM_PCT);
    left  += trim_pwm;
    right -= trim_pwm;

    // make sure values stay in valid PWM range
    if (left  < 0)         left  = 0;
    if (right < 0)         right = 0;
    if (left  > MAX_SPEED) left  = MAX_SPEED;
    if (right > MAX_SPEED) right = MAX_SPEED;

    // send final motor commands
    motors_set(left, right);

    // save for next loop iteration
    prev_error = error;
    last_outL  = left;
    last_outR  = right;
    last_error = error;
}

// When entering PRE_TURN, sample the distance to the followed wall
static void sample_setpoint_if_pending(void) {
    if (!sample_setpoint) return;
    uint16_t hold    = followed_wall_mm();
    turn_setpoint_mm = (hold == US_NO_READING) ? TURN_SETPOINT_DEFAULT_MM
                                                : hold;
    sample_setpoint  = 0;
}

// ---- CORRECTION: PID + opening detection ------------------------------ 

static void correction_step(uint16_t ovf) {
    uint16_t L = us_distance_mm(US_LEFT);
    uint16_t R = us_distance_mm(US_RIGHT);
    uint16_t F = us_distance_mm(US_FRONT);

    // ignore turn noise right after entering CORRECTION
    uint8_t in_cooldown = ((uint16_t)(ovf - state_start_ovf)
                           < CORRECTION_ENTRY_COOLDOWN_OVFS);

    if (in_cooldown) {
        open_count_L = open_count_R = 0;
    } else {
        uint8_t fclose = front_is_close(F);
        uint8_t l_open = side_is_open(L);
        uint8_t r_open = side_is_open(R);

        // detect left opening
        if (fclose && l_open && !r_open) {
            if (open_count_L < 255) open_count_L++;
        } else {
            open_count_L = 0;
        }

        // detect right opening
        if (fclose && r_open && !l_open) {
            if (open_count_R < 255) open_count_R++;
        } else {
            open_count_R = 0;
        }

        // trigger turn if stable detection
        if (open_count_L >= OPENING_CONFIRM || open_count_R >= OPENING_CONFIRM) {
            pending_dir   = (open_count_L >= OPENING_CONFIRM) ? 'L' : 'R';
            front_braking = 0;
            reset_pid();
            enter_state(ST_PRE_TURN, ovf);
            return;
        }
    }

    // stop if front is too close
    if (F != US_NO_READING && F < FRONT_STOP_MM) {
        motors_stop();
        last_outL = last_outR = 0;
        front_braking = 1;
        return;
    }
    front_braking = 0;

    // crawl forward if both sensors are missing
    if (L == US_NO_READING && R == US_NO_READING) {
        int16_t crawl = BASE_SPEED / 2;
        motors_set(crawl, crawl);
        last_outL  = last_outR = crawl;
        last_error = 0;
        return;
    }

    // replace open side with reference distance
    if (L == US_NO_READING || L >= SIDE_OPEN_MM) L = turn_setpoint_mm;
    if (R == US_NO_READING || R >= SIDE_OPEN_MM) R = turn_setpoint_mm;

    // compute center error
    int16_t error = (int16_t)R - (int16_t)L;

    // avoid derivative spike on first cycle
    if (seed_prev_error) {
        prev_error      = error;
        seed_prev_error = 0;
    }

    // derivative term
    int16_t deriv = error - prev_error;

    // integral term
    integ += error;
    if (integ >  INTEG_LIMIT) integ =  INTEG_LIMIT;
    if (integ < -INTEG_LIMIT) integ = -INTEG_LIMIT;

    // PID output
    int32_t turn = ((int32_t)KP_X100 * error
                  + (int32_t)KD_X100 * deriv
                  + (int32_t)KI_X100 * integ) / 100;

    // convert to motor speeds
    int16_t turn_pwm = clamp16(turn, BASE_SPEED);
    int16_t left  = (int16_t)((int32_t)BASE_SPEED + turn_pwm);
    int16_t right = (int16_t)((int32_t)BASE_SPEED - turn_pwm);

    // motor imbalance correction
    int16_t trim_pwm = PCT_TO_PWM(MOTOR_TRIM_PCT);
    left  += trim_pwm;
    right -= trim_pwm;

    // clamp outputs
    if (left  < 0)         left  = 0;
    if (right < 0)         right = 0;
    if (left  > MAX_SPEED) left  = MAX_SPEED;
    if (right > MAX_SPEED) right = MAX_SPEED;

    // send to motors
    motors_set(left, right);

    // store state
    prev_error = error;
    last_outL  = left;
    last_outR  = right;
    last_error = error;
}

// ---- PRE_TURN / SPIN / POST_TURN -------------------------------------- 

static void pre_turn_step(uint16_t ovf) {

    // capture corridor reference if it hasn’t been done yet
    sample_setpoint_if_pending();

    // read front distance
    uint16_t F = us_distance_mm(US_FRONT);

    // if front wall is close, start turning
    if (F != US_NO_READING && F <= F_TURN_MM) {
        enter_state(ST_SPIN, ovf);
        return;
    }

    // fallback: if we waited too long, force turn anyway
    if ((uint16_t)(ovf - state_start_ovf) >= PRE_TURN_TIMEOUT_OVFS) {
        enter_state(ST_SPIN, ovf);
        return;
    }

    // keep moving forward while holding wall alignment
    wall_hold_drive(TURN_SPEED);
}

static void spin_step(uint16_t ovf) {
    uint16_t elapsed = (uint16_t)(ovf - state_start_ovf);

    // phase 1: active rotation. 
    if (elapsed < TURN_SPIN_OVFS) {
        if (pending_dir == 'L') {
            motors_set(-TURN_SPIN_PWM_L, +TURN_SPIN_PWM_L);
            last_outL = -TURN_SPIN_PWM_L; last_outR = +TURN_SPIN_PWM_L;
        } else {
            motors_set(+TURN_SPIN_PWM_R, -TURN_SPIN_PWM_R);
            last_outL = +TURN_SPIN_PWM_R; last_outR = -TURN_SPIN_PWM_R;
        }
        return;
    }

    // phase 2: stop and wait for inertia to stop
    if (elapsed < (uint16_t)(TURN_SPIN_OVFS + TURN_SETTLE_OVFS)) {
        motors_stop();
        last_outL = last_outR = 0;
        return;
    }

    // phase 3: hand off. POST_TURN is bypassed -- CORRECTION's
    // per-cycle virtual-wall substitution handles the asymmetric
    // post-turn geometry without a dedicated state. 
    record_turn(pending_dir);
    reset_pid();
    enter_state(ST_CORRECTION, ovf);
}

// DEPRECATED
// static void post_turn_step(uint16_t ovf) {
//     uint16_t since_entry = (uint16_t)(ovf - state_start_ovf);

//     // Folded settle: bleed off spin inertia before wall-holding so the
//     // first sensor read isn't taken mid-rotation. 
//     if (since_entry < TURN_SETTLE_OVFS) {
//         motors_stop();
//         last_outL = last_outR = 0;
//         return;
//     }

//     uint16_t L = us_distance_mm(US_LEFT);
//     uint16_t R = us_distance_mm(US_RIGHT);
//     uint16_t F = us_distance_mm(US_FRONT);

//     uint8_t both_now  = (L != US_NO_READING && R != US_NO_READING
//                          && L <= POST_BOTH_WALLS_MM
//                          && R <= POST_BOTH_WALLS_MM);
//     uint8_t front_now = (F != US_NO_READING && F <= F_TURN_MM);

//     // Per-cycle corner-geometry tests: front close AND one side open
//     // AND the other side reading a real corridor wall, all together.
//     // Debouncing THIS (not just front_now) stops single-cycle flickers
//     // from picking the spin direction. 
//     uint8_t lo = side_is_open(L);
//     uint8_t ro = side_is_open(R);
//     uint8_t l_wall_ok = (L != US_NO_READING
//                          && L >= SIDE_VALID_MIN_MM
//                          && L <  SIDE_OPEN_MM);
//     uint8_t r_wall_ok = (R != US_NO_READING
//                          && R >= SIDE_VALID_MIN_MM
//                          && R <  SIDE_OPEN_MM);
//     uint8_t L_corner_now = front_now && lo && r_wall_ok;
//     uint8_t R_corner_now = front_now && ro && l_wall_ok;

//     if (both_now)     { if (post_both_count     < 255) post_both_count++;     } else post_both_count     = 0;
//     if (front_now)    { if (post_front_count    < 255) post_front_count++;    } else post_front_count    = 0;
//     if (L_corner_now) { if (post_corner_L_count < 255) post_corner_L_count++; } else post_corner_L_count = 0;
//     if (R_corner_now) { if (post_corner_R_count < 255) post_corner_R_count++; } else post_corner_R_count = 0;

//     // Priority 1 -- Exit (a): both side walls visible. Wins outright
//     // over Exit (b) when both_now is currently true (a flickery side
//     // reading shouldn't let a corner trigger fire while we're clearly
//     // inside a corridor). 
//     if (post_both_count >= POST_EXIT_CONFIRM) {
//         reset_pid();
//         enter_state(ST_CORRECTION, ovf);
//         return;
//     }

//     // Priority 2 -- Exit (b): only checked when we DON'T currently see
//     // both walls. Direction is locked in by 3 cycles of consistent
//     // corner geometry, not just by a noisy front reading. 
//     if (!both_now) {
//         if (post_corner_L_count >= POST_EXIT_CONFIRM) {
//             pending_dir = 'L';
//             reset_pid();
//             enter_state(ST_SPIN, ovf);
//             return;
//         }
//         if (post_corner_R_count >= POST_EXIT_CONFIRM) {
//             pending_dir = 'R';
//             reset_pid();
//             enter_state(ST_SPIN, ovf);
//             return;
//         }
//         // Front debounced close but corner geometry never confirmed --
//         // hand off to CORRECTION. Its FRONT_STOP brake and debounced
//         // opening detector will resolve whatever is actually ahead. 
//         if (post_front_count >= POST_EXIT_CONFIRM) {
//             reset_pid();
//             enter_state(ST_CORRECTION, ovf);
//             return;
//         }
//     }

//     if (since_entry >= POST_TURN_TIMEOUT_OVFS) {
//         reset_pid();
//         enter_state(ST_CORRECTION, ovf);
//         return;
//     }

//     wall_hold_drive(POST_TURN_SPEED);
// }

// ---- REVERSE RECOVERY ------------------------------------------------- 

static uint8_t recovery_trigger_allowed(state_t s) {
    return (s == ST_CORRECTION || s == ST_PRE_TURN || s == ST_POST_TURN);
}

static void recovery_reset_exit_counts(void) {
    recovery_clear_count = 0;
    recovery_corner_L_count = 0;
    recovery_corner_R_count = 0;
}

static void enter_reverse_recovery(uint16_t ovf) {
    recovery_stopped_count = 0;
    recovery_impact_count = 0;
    recovery_reset_exit_counts();
    front_braking = 0;
    reset_pid();
    enter_state(ST_REVERSE_RECOVERY, ovf);
}

static void recovery_monitor_update(uint16_t ovf) {
    if (!recovery_trigger_allowed(state)) {
        recovery_stopped_count = 0;
        recovery_impact_count = 0;
        return;
    }

    if (recovery_cooldown_active) {
        if ((uint16_t)(ovf - recovery_cooldown_start_ovf)
            < RECOVERY_COOLDOWN_OVFS) {
            recovery_stopped_count = 0;
            recovery_impact_count = 0;
            return;
        }
        recovery_cooldown_active = 0;
    }

    uint16_t F = us_distance_mm(US_FRONT);
    if (F != US_NO_READING && F <= RECOVERY_IMPACT_FRONT_MM) {
        if (recovery_impact_count < 255) recovery_impact_count++;
    } else {
        recovery_impact_count = 0;
    }

    if (last_outL == 0 && last_outR == 0) {
        if (recovery_stopped_count < 255) recovery_stopped_count++;
    } else {
        recovery_stopped_count = 0;
    }

    if (recovery_impact_count >= RECOVERY_IMPACT_CONFIRM_OVFS
        || recovery_stopped_count >= RECOVERY_STOPPED_CONFIRM_OVFS) {
        enter_reverse_recovery(ovf);
    }
}

static void reverse_recovery_step(uint16_t ovf) {
    uint16_t elapsed = (uint16_t)(ovf - state_start_ovf);

    uint16_t L = us_distance_mm(US_LEFT);
    uint16_t R = us_distance_mm(US_RIGHT);
    uint16_t F = us_distance_mm(US_FRONT);

    if (elapsed >= RECOVERY_MIN_REVERSE_OVFS) {
        uint8_t front_clear = (F == US_NO_READING || F >= RECOVERY_FRONT_CLEAR_MM);
        uint8_t l_open = side_is_open(L);
        uint8_t r_open = side_is_open(R);
        uint8_t l_wall_ok = (L != US_NO_READING
                             && L >= SIDE_VALID_MIN_MM
                             && L <  SIDE_OPEN_MM);
        uint8_t r_wall_ok = (R != US_NO_READING
                             && R >= SIDE_VALID_MIN_MM
                             && R <  SIDE_OPEN_MM);
        uint8_t wall_for_correction = l_wall_ok || r_wall_ok;
        uint8_t L_corner_now = front_is_close(F) && l_open && r_wall_ok;
        uint8_t R_corner_now = front_is_close(F) && r_open && l_wall_ok;

        if (front_clear && wall_for_correction) {
            if (recovery_clear_count < 255) recovery_clear_count++;
        } else {
            recovery_clear_count = 0;
        }
        if (L_corner_now) {
            if (recovery_corner_L_count < 255) recovery_corner_L_count++;
        } else {
            recovery_corner_L_count = 0;
        }
        if (R_corner_now) {
            if (recovery_corner_R_count < 255) recovery_corner_R_count++;
        } else {
            recovery_corner_R_count = 0;
        }

        if (recovery_corner_L_count >= RECOVERY_EXIT_CONFIRM_OVFS) {
            pending_dir = 'L';
            recovery_cooldown_active = 1;
            recovery_cooldown_start_ovf = ovf;
            recovery_reset_exit_counts();
            motors_stop();
            last_outL = last_outR = 0;
            reset_pid();
            enter_state(ST_PRE_TURN, ovf);
            return;
        }
        if (recovery_corner_R_count >= RECOVERY_EXIT_CONFIRM_OVFS) {
            pending_dir = 'R';
            recovery_cooldown_active = 1;
            recovery_cooldown_start_ovf = ovf;
            recovery_reset_exit_counts();
            motors_stop();
            last_outL = last_outR = 0;
            reset_pid();
            enter_state(ST_PRE_TURN, ovf);
            return;
        }
        if (recovery_clear_count >= RECOVERY_EXIT_CONFIRM_OVFS
            || elapsed >= RECOVERY_MAX_REVERSE_OVFS) {
            recovery_cooldown_active = 1;
            recovery_cooldown_start_ovf = ovf;
            recovery_reset_exit_counts();
            motors_stop();
            last_outL = last_outR = 0;
            reset_pid();
            enter_state(ST_CORRECTION, ovf);
            return;
        }
    }

    motors_set(-RECOVERY_REVERSE_SPEED, -RECOVERY_REVERSE_SPEED);
    last_outL = last_outR = -RECOVERY_REVERSE_SPEED;
    last_error = 0;
}

// ---- COMPLETE --------------------------------------------------------- 

static void complete_step(void) {
    motors_stop();
    motors_enable(0);
    last_outL = last_outR = 0;

    if (!completion_report_sent) {
        completion_report_sent = 1;
        UART_sendString("\r\nRUN COMPLETE\r\n");
        sendReport();
    }
}

// ---- ENTRY POINTS ----------------------------------------------------- 

void setup(void) {
    LED_DDR  |=  (1 << LED_BIT);
    LED_PORT &= ~(1 << LED_BIT);

    // all initializations 
    UART_init(9600);
    us_init();
    motors_init();
    turns_init();
    completion_detector_init(&completion_detector, &completion_config);
    sei();

    // // ---- Press-to-start ---- 
    //  UART_sendString("Send 's' to start...\r\n");
    //  char ch;
    //  while (1) {
    //      if (UART_receiveChar(&ch) && ch == 's') break;
    //  }
    //  UART_sendString("Starting!\r\n");
    // // ------------------------ 

    motors_enable(1);
}

void loop(void) {

    // timers for periodic PID + debug printing
    static uint16_t last_pid_ovf   = 0;
#if PRINT_PERIODIC_STATUS
    static uint16_t last_print_ovf = 0;
#endif

    // update ultrasonic sensors
    us_service();

    // current time base
    uint16_t ovf = us_overflow_count();

    // run control loop at fixed interval
    if ((uint16_t)(ovf - last_pid_ovf) >= PID_PERIOD_OVFS) {
        last_pid_ovf = ovf;

        // check if maze complete is complete
        if (state != ST_COMPLETE
            && completion_detector_update(&completion_detector,
                                          us_distance_mm(US_FRONT),
                                          us_distance_mm(US_LEFT),
                                          us_distance_mm(US_RIGHT),
                                          ovf)) {
            enter_state(ST_COMPLETE, ovf);
        }

        // check for stuck / collision recovery
        recovery_monitor_update(ovf);

        // run current state logic
        switch (state) {
            case ST_CORRECTION: correction_step(ovf); break;
            case ST_PRE_TURN:   pre_turn_step(ovf);   break;
            case ST_SPIN:       spin_step(ovf);        break;
            // case ST_POST_TURN:  post_turn_step(ovf);   break; 
            case ST_REVERSE_RECOVERY: reverse_recovery_step(ovf); break;
            case ST_COMPLETE:   complete_step();       break;
        }
    }
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
