/*
 * WallBot — Step 5: PID centering + 90° turn detection & execute.
 * (updated to use new UART_ API)
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

/* ---- TUNABLES: motion ------------------------------------------------- */

#define MAX_SPEED_PCT     100
#define BASE_SPEED_PCT     75
#define MOTOR_TRIM_PCT      6

#define KP_X100            18
#define KD_X100            68
#define KI_X100             0
#define INTEG_LIMIT      4000

#define FRONT_STOP_MM     120

/* ---- TUNABLES: turn detection & execute ------------------------------- */

#define OPENING_MM           300
#define OPENING_CONFIRM      3

#define TURN_ENTRY_OVFS     12
#define TURN_SPIN_OVFS      13
#define TURN_SETTLE_OVFS     3
#define TURN_SPIN_PCT       60

#define TURN_SEQ_MAX        32

/* ---- TUNABLES: scheduling --------------------------------------------- */

#define PID_PERIOD_OVFS     1
#define PRINT_PERIOD_OVFS   8

/* ---- DERIVED ---------------------------------------------------------- */

#define PCT_TO_PWM(p)     ((int16_t)(((int32_t)(p) * 255) / 100))
#define MAX_SPEED         PCT_TO_PWM(MAX_SPEED_PCT)
#define BASE_SPEED        PCT_TO_PWM(BASE_SPEED_PCT)
#define TURN_SPIN_PWM     PCT_TO_PWM(TURN_SPIN_PCT)

/* ---- STATE ------------------------------------------------------------ */

typedef enum { ST_FOLLOW = 0, ST_TURN_ENTRY, ST_TURN_SPIN, ST_TURN_SETTLE } state_t;

static state_t  state           = ST_FOLLOW;
static uint16_t state_start_ovf = 0;
static char     pending_dir     = 'L';

static uint8_t  open_count_L   = 0;
static uint8_t  open_count_R   = 0;

static uint8_t  turn_count     = 0;
static char     turn_seq[TURN_SEQ_MAX];

static int16_t  prev_error     = 0;
static int32_t  integ          = 0;

static int16_t  last_outL      = 0;
static int16_t  last_outR      = 0;
static int16_t  last_error     = 0;
static uint8_t  front_braking  = 0;

/* ---- HELPERS ---------------------------------------------------------- */

static inline int16_t clamp16(int32_t v, int16_t lim) {
    if (v >  lim) return  lim;
    if (v < -lim) return -lim;
    return (int16_t)v;
}

static inline uint8_t is_open(uint16_t mm) {
    return (mm == US_NO_READING) || (mm > OPENING_MM);
}

static void enter_state(state_t s, uint16_t now_ovf) {
    state           = s;
    state_start_ovf = now_ovf;
}

static void reset_pid(void) {
    prev_error   = 0;
    integ        = 0;
    open_count_L = open_count_R = 0;
}

static void record_turn(char dir) {
    if (turn_count < TURN_SEQ_MAX) turn_seq[turn_count] = dir;
    turn_count++;
    UART_sendString("\r\n*** TURN #");
    UART_sendInt(turn_count);
    UART_sendChar(' ');
    UART_sendChar(dir);
    UART_sendString(" ***\r\n");
}

/* Print a labelled integer field followed by a space. */
static void print_field(const char *label, int16_t v) {
    UART_sendString(label);
    UART_sendInt(v);
    UART_sendChar(' ');
}

/* Print a labelled distance; shows "----" for US_NO_READING. */
static void print_dist(const char *label, uint16_t mm) {
    UART_sendString(label);
    if (mm == US_NO_READING) UART_sendString("----");
    else                     UART_sendInt((int)mm);
    UART_sendChar(' ');
}

static const char *state_tag(void) {
    switch (state) {
        case ST_FOLLOW:      return "FOL";
        case ST_TURN_ENTRY:  return "ENT";
        case ST_TURN_SPIN:   return "SPN";
        case ST_TURN_SETTLE: return "SET";
    }
    return "???";
}

/* ---- FOLLOW: PID + opening detection ---------------------------------- */

static void follow_step(uint16_t ovf) {
    uint16_t L = us_distance_mm(US_LEFT);
    uint16_t R = us_distance_mm(US_RIGHT);
    uint16_t F = us_distance_mm(US_FRONT);

    if (is_open(L)) { if (open_count_L < 255) open_count_L++; } else open_count_L = 0;
    if (is_open(R)) { if (open_count_R < 255) open_count_R++; } else open_count_R = 0;

    if (open_count_L >= OPENING_CONFIRM || open_count_R >= OPENING_CONFIRM) {
        pending_dir   = (open_count_L >= OPENING_CONFIRM) ? 'L' : 'R';
        front_braking = 0;
        enter_state(ST_TURN_ENTRY, ovf);
        return;
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
        last_outL = last_outR = crawl;
        last_error = 0;
        return;
    }

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

/* ---- TURN: drive forward, spin, settle -------------------------------- */

static void turn_step(uint16_t ovf) {
    uint16_t elapsed = ovf - state_start_ovf;

    switch (state) {
        case ST_TURN_ENTRY: {
            int16_t trim = PCT_TO_PWM(MOTOR_TRIM_PCT);
            int16_t l = clamp16((int32_t)BASE_SPEED + trim, MAX_SPEED);
            int16_t r = clamp16((int32_t)BASE_SPEED - trim, MAX_SPEED);
            if (l < 0) l = 0;
            if (r < 0) r = 0;
            motors_set(l, r);
            last_outL = l;
            last_outR = r;
            if (elapsed >= TURN_ENTRY_OVFS) enter_state(ST_TURN_SPIN, ovf);
            break;
        }
        case ST_TURN_SPIN:
            if (pending_dir == 'L') {
                motors_set(-TURN_SPIN_PWM, +TURN_SPIN_PWM);
                last_outL = -TURN_SPIN_PWM; last_outR = +TURN_SPIN_PWM;
            } else {
                motors_set(+TURN_SPIN_PWM, -TURN_SPIN_PWM);
                last_outL = +TURN_SPIN_PWM; last_outR = -TURN_SPIN_PWM;
            }
            if (elapsed >= TURN_SPIN_OVFS) enter_state(ST_TURN_SETTLE, ovf);
            break;

        case ST_TURN_SETTLE:
            motors_stop();
            last_outL = last_outR = 0;
            if (elapsed >= TURN_SETTLE_OVFS) {
                record_turn(pending_dir);
                reset_pid();
                enter_state(ST_FOLLOW, ovf);
            }
            break;

        default: break;
    }
}

/* ---- ENTRY POINTS ----------------------------------------------------- */

void setup(void) {
    LED_DDR  |=  (1 << LED_BIT);
    LED_PORT &= ~(1 << LED_BIT);

    UART_init(9600);
    us_init();
    motors_init();
    sei();

    UART_sendString("\r\nWallBot: PID + 90 deg turn detect/execute\r\n");
    UART_sendString("MAX=");      UART_sendInt(MAX_SPEED_PCT);
    UART_sendString("%  BASE=");  UART_sendInt(BASE_SPEED_PCT);
    UART_sendString("%  TRIM=");  UART_sendInt(MOTOR_TRIM_PCT);
    UART_sendString("%  Kp*100="); UART_sendInt(KP_X100);
    UART_sendString(" Kd*100=");  UART_sendInt(KD_X100);
    UART_sendString(" Ki*100=");  UART_sendInt(KI_X100);
    UART_sendString("\r\nOPEN_MM="); UART_sendInt(OPENING_MM);
    UART_sendString(" CONFIRM="); UART_sendInt(OPENING_CONFIRM);
    UART_sendString(" ENTRY=");   UART_sendInt(TURN_ENTRY_OVFS);
    UART_sendString(" SPIN=");    UART_sendInt(TURN_SPIN_OVFS);
    UART_sendChar('@');           UART_sendInt(TURN_SPIN_PCT);
    UART_sendString("%\r\n");

    motors_enable(1);
}

void loop(void) {
    static uint16_t last_print_ovf = 0;
    static uint16_t last_pid_ovf   = 0;

    us_service();

    uint16_t ovf = us_overflow_count();

    if ((uint16_t)(ovf - last_pid_ovf) >= PID_PERIOD_OVFS) {
        last_pid_ovf = ovf;
        if (state == ST_FOLLOW) follow_step(ovf);
        else                    turn_step(ovf);
    }

    if ((uint16_t)(ovf - last_print_ovf) >= PRINT_PERIOD_OVFS) {
        last_print_ovf = ovf;
        UART_sendChar('['); UART_sendString(state_tag()); UART_sendString("] ");
        print_dist ("L=",  us_distance_mm(US_LEFT));
        print_dist ("R=",  us_distance_mm(US_RIGHT));
        print_dist ("F=",  us_distance_mm(US_FRONT));
        print_field("err=", last_error);
        print_field("oL=",  last_outL);
        print_field("oR=",  last_outR);
        print_field("T=",   (int16_t)turn_count);
        if (turn_count) {
            UART_sendString("seq=");
            uint8_t n = turn_count < TURN_SEQ_MAX ? turn_count : TURN_SEQ_MAX;
            for (uint8_t i = 0; i < n; i++) {
                if (i) UART_sendChar(',');
                UART_sendChar(turn_seq[i]);
            }
        }
        if (front_braking) UART_sendString(" [STOP]");
        UART_sendString("\r\n");
        LED_PORT ^= (1 << LED_BIT);
    }
}