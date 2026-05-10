/*
 * WallBot — PID centering + sensor-gated 90° turn execution.
 *
 * Added:
 *   ST_POST_ALIGN
 *
 * Purpose:
 *   After a turn, sensors temporarily see bad geometry
 *   (old corridor / diagonal reflections).
 *
 *   Instead of immediately returning to aggressive PID centering,
 *   the robot briefly performs gentle one-wall stabilization
 *   before resuming normal correction.
 *
 * Existing states/behavior are intentionally preserved.
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

/* ---- TUNABLES: motion ----------------------------------------------- */

#define MAX_SPEED_PCT     100
#define BASE_SPEED_PCT     75
#define MOTOR_TRIM_PCT      6

#define KP_X100            18
#define KD_X100            68
#define KI_X100             0
#define INTEG_LIMIT      4000

#define FRONT_STOP_MM      70

/* ---- TUNABLES: turn detection --------------------------------------- */

#define F_DETECT_MAX_MM    350
#define SIDE_OPEN_MM       400
#define OPENING_CONFIRM      3

/* ---- TUNABLES: turn execution --------------------------------------- */

#define F_TURN_MM          150

#define APPROACH_TIMEOUT_OVFS  60

#define TURN_SPIN_PCT       35
#define TURN_SPIN_OVFS      23
#define TURN_SETTLE_OVFS     4

#define APPROACH_SPEED_PCT  50

#define APPROACH_SETPOINT_MM 225
#define APPROACH_KP_X100     20

/* ---- POST TURN ALIGN ----------------------------------------------- */

#define POST_ALIGN_OVFS        8
#define POST_ALIGN_SPEED_PCT  40
#define POST_ALIGN_KP_X100    10
#define POST_ALIGN_MAX_TURN   25

/* ---- misc ----------------------------------------------------------- */

#define TURN_SEQ_MAX        32

/* ---- scheduling ----------------------------------------------------- */

#define PID_PERIOD_OVFS      1
#define PRINT_PERIOD_OVFS    8

/* ---- DERIVED -------------------------------------------------------- */

#define PCT_TO_PWM(p)     ((int16_t)(((int32_t)(p) * 255) / 100))

#define MAX_SPEED         PCT_TO_PWM(MAX_SPEED_PCT)
#define BASE_SPEED        PCT_TO_PWM(BASE_SPEED_PCT)
#define APPROACH_SPEED    PCT_TO_PWM(APPROACH_SPEED_PCT)
#define TURN_SPIN_PWM     PCT_TO_PWM(TURN_SPIN_PCT)
#define POST_ALIGN_SPEED  PCT_TO_PWM(POST_ALIGN_SPEED_PCT)

/* ---- STATE ---------------------------------------------------------- */

typedef enum {
    ST_CORRECTION = 0,
    ST_APPROACH,
    ST_SPIN,
    ST_SETTLE,
    ST_POST_ALIGN
} state_t;

static state_t state = ST_CORRECTION;

static uint16_t state_start_ovf = 0;

static char pending_dir = 'L';

static uint8_t open_count_L = 0;
static uint8_t open_count_R = 0;

static uint8_t turn_count = 0;
static char turn_seq[TURN_SEQ_MAX];

static int16_t prev_error = 0;
static int32_t integ      = 0;

static int16_t last_outL  = 0;
static int16_t last_outR  = 0;
static int16_t last_error = 0;

static uint8_t front_braking = 0;

/* ---- HELPERS -------------------------------------------------------- */

static inline int16_t clamp16(int32_t v, int16_t lim)
{
    if (v > lim)  return lim;
    if (v < -lim) return -lim;
    return (int16_t)v;
}

static inline uint8_t side_is_open(uint16_t mm)
{
    return (mm == US_NO_READING) || (mm >= SIDE_OPEN_MM);
}

static inline uint8_t front_is_close(uint16_t mm)
{
    return (mm != US_NO_READING) && (mm <= F_DETECT_MAX_MM);
}

static void enter_state(state_t s, uint16_t now_ovf)
{
    state = s;
    state_start_ovf = now_ovf;
}

static void reset_pid(void)
{
    prev_error = 0;
    integ      = 0;

    open_count_L = 0;
    open_count_R = 0;
}

static void record_turn(char dir)
{
    if (turn_count < TURN_SEQ_MAX) {
        turn_seq[turn_count] = dir;
    }

    turn_count++;

    uart_puts("\r\n*** TURN #");
    uart_put_u16(turn_count);
    uart_puts(" ");
    uart_putc(dir);
    uart_puts(" ***\r\n");
}

static void print_field(const char *label,
                        int16_t v,
                        uint8_t is_signed)
{
    uart_puts(label);

    if (is_signed)
        uart_put_i16(v);
    else
        uart_put_u16((uint16_t)v);

    uart_putc(' ');
}

static void print_dist(const char *label, uint16_t mm)
{
    uart_puts(label);

    if (mm == US_NO_READING)
        uart_puts("----");
    else
        uart_put_u16(mm);

    uart_putc(' ');
}

static const char *state_tag(void)
{
    switch (state)
    {
        case ST_CORRECTION: return "COR";
        case ST_APPROACH:   return "APP";
        case ST_SPIN:       return "SPN";
        case ST_SETTLE:     return "SET";
        case ST_POST_ALIGN: return "PAL";
    }

    return "???";
}

/* ---- CORRECTION ----------------------------------------------------- */

static void correction_step(uint16_t ovf)
{
    uint16_t L = us_distance_mm(US_LEFT);
    uint16_t R = us_distance_mm(US_RIGHT);
    uint16_t F = us_distance_mm(US_FRONT);

    uint8_t fclose = front_is_close(F);

    if (fclose && side_is_open(L)) {
        if (open_count_L < 255)
            open_count_L++;
    } else {
        open_count_L = 0;
    }

    if (fclose && side_is_open(R)) {
        if (open_count_R < 255)
            open_count_R++;
    } else {
        open_count_R = 0;
    }

    if (open_count_L >= OPENING_CONFIRM ||
        open_count_R >= OPENING_CONFIRM)
    {
        pending_dir =
            (open_count_L >= OPENING_CONFIRM) ? 'L' : 'R';

        front_braking = 0;

        enter_state(ST_APPROACH, ovf);
        return;
    }

    if (F != US_NO_READING && F < FRONT_STOP_MM)
    {
        motors_stop();

        last_outL = 0;
        last_outR = 0;

        front_braking = 1;
        return;
    }

    front_braking = 0;

    if (L == US_NO_READING || R == US_NO_READING)
    {
        int16_t crawl = BASE_SPEED / 2;

        motors_set(crawl, crawl);

        last_outL = crawl;
        last_outR = crawl;

        last_error = 0;

        return;
    }

    int16_t error = (int16_t)R - (int16_t)L;

    int16_t deriv = error - prev_error;

    integ += error;

    if (integ > INTEG_LIMIT)
        integ = INTEG_LIMIT;

    if (integ < -INTEG_LIMIT)
        integ = -INTEG_LIMIT;

    int32_t turn =
        ((int32_t)KP_X100 * error +
         (int32_t)KD_X100 * deriv +
         (int32_t)KI_X100 * integ) / 100;

    int16_t turn_pwm = clamp16(turn, BASE_SPEED);

    int16_t left  = BASE_SPEED + turn_pwm;
    int16_t right = BASE_SPEED - turn_pwm;

    int16_t trim_pwm = PCT_TO_PWM(MOTOR_TRIM_PCT);

    left  += trim_pwm;
    right -= trim_pwm;

    if (left < 0)          left = 0;
    if (right < 0)         right = 0;
    if (left > MAX_SPEED)  left = MAX_SPEED;
    if (right > MAX_SPEED) right = MAX_SPEED;

    motors_set(left, right);

    prev_error = error;

    last_outL  = left;
    last_outR  = right;
    last_error = error;
}

/* ---- APPROACH ------------------------------------------------------- */

static void approach_step(uint16_t ovf)
{
    uint16_t F = us_distance_mm(US_FRONT);

    if (F != US_NO_READING && F <= F_TURN_MM)
    {
        enter_state(ST_SPIN, ovf);
        return;
    }

    if ((uint16_t)(ovf - state_start_ovf)
            >= APPROACH_TIMEOUT_OVFS)
    {
        enter_state(ST_SPIN, ovf);
        return;
    }

    uint16_t hold;
    int16_t err;

    if (pending_dir == 'L')
    {
        hold = us_distance_mm(US_RIGHT);

        if (hold == US_NO_READING)
            err = 0;
        else
            err =
                (int16_t)APPROACH_SETPOINT_MM -
                (int16_t)hold;
    }
    else
    {
        hold = us_distance_mm(US_LEFT);

        if (hold == US_NO_READING)
            err = 0;
        else
            err =
                (int16_t)hold -
                (int16_t)APPROACH_SETPOINT_MM;
    }

    int32_t correct =
        ((int32_t)APPROACH_KP_X100 * err) / 100;

    int16_t correct_pwm =
        clamp16(correct, APPROACH_SPEED);

    int16_t left =
        APPROACH_SPEED + correct_pwm;

    int16_t right =
        APPROACH_SPEED - correct_pwm;

    int16_t trim_pwm = PCT_TO_PWM(MOTOR_TRIM_PCT);

    left  += trim_pwm;
    right -= trim_pwm;

    if (left < 0)          left = 0;
    if (right < 0)         right = 0;
    if (left > MAX_SPEED)  left = MAX_SPEED;
    if (right > MAX_SPEED) right = MAX_SPEED;

    motors_set(left, right);

    last_outL  = left;
    last_outR  = right;
    last_error = err;
}

/* ---- SPIN ----------------------------------------------------------- */

static void spin_step(uint16_t ovf)
{
    if (pending_dir == 'L')
    {
        motors_set(-TURN_SPIN_PWM,
                    TURN_SPIN_PWM);

        last_outL = -TURN_SPIN_PWM;
        last_outR =  TURN_SPIN_PWM;
    }
    else
    {
        motors_set( TURN_SPIN_PWM,
                   -TURN_SPIN_PWM);

        last_outL =  TURN_SPIN_PWM;
        last_outR = -TURN_SPIN_PWM;
    }

    if ((uint16_t)(ovf - state_start_ovf)
            >= TURN_SPIN_OVFS)
    {
        enter_state(ST_SETTLE, ovf);
    }
}

/* ---- SETTLE --------------------------------------------------------- */

static void settle_step(uint16_t ovf)
{
    motors_stop();

    last_outL = 0;
    last_outR = 0;

    if ((uint16_t)(ovf - state_start_ovf)
            >= TURN_SETTLE_OVFS)
    {
        record_turn(pending_dir);

        reset_pid();

        enter_state(ST_POST_ALIGN, ovf);
    }
}

/* ---- POST ALIGN ----------------------------------------------------- */

static void post_align_step(uint16_t ovf)
{
    uint16_t hold;
    int16_t err;

    if (pending_dir == 'L')
    {
        hold = us_distance_mm(US_RIGHT);

        if (hold == US_NO_READING)
            err = 0;
        else
            err =
                (int16_t)APPROACH_SETPOINT_MM -
                (int16_t)hold;
    }
    else
    {
        hold = us_distance_mm(US_LEFT);

        if (hold == US_NO_READING)
            err = 0;
        else
            err =
                (int16_t)hold -
                (int16_t)APPROACH_SETPOINT_MM;
    }

    int32_t correct =
        ((int32_t)POST_ALIGN_KP_X100 * err) / 100;

    int16_t turn_pwm =
        clamp16(correct, POST_ALIGN_MAX_TURN);

    int16_t left =
        POST_ALIGN_SPEED + turn_pwm;

    int16_t right =
        POST_ALIGN_SPEED - turn_pwm;

    int16_t trim_pwm = PCT_TO_PWM(MOTOR_TRIM_PCT);

    left  += trim_pwm;
    right -= trim_pwm;

    if (left < 0)          left = 0;
    if (right < 0)         right = 0;
    if (left > MAX_SPEED)  left = MAX_SPEED;
    if (right > MAX_SPEED) right = MAX_SPEED;

    motors_set(left, right);

    last_outL  = left;
    last_outR  = right;
    last_error = err;

    if ((uint16_t)(ovf - state_start_ovf)
            >= POST_ALIGN_OVFS)
    {
        reset_pid();

        enter_state(ST_CORRECTION, ovf);
    }
}

/* ---- ENTRY POINTS --------------------------------------------------- */

void setup(void)
{
    LED_DDR  |=  (1 << LED_BIT);
    LED_PORT &= ~(1 << LED_BIT);

    uart_init(9600);

    us_init();

    motors_init();

    sei();

    uart_puts("\r\nWallBot + POST_ALIGN\r\n");

    motors_enable(1);
}

void loop(void)
{
    static uint16_t last_print_ovf = 0;
    static uint16_t last_pid_ovf   = 0;

    us_service();

    uint16_t ovf = us_overflow_count();

    if ((uint16_t)(ovf - last_pid_ovf)
            >= PID_PERIOD_OVFS)
    {
        last_pid_ovf = ovf;

        switch (state)
        {
            case ST_CORRECTION:
                correction_step(ovf);
                break;

            case ST_APPROACH:
                approach_step(ovf);
                break;

            case ST_SPIN:
                spin_step(ovf);
                break;

            case ST_SETTLE:
                settle_step(ovf);
                break;

            case ST_POST_ALIGN:
                post_align_step(ovf);
                break;
        }
    }

    if ((uint16_t)(ovf - last_print_ovf)
            >= PRINT_PERIOD_OVFS)
    {
        last_print_ovf = ovf;

        uart_putc('[');
        uart_puts(state_tag());
        uart_puts("] ");

        print_dist("L=", us_distance_mm(US_LEFT));
        print_dist("R=", us_distance_mm(US_RIGHT));
        print_dist("F=", us_distance_mm(US_FRONT));

        print_field("err=", last_error, 1);
        print_field("oL=",  last_outL, 1);
        print_field("oR=",  last_outR, 1);
        print_field("T=",   (int16_t)turn_count, 0);

        if (turn_count)
        {
            uart_puts("seq=");

            uint8_t n =
                turn_count < TURN_SEQ_MAX ?
                turn_count :
                TURN_SEQ_MAX;

            for (uint8_t i = 0; i < n; i++)
            {
                if (i)
                    uart_putc(',');

                uart_putc(turn_seq[i]);
            }
        }

        if (front_braking)
            uart_puts(" [STOP]");

        uart_puts("\r\n");

        LED_PORT ^= (1 << LED_BIT);
    }
}