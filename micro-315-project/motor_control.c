#include "motor_control.h"

#include <ch.h>
#include <hal.h>

#include "utils.h"

/* === Definitions === */

/** The 4-pin GPIO configuration for a particular motor state. */
typedef bool motor_state_t[4];

/** Represents a motor, holding counters and configuration for that motor. */
typedef struct
{
    uint16_t speed;
    mctl_dir_t direction;
    int32_t half_step_count;
    uint8_t step_index;
} motor_t;

/* === Constants === */

#define MOTOR_TIMER_FREQ 100000  // [Hz]

#define MOTOR_SPEED_LIMIT 1100  // [step/s]
#define SPEED_THRESHOLD 650     // Threshold speed to disable power-saving feature [step/s]
#define NB_STEPS 8              // The number of active states the motor can be in
#define NB_PHASES 4             // The number of phases/pins a motor has

#define ZERO_SPEED 0            // The speed at which the motor is stopped
#define MAX_PERIOD UINT16_MAX   // The maximum counter value for a timer (16 bits)
#define STEP_HALF_STEP_RATIO 2  // The ratio between a full step and a half step
#define POWERSAVE_CHANNEL 0     // The timer channel used for powersaving

/** Lookup table for motor step sequence. */
static const motor_state_t _step_lut[NB_STEPS] = {
    { 1, 0, 1, 0 }, { 0, 0, 1, 0 }, { 0, 1, 1, 0 }, { 0, 1, 0, 0 },
    { 0, 1, 0, 1 }, { 0, 0, 0, 1 }, { 1, 0, 0, 1 }, { 1, 0, 0, 0 },
};

/** Prevent an active current going through the motor when not in use. */
static const motor_state_t _state_halt = { 0, 0, 0, 0 };

/** Pointers to timers corresponding to each motor. */
static const PWMDriver *_motor_timers[NB_MOTORS] = { &PWMD3, &PWMD4 };

/** The GPIO pins corresponding to each motor phase. */
static const uint8_t _motor_pins[NB_MOTORS][4] = {
    { GPIOE_MOT_R_IN1, GPIOE_MOT_R_IN2, GPIOE_MOT_R_IN3, GPIOE_MOT_R_IN4 },
    { GPIOE_MOT_L_IN1, GPIOE_MOT_L_IN2, GPIOE_MOT_L_IN3, GPIOE_MOT_L_IN4 },
};

/* === Static variables === */

/** The state for each of the motors phases, corresponding to a motor position. */
static motor_t _motors[NB_MOTORS];

/** Contains the PWM configuration for each motor. */
static PWMConfig _motor_pwm_configs[NB_MOTORS];

/* === Private functions === */

/* == Utility == */

/** A modulo-8 implementation that uses a single binary AND operation. */
static inline uint8_t fast_modulo_8(int8_t index)
{
    return index & 0b0111;
}

/** Returns 1 if the condition is true, or -1 if not. */
static inline int8_t unit_xor(bool condition)
{
    return condition ? 1 : -1;
}

static inline uint32_t S2HS(uint32_t step)
{
    return step * STEP_HALF_STEP_RATIO;
}

static inline uint32_t HS2S(uint32_t step)
{
    return step / STEP_HALF_STEP_RATIO;
}

/** Returns the direction of a particular motor velocity. */
static inline mctl_dir_t V2D(int16_t velocity)
{
    return velocity == ZERO_SPEED ? MC_HALT : velocity > ZERO_SPEED ? MC_FORWARD : MC_BACKWARD;
}

/* == GPIO == */

/** Updates a single motor pin to a boolean value. */
static void update_motor_pin(uint32_t pin, bool active)
{
    active ? palSetPad(GPIOE, pin) : palClearPad(GPIOE, pin);
}

/** Applies a given motor state to the given motor's GPIO pins */
static void update_motor_pins(mctl_side_t side, const motor_state_t state)

{
    for (uint8_t i = 0; i < NB_PHASES; ++i)
        update_motor_pin(_motor_pins[side][i], state[i]);
}

/** Returns the motor side that is associated with a particular timer. */
static mctl_side_t side_from_timer(PWMDriver *timer) __attribute__((hot));
static mctl_side_t side_from_timer(PWMDriver *timer)
{
    for (uint8_t i = 0; i < NB_MOTORS; ++i)
        if (_motor_timers[i] == timer)
            return i;

    chSysHalt("Unknown timer");
    __builtin_unreachable();
}

/* == Commands == */

/** Immediately cut the current running to a motor. Does not update the state! */
static inline void halt_motor(mctl_side_t side)
{
    update_motor_pins(side, _state_halt);
}

/** Immediately cut current to all motors. Does not update the state! */
static void halt_motor_all(void)
{
    for (uint8_t i = 0; i < NB_MOTORS; ++i)
        halt_motor(i);
}

/** Enables PWM-based current reduction for the given motor. */
static inline void motor_powersave_enable(mctl_side_t side)
{
    pwmEnableChannel((PWMDriver *)_motor_timers[side], POWERSAVE_CHANNEL,
                     (pwmcnt_t)(MOTOR_TIMER_FREQ / SPEED_THRESHOLD));
}

/** Disables PWM-based current reduction for the given motor. */
static inline void motor_powersave_disable(mctl_side_t side)
{
    pwmDisableChannel((PWMDriver *)_motor_timers[side], POWERSAVE_CHANNEL);
}

/* == Callbacks == */

/** Callback to update the motor's state and GPIO pins. */
static void motor_tick_cb(PWMDriver *timer)
{
    mctl_side_t side = side_from_timer(timer);
    motor_t *motor = &_motors[side];

    if (motor->direction == MC_HALT)
    {
        halt_motor(side);
        return;
    }

    int8_t dir_int = unit_xor(motor->direction == MC_FORWARD);
    int8_t side_int = unit_xor(side == MC_LEFT);
    int8_t index_inc = side_int * dir_int;
    uint8_t step_index = fast_modulo_8((int8_t)motor->step_index + index_inc);

    motor->step_index = step_index;
    motor->half_step_count += dir_int;

    update_motor_pins(side, _step_lut[step_index]);
}

/**
 * Callback invoked by CH1 of the PWM to cut current to motors during the
 * active part of the duty cycle to avoid high current draw at low speeds.
 */
static void motor_powersave_cb(PWMDriver *timer)
{
    mctl_side_t side = side_from_timer(timer);
    halt_motor(side);
}

/* == Initialisation == */

/** Initialises the state structs for the given motor. */
static void init_motor(mctl_side_t side)
{
    _motors[side] = (motor_t){
        .speed = ZERO_SPEED,
        .direction = MC_HALT,
        .half_step_count = 0,
        .step_index = 0,
    };
}

/** Initialises the state struct for all motors. */
static void init_motor_all(void)
{
    for (uint8_t i = 0; i < NB_MOTORS; ++i)
        init_motor(i);
}

/** Initialise the PWM timer for the given motor. */
static void init_motor_pwm(mctl_side_t side)
{
    PWMDriver *timer = (PWMDriver *)_motor_timers[side];
    PWMConfig *timer_config = &_motor_pwm_configs[side];

    *timer_config =  (PWMConfig){
        .frequency = MOTOR_TIMER_FREQ,
        .period = MAX_PERIOD,
        .cr2 = 0,
        .callback = motor_tick_cb,
        .channels = {
            {.mode = PWM_OUTPUT_ACTIVE_HIGH, .callback = motor_powersave_cb},
            {.mode = PWM_OUTPUT_DISABLED, .callback = NULL},
            {.mode = PWM_OUTPUT_DISABLED, .callback = NULL},
            {.mode = PWM_OUTPUT_DISABLED, .callback = NULL},
        },
    };

    // Configure the PWM peripheral
    pwmStart(timer, timer_config);

    // Enable period-start interrupts, invoking the motor_tick() callback
    pwmEnablePeriodicNotification(timer);
}

/** Initialise the PWM timer for all motors. */
static void init_motor_pwm_all(void)
{
    for (uint8_t i = 0; i < NB_MOTORS; ++i)
        init_motor_pwm(i);
}

/* === Public functions === */

uint32_t mctl_get_counter(mctl_side_t side)
{
    motor_t *motor = &_motors[side];
    return HS2S(motor->half_step_count);
}

void mctl_set_counter(mctl_side_t side, uint32_t steps)
{
    motor_t *motor = &_motors[side];
    motor->half_step_count = S2HS(steps);
}

void mctl_set_inverse_velocity(int16_t velocity)
{
    mctl_set_motor_velocity(MC_RIGHT, velocity);
    mctl_set_motor_velocity(MC_LEFT, -velocity);
}

void mctl_set_common_velocity(int16_t velocity)
{
    mctl_set_motor_velocity(MC_RIGHT, velocity);
    mctl_set_motor_velocity(MC_LEFT, velocity);
}

void mctl_set_motor_velocity(mctl_side_t side, int16_t velocity)
{
    motor_t *motor = &_motors[side];

    uint16_t step_speed = clamp(abs(velocity), ZERO_SPEED, MOTOR_SPEED_LIMIT);
    motor->speed = step_speed;
    motor->direction = V2D(velocity);

    if (step_speed == ZERO_SPEED)
    {
        motor_powersave_disable(side);
        pwmChangePeriod((PWMDriver *)_motor_timers[side], 1000);
        return;
    }

    uint16_t half_step_speed = 2 * step_speed;
    if (half_step_speed < SPEED_THRESHOLD)
        motor_powersave_enable(side);
    else
        motor_powersave_disable(side);

    pwmChangePeriod((PWMDriver *)_motor_timers[side], MOTOR_TIMER_FREQ / half_step_speed);
}

void mctl_init(void)
{
    init_motor_all();
    halt_motor_all();
}

void mctl_start(void)
{
    init_motor_pwm_all();
}
