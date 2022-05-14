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
    uint16_t step_speed;
    mctl_dir_t direction;
    uint8_t step_index;
    int32_t half_step_count;
    bool half_step_target_enabled;
    int32_t half_step_target;
    threads_queue_t target_queue;
    bool interrupt_enabled;
    bool powersave_enabled;
} motor_t;

typedef PWMDriver timer_t;

/* === Definitions === */

#define MOTOR_TIMER_FREQ 100000  // [Hz]

#define MOTOR_SPEED_LIMIT 1100  // [step/s]
#define SPEED_THRESHOLD 650     // Threshold speed to disable power-saving feature [step/s]
#define NB_STEPS 8              // The number of active states the motor can be in
#define NB_PHASES 4             // The number of phases/pins a motor has

#define ZERO_SPEED 0            // The speed at which the motor is stopped
#define ZERO_COUNTER 0          // The initial counter state
#define MAX_PERIOD UINT16_MAX   // The maximum counter value for a timer (16 bits)
#define STEP_HALF_STEP_RATIO 2  // The ratio between a full step and a half step
#define POWERSAVE_CHANNEL 0     // The timer channel used for powersaving

/* === Constants === */

/** Lookup table for motor step sequence. */
static const motor_state_t _step_lut[NB_STEPS] = {
    { 1, 0, 1, 0 }, { 0, 0, 1, 0 }, { 0, 1, 1, 0 }, { 0, 1, 0, 0 },
    { 0, 1, 0, 1 }, { 0, 0, 0, 1 }, { 1, 0, 0, 1 }, { 1, 0, 0, 0 },
};

/** Prevent an active current going through the motor when not in use. */
static const motor_state_t _state_halt = { 0, 0, 0, 0 };

/** Pointers to timers corresponding to each motor. */
static const timer_t *_motor_timers[NB_MOTORS] = { &PWMD3, &PWMD4 };

/** The GPIO pins corresponding to each motor phase. */
static const uint8_t _motor_pins[NB_MOTORS][4] = {
    { GPIOE_MOT_R_IN1, GPIOE_MOT_R_IN2, GPIOE_MOT_R_IN3, GPIOE_MOT_R_IN4 },
    { GPIOE_MOT_L_IN1, GPIOE_MOT_L_IN2, GPIOE_MOT_L_IN3, GPIOE_MOT_L_IN4 },
};

/* === Static variables === */

/** Lock for syncrhonising access to static variables. */
static mutex_t _lock;

/** The state for each of the motors phases, corresponding to a motor position. */
static motor_t _motors[NB_MOTORS];

/** Contains the PWM configuration for each motor. */
static PWMConfig _motor_pwm_configs[NB_MOTORS];

/** Queue for threads waiting for all motors to be halted. */
static threads_queue_t _motor_halt_queue;

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

/** Get the motor associated with a side. */
static inline motor_t *get_motor(mctl_side_t side)
{
    return &_motors[side];
}

/** Utility to cast timer to non-const for ChibiOS functions. */
static inline timer_t *get_timer(mctl_side_t side)
{
    return (timer_t *)_motor_timers[side];
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
static mctl_side_t side_from_timer(timer_t *timer) __attribute__((hot));
static mctl_side_t side_from_timer(timer_t *timer)
{
    for (uint8_t i = 0; i < NB_MOTORS; ++i)
        if (_motor_timers[i] == timer)
            return i;

    chSysHalt("Unknown timer");
    __builtin_unreachable();
}

/** Returns whether the step counter has reached the target count. */
static bool motor_at_target(motor_t *motor)
{
    bool enabled = motor->half_step_target_enabled;
    bool reached = motor->half_step_count == motor->half_step_target;
    return enabled && reached;
}

/** Returns true if all motors are in the `MC_HALT` state. */
static bool all_halted(void)
{
    for (uint8_t i = 0; i < NB_MOTORS; ++i)
        if (_motors[i].direction != MC_HALT)
            return false;

    return true;
}

/* == Commands == */

/** Immediately cut the current running to a motor, **without** updating the state. */
static inline void halt_motor(mctl_side_t side)
{
    update_motor_pins(side, _state_halt);
}

/** Immediately cut current to all motors, **without** updating the state. */
static void halt_motor_all(void)
{
    for (uint8_t i = 0; i < NB_MOTORS; ++i)
        halt_motor(i);
}

/**
 * Enable/disable timer interrupts when the counter resets.
 * Checks whether a change is needed to avoid resetting the timer.
 */
static void motor_set_interrupt_enabled(mctl_side_t side, bool enabled)
{
    motor_t *motor = get_motor(side);
    if (motor->interrupt_enabled == enabled)
        return;

    motor->interrupt_enabled = enabled;

    timer_t *timer = get_timer(side);
    if (enabled)
        pwmEnablePeriodicNotification(timer);
    else
        pwmDisablePeriodicNotification(timer);
}

/** I-class variant to disable timer interrupts (used to shutdown from ISR). */
static inline void motor_interrupt_disableI(mctl_side_t side)
{
    motor_t *motor = get_motor(side);
    if (!motor->interrupt_enabled)
        return;

    motor->interrupt_enabled = false;
    pwmDisablePeriodicNotificationI(get_timer(side));
}

/**
 * CH1 timer interrupts are used to invoke motor_powersave_cb().
 * The invocation frequency is set to that of the main timer at SPEED_THRESHOLD.
 */
static void motor_set_powersave_enabled(mctl_side_t side, bool enabled)
{
    motor_t *motor = get_motor(side);
    if (motor->powersave_enabled == enabled)
        return;

    motor->powersave_enabled = enabled;

    timer_t *timer = get_timer(side);
    const pwmcnt_t interrupt_period = MOTOR_TIMER_FREQ / SPEED_THRESHOLD;
    if (enabled)
        pwmEnableChannel(timer, POWERSAVE_CHANNEL, interrupt_period);
    else
        pwmDisableChannel(timer, POWERSAVE_CHANNEL);
}

/** I-class variant to disable powersave interrupts (shutdown during ISR). */
static inline void motor_powersave_disableI(mctl_side_t side)
{
    motor_t *motor = get_motor(side);
    if (!motor->powersave_enabled)
        return;

    motor->powersave_enabled = false;
    pwmDisableChannelI(get_timer(side), POWERSAVE_CHANNEL);
}

/** Update the timer period for a given step speed. */
static void update_timer_speed(mctl_side_t side, uint32_t step_speed)
{
    motor_t *motor = get_motor(side);
    timer_t *timer = get_timer(side);

    if (motor->step_speed == step_speed)
        return;
    motor->step_speed = step_speed;

    pwmcnt_t counter = step_speed == ZERO_SPEED ? MAX_PERIOD : MOTOR_TIMER_FREQ / S2HS(step_speed);
    pwmChangePeriod(timer, counter);

    // Set the update bit to see immediate changes
    // See https://forum.chibios.org/viewtopic.php?t=5763
    timer->tim->EGR = 1;
}

/** I-class function that does a complete motor shutdown. */
static void motor_full_shutdownI(mctl_side_t side)
{
    motor_t *motor = get_motor(side);
    timer_t *timer = get_timer(side);

    halt_motor(side);

    motor_interrupt_disableI(side);
    motor_powersave_disableI(side);

    motor->step_speed = ZERO_SPEED;
    pwmChangePeriodI(timer, MAX_PERIOD);

    chSysLockFromISR();

    chThdDequeueAllI(&get_motor(side)->target_queue, MSG_OK);

    if (all_halted())
        chThdDequeueAllI(&_motor_halt_queue, MSG_OK);

    chSysUnlockFromISR();
}

/* == Callbacks == */

/**
 * Invoked in a ISR when the main timer counter resets. Updates the motor
 * state and the motor's GPIO pins for the next step configuration.
 *
 * If the motor has been marked as halted, the motor will undergo a full
 * shutdown.
 */
static void motor_tick_cb(timer_t *timer)
{
    // Bail from the ISR if the mutex is locked
    if (_lock.m_cnt != 0)
        return;

    mctl_side_t side = side_from_timer(timer);
    motor_t *motor = get_motor(side);

    if (motor_at_target(motor))
    {
        motor->direction = MC_HALT;
        motor->half_step_target_enabled = false;
    }

    switch (motor->direction)
    {
        case MC_HALT:
            motor_full_shutdownI(side);
            break;

        case MC_FORWARD:
        case MC_BACKWARD:
        {
            int8_t dir_int = unit_xor(motor->direction == MC_FORWARD);
            int8_t side_int = unit_xor(side == MC_LEFT);
            int8_t index_inc = side_int * dir_int;
            uint8_t step_index = fast_modulo_8((int8_t)motor->step_index + index_inc);

            motor->step_index = step_index;
            motor->half_step_count += dir_int;

            update_motor_pins(side, _step_lut[step_index]);
            break;
        }
    }
}

/**
 * Callback invoked by CH1 of the PWM timer to limit the current that
 * is drawn by the motors at low speed. While the period of the PWM
 * timer remains unchanged, CH1 will fire after the same amount of time
 * as if the motor was turning at SPEED_THRESHOLD. Current is then
 * reapplied at the next motor tick.
 */
static void motor_powersave_cb(timer_t *timer)
{
    halt_motor(side_from_timer(timer));
}

/* == Initialisation == */

/** Initialises the state structs for the given motor. */
static void init_motor(mctl_side_t side)
{
    motor_t *motor = get_motor(side);

    *motor = (motor_t){
        .step_speed = ZERO_SPEED,
        .direction = MC_HALT,
        .half_step_count = ZERO_COUNTER,
        .half_step_target = ZERO_COUNTER,
        .step_index = 0,
        .interrupt_enabled = false,
        .powersave_enabled = false,
    };

    chThdQueueObjectInit(&motor->target_queue);
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
    timer_t *timer = get_timer(side);
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

    // Configure and start the timer
    pwmStart(timer, timer_config);
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
    chMtxLock(&_lock);
    uint32_t steps = HS2S(get_motor(side)->half_step_count);
    chMtxUnlock(&_lock);
    return steps;
}

void mctl_set_counter(mctl_side_t side, uint32_t steps)
{
    chMtxLock(&_lock);
    get_motor(side)->half_step_count = S2HS(steps);
    chMtxUnlock(&_lock);
}

uint32_t mctl_get_target(mctl_side_t side)
{
    chMtxLock(&_lock);
    uint32_t steps = HS2S(get_motor(side)->half_step_target);
    chMtxUnlock(&_lock);
    return steps;
}

void mctl_set_target(mctl_side_t side, uint32_t steps)
{
    chMtxLock(&_lock);
    get_motor(side)->half_step_target = S2HS(steps);
    chMtxUnlock(&_lock);
}

void mctl_set_target_enabled(mctl_side_t side, bool enabled)
{
    chMtxLock(&_lock);
    get_motor(side)->half_step_target_enabled = enabled;
    chMtxUnlock(&_lock);
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

void mctl_rotate_by(mctl_side_t side, uint32_t steps, int16_t velocity)
{
    chMtxLock(&_lock);
    motor_t *motor = get_motor(side);
    motor->half_step_count = ZERO_COUNTER;
    motor->half_step_target = unit_xor(velocity) * S2HS(steps);
    mctl_set_motor_velocity(side, velocity);
    chMtxUnlock(&_lock);
}

void mctl_set_motor_velocity(mctl_side_t side, int16_t velocity)
{
    motor_t *motor = get_motor(side);
    uint16_t step_speed = clamp(abs(velocity), ZERO_SPEED, MOTOR_SPEED_LIMIT);

    chMtxLock(&_lock);
    motor->direction = V2D(velocity);

    // Motor shutdown is queued on next tick
    if (step_speed == ZERO_SPEED)
        return;

    motor_set_powersave_enabled(side, S2HS(step_speed) < SPEED_THRESHOLD);
    update_timer_speed(side, step_speed);
    motor_set_interrupt_enabled(side, true);

    chMtxUnlock(&_lock);
}

void mctl_wait_halted(mctl_side_t side)
{
    motor_t *motor = get_motor(side);

    chSysLock();
    chMtxLockS(&_lock);

    if (motor->direction != MC_HALT)
        chThdEnqueueTimeoutS(&motor->target_queue, TIME_INFINITE);

    chMtxUnlockS(&_lock);
    chSysUnlock();
}

void mctl_wait_halted_all(void)
{
    chSysLock();
    chMtxLockS(&_lock);
    bool all_are_halted = all_halted();
    chMtxUnlockS(&_lock);

    if (!all_are_halted)
        chThdEnqueueTimeoutS(&_motor_halt_queue, TIME_INFINITE);

    chSysUnlock();
}

void mctl_init(void)
{
    chMtxObjectInit(&_lock);
    chThdQueueObjectInit(&_motor_halt_queue);
    init_motor_all();
    halt_motor_all();
}

void mctl_start(void)
{
    init_motor_pwm_all();
}
