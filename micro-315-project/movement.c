#include "movement.h"

#include <ch.h>
#include <hal.h>
#include <math.h>

#include "motor_control.h"
#include "utils.h"

#define WHEEL_DIAMETRE 0.041  // [m]
#define WHEEL_DISTANCE 0.053  // [m]
#define STEPS_PER_REV 1000    // Steps per wheel revolution (20 w/ 50:1 gear)

// Number of steps to make the robot move by one metre
#define STEPS_PER_METRE STEPS_PER_REV / (M_PI * WHEEL_DIAMETRE)

// Number of steps to turn the robot 360 degrees
#define STEPS_PER_FULL_TURN STEPS_PER_REV*(WHEEL_DISTANCE / WHEEL_DIAMETRE)

// Number of steps to turn the robot by one radian
#define STEPS_PER_RADIAN_TURN STEPS_PER_FULL_TURN / M_TWOPI

#define ACCELERATION_DISTANCE 0.05   // The distance to accelerate over [m]
#define ACCELERATION_ANGLE M_PI / 8  // The angle to accelerate over [rad]
#define ACCELERATION_FRACTION 0.1    // Backup fraction if the distance/angle is too short

#define ACCEL_EXP_LEVELS 8  // The number of discrete speeds during exponential acceleration

#define STEP_ACCEL 5000     // The constant acceleration factor [steps/s^2]
#define ACCEL_FREQ 100      // The frequency at which to discretise acceleration [Hz]
#define MIN_ACCEL_SPEED 50  // The minimum speed during acceleration [steps/s]

typedef enum
{
    ACCELERATE,
    DECELERATE,
} accel_mode_t;

/* === Private functions === */

/* == Utility == */

/** Converts a distance to the number of wheel steps needed for that distance. */
static inline int32_t D2S(float distance)
{
    return (int32_t)(STEPS_PER_METRE * distance);
}

/** Converts a robot rotation in radians to number of wheel steps. */
static inline int32_t A2S(float angle)
{
    return (int32_t)(STEPS_PER_RADIAN_TURN * angle);
}

/** Returns 2^e using bit-shifting. */
static inline uint32_t fast_power_2(uint32_t e)
{
    return (1 << e);
}

/* == Commands == */

static void _set_motor_target(mctl_side_t side, int32_t steps)
{
    mctl_set_target(side, steps);
    mctl_set_target_enabled(side, true);
}

static void _set_motor_targets(int32_t right, int32_t left)
{
    _set_motor_target(MC_RIGHT, right);
    _set_motor_target(MC_LEFT, left);
}

static void _reset_motor_counters(void)
{
    mctl_set_counter(MC_RIGHT, 0);
    mctl_set_counter(MC_LEFT, 0);
}

static void _handle_mode(move_mode_t mode)
{
    if (mode == MOVE_WAIT)
        mctl_wait_halted_all();
}

static void _move_by_steps(int32_t steps, move_mode_t mode, move_speed_t speed)
{
    _set_motor_targets(steps, steps);
    _reset_motor_counters();
    mctl_set_common_velocity(steps > 0 ? speed : -speed);
    _handle_mode(mode);
}

static void _rotate_by_steps(int32_t steps, move_mode_t mode, move_speed_t speed)
{
    _set_motor_targets(steps, -steps);
    _reset_motor_counters();
    mctl_set_inverse_velocity(steps > 0 ? speed : -speed);
    _handle_mode(mode);
}

/**
 * A fallback acceleration function. The given distance is split into nested
 * regions that are fractions of powers of two, with a linear speed profile.
 * This is a fast alternative and reliable to other acceleration functions,
 * if not as smooth.
 */
static void _do_accelerated_exp(int32_t steps, move_speed_t speed, accel_mode_t mode,
                                void (*fn)(int32_t, move_mode_t, move_speed_t))
{
    int32_t remaining_steps = steps;

    // Generate 1/2^n,...,1/2^1 sequence for acceleration (inverse for deceleration)
    for (size_t i = 0; i < ACCEL_EXP_LEVELS; ++i)
    {
        int32_t dividing_coefficient = mode == ACCELERATE ? (ACCEL_EXP_LEVELS - i) : i + 1;
        int32_t iteration_coefficient = fast_power_2(dividing_coefficient);
        move_speed_t iteration_speed = max(MIN_ACCEL_SPEED, speed / iteration_coefficient);

        // Consume all remaining steps during the last iteration
        int32_t iteration_steps
            = i == ACCEL_EXP_LEVELS - 1 ? remaining_steps : steps / iteration_coefficient;

        if (abs(remaining_steps) < abs(iteration_steps) || iteration_steps == 0)
            continue;

        remaining_steps -= iteration_steps;
        (*fn)(iteration_steps, MOVE_WAIT, iteration_speed);
    }
}

/**
 * A generic acceleration function, taking a function pointer to the movement/rotation
 * function. This approximates constant acceleration motion in the discrete domain,
 * aiming at updating the motors at a constant frequency.
 *
 * This function approximates discrete values using continuous-time equations
 * to reduce calculation complexity. If the constant-acceleration is not estimated
 * to fit within the given step quota, the fallback exponential acceleration function
 * is used instead.
 */
static void _do_accelerated(int32_t steps, move_speed_t speed, accel_mode_t mode,
                            void (*fn)(int32_t, move_mode_t, move_speed_t))
{
    uint32_t estimated_steps = speed * speed / (2 * STEP_ACCEL);
    size_t n = ACCEL_FREQ * (float)speed / STEP_ACCEL;

    if (estimated_steps > abs(steps) || n == 0)
    {
        _do_accelerated_exp(steps, speed, mode, fn);
        return;
    }

    int32_t remaining_steps = steps;

    // Skip the first iteration at zero/max speed
    // Also helps with reducing overshoot
    for (size_t i = 0; i < n; ++i)
    {
        move_speed_t v_i_abs = i * speed / n;
        if (mode == ACCELERATE)
            v_i_abs = speed - v_i_abs;
        v_i_abs = max(MIN_ACCEL_SPEED, speed - v_i_abs);

        // Zero steps would cause a deadlock
        uint32_t x_i_abs = max(1, v_i_abs / ACCEL_FREQ);
        int32_t x_i = steps < 0 ? -x_i_abs : x_i_abs;

        // Skip iteration if it would the remaining step quota
        if (abs(remaining_steps) < x_i_abs)
            continue;

        remaining_steps -= x_i;
        (*fn)(x_i, MOVE_WAIT, v_i_abs);
    }

    uint32_t v_abs = max(MIN_ACCEL_SPEED, mode == ACCELERATE ? speed : speed / n);
    if (abs(remaining_steps) > 0)
        (*fn)(remaining_steps, MOVE_WAIT, v_abs);
}

/* === Public functions === */

void move_by(float distance, move_mode_t mode, move_speed_t speed)
{
    _move_by_steps(D2S(distance), mode, speed);
}

void move_by_smooth(float distance, move_speed_t speed)
{
    float accel_fr_abs = fabsf(ACCELERATION_FRACTION * distance);
    float accel_distance_abs = fminf(ACCELERATION_DISTANCE, accel_fr_abs);
    float acceleration_distance = copysignf(accel_distance_abs, distance);

    int32_t total_steps = D2S(distance);
    int32_t accel_steps = D2S(acceleration_distance);
    int32_t full_steps = total_steps - 2 * accel_steps;

    _do_accelerated(accel_steps, speed, ACCELERATE, &_move_by_steps);
    _move_by_steps(full_steps, MOVE_WAIT, speed);
    _do_accelerated(accel_steps, speed, DECELERATE, &_move_by_steps);
}

void rotate_by(float angle, move_mode_t mode, move_speed_t speed)
{
    int32_t target = STEPS_PER_RADIAN_TURN * angle;
    _rotate_by_steps(target, mode, speed);
}

void rotate_by_smooth(float angle, move_speed_t speed)
{
    float accel_fr_abs = fabsf(ACCELERATION_FRACTION * angle);
    float accel_angle_abs = fminf(ACCELERATION_ANGLE, accel_fr_abs);
    float acceleration_angle = copysignf(accel_angle_abs, angle);

    int32_t total_steps = A2S(angle);
    int32_t accel_steps = A2S(acceleration_angle);
    int32_t full_steps = total_steps - 2 * accel_steps;

    _do_accelerated(accel_steps, speed, ACCELERATE, _rotate_by_steps);
    _rotate_by_steps(full_steps, MOVE_WAIT, speed);
    _do_accelerated(accel_steps, speed, DECELERATE, _rotate_by_steps);
}

void rotate_by_turns(float turns, move_mode_t mode, move_speed_t speed)
{
    rotate_by(M_TWOPI * turns, mode, speed);
}

void rotate_by_turns_smooth(float turns, move_speed_t speed)
{
    rotate_by_smooth(M_TWOPI * turns, speed);
}
