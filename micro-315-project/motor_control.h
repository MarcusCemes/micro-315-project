#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MC_HALT,
    MC_FORWARD,
    MC_BACKWARD,
} mctl_dir_t;

typedef enum
{
    MC_RIGHT,
    MC_LEFT,
    NB_MOTORS,
} mctl_side_t;

/** Gets the step counter value for a given motor. */
uint32_t mctl_get_counter(mctl_side_t side);

/** Sets the step counter value for a given motor. */
void mctl_set_counter(mctl_side_t side, uint32_t steps);

/** Gets the motor's step target count. */
uint32_t mctl_get_target(mctl_side_t side);

/** Sets the motor's step target count. */
void mctl_set_target(mctl_side_t side, uint32_t steps);

/**
 * Actives/decactives the step target of a motor, after which
 * the motor will automatically shutdown and notify any waiting
 * threads.
 */
void mctl_set_target_enabled(mctl_side_t side, bool enabled);

/** Set the angular velocity of both motors, with the left motor is inverted. */
void mctl_set_inverse_velocity(int16_t velocity);

/** Set the angular velocity of both motors. */
void mctl_set_common_velocity(int16_t velocity);

/** Rotate the motor a specific number of steps at a given velocity. */
void mctl_rotate_by(mctl_side_t side, uint32_t steps, int16_t velocity);

/**
 * Set the angular velocity [step/s] of a motor. Setting to zero will
 * put the motor in a halt state. Speeds lower than the threshold will
 * enable a powersaving feature that uses a PWM duty cycle.
 */
void mctl_set_motor_velocity(mctl_side_t side, int16_t velocity);

/** Checks if the motor is stopped, and if not, blocks until it is halted. */
void mctl_wait_halted(mctl_side_t side);

/** Blocks until all motors are halted. */
void mctl_wait_halted_all(void);

/** Initialise motor state and cut current to motors. */
void mctl_init(void);

/** Start timers responsible for motor control. */
void mctl_start(void);

#endif
