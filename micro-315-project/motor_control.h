#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

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

/** Set the angular velocity of both motors, with the left motor is inverted. */
void mctl_set_inverse_speed(int16_t velocity);

/** Set the angular velocity of both motors. */
void mctl_set_common_speed(int16_t velocity);

/**
 * Set the angular velocity [step/s] of a motor. Setting to zero will
 * put the motor in a halt state. Speeds lower than the threshold will
 * enable a powersaving feature that uses a PWM duty cycle.
 */
void mctl_set_motor_velocity(mctl_side_t side, int16_t velocity);

/** Initialise motor state and cut current to motors. */
void mctl_init(void);

/** Start timers responsible for motor control. */
void mctl_start(void);

#endif
