#ifndef UTILS_H
#define UTILS_H

#include <arm_math.h>
#include <ch.h>
#include <math.h>
#include <stdlib.h>

/* == Definitions == */

#define PI_AS_DEG 180

/**
 * Implementation of a Readers-Writer lock, favouring reads.
 * Allows a resource to be safely accessed by multiple threads
 * concurrently, such as when broadcasting data.
 */
typedef struct
{
    mutex_t mtx;
    condition_variable_t cond;
    int32_t readers;
} rw_lock_t;

/* == Inline functions == */

/** Returns the smallest of two integers. */
static inline size_t min(size_t a, size_t b)
{
    return a <= b ? a : b;
}

/** The sign of a float, returning -1, 0 or 1. */
static inline int8_t signf(float number)
{
    return number > 0 ? 1 : number < 0 ? -1 : 0;
}

/** COnvert radians to degrees. */
static inline float rad2deg(float radians)
{
    return radians * PI_AS_DEG / M_PI;
}

/**
 * A fast implementation of `i = (i + 1) % modulo`, using a branch.
 * This will only check if the incremented value is **equal** to
 * the modulo value to reset to zero!
 */
static inline size_t inc_mod(size_t i, size_t modulo)
{
    size_t incremented = i + 1;
    return incremented == modulo ? 0 : incremented;
}

/* == Readers-writer lock == */

void rw_init(rw_lock_t* lock);
void rw_read_lock(rw_lock_t* lock);
void rw_read_unlock(rw_lock_t* lock);
void rw_write_lock(rw_lock_t* lock);
void rw_write_unlock(rw_lock_t* lock);

/* == PID controller == */

typedef arm_pid_instance_f32 pid_ctl_t;

/** Initialise the PID instance, also reseting the state. */
void pid_init(pid_ctl_t* pid, float Kp, float Ki, float Kd);

/** Reset the PID instance. */
void pid_reset(pid_ctl_t* pid);

/** Update the PID instance with a new discrete input. */
static inline float pid_update(pid_ctl_t* pid, float in)
{
    return arm_pid_f32(pid, in);
}

#endif
