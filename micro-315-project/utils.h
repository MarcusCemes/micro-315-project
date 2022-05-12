#ifndef UTILS_H
#define UTILS_H

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

/* == Readers-writer lock == */

void rw_init(rw_lock_t* lock);
void rw_read_lock(rw_lock_t* lock);
void rw_read_unlock(rw_lock_t* lock);
void rw_write_lock(rw_lock_t* lock);
void rw_write_unlock(rw_lock_t* lock);

#endif
