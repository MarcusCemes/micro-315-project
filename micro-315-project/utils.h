#ifndef UTILS_H
#define UTILS_H

#include <stdlib.h>

/**
 * Implementation of a Readers–writer lock, favouring reads.
 * Allows a resource to be safely accessed by multiple threads
 * concurrently, such as when broadcasting data.
 */
typedef struct rw_lock_t rw_lock_t;

/** Returns the smallest of two integers. */
size_t min(size_t a, size_t b);

void rw_init(rw_lock_t* lock);
void rw_read_lock(rw_lock_t* lock);
void rw_read_unlock(rw_lock_t* lock);
void rw_write_lock(rw_lock_t* lock);
void rw_write_unlock(rw_lock_t* lock);

#endif