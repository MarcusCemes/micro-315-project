#include "utils.h"

inline size_t min(size_t a, size_t b)
{
    return a <= b ? a : b;
}

inline int8_t signf(float number)
{
    return number > 0 ? 1 : number < 0 ? -1 : 0;
}

/* == Readers-Writer lock implementation == */

void rw_init(rw_lock_t* lock)
{
    chMtxObjectInit(&lock->mtx);
    chCondObjectInit(&lock->cond);
    lock->readers = 0;
}

void rw_read_lock(rw_lock_t* lock)
{
    chMtxLock(&lock->mtx);
    while (lock->readers == -1)
        chCondWait(&lock->cond);
    ++lock->readers;
    chMtxUnlock(&lock->mtx);
}

void rw_read_unlock(rw_lock_t* lock)
{
    chMtxLock(&lock->mtx);
    --lock->readers;
    if (lock->readers == 0)
        chCondBroadcast(&lock->cond);
    chMtxUnlock(&lock->mtx);
}

void rw_write_lock(rw_lock_t* lock)
{
    chMtxLock(&lock->mtx);
    while (lock->readers != 0)
        chCondWait(&lock->cond);
    lock->readers = -1;
    chMtxUnlock(&lock->mtx);
}

void rw_write_unlock(rw_lock_t* lock)
{
    chMtxLock(&lock->mtx);
    lock->readers = 0;
    chCondBroadcast(&lock->cond);
    chMtxUnlock(&lock->mtx);
}
