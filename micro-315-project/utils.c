#include "utils.h"

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

/* == PID controller == */

typedef enum
{
    KEEP_STATE,
    RESET_STATE,
} pid_state_reset_t;

void pid_init(pid_ctl_t* pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    arm_pid_init_f32(pid, RESET_STATE);
}

void pid_update_parameters(pid_ctl_t* pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    arm_pid_init_f32(pid, KEEP_STATE);
}

void pid_reset(pid_ctl_t* pid)
{
    arm_pid_reset_f32(pid);
}
