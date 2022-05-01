#ifndef LIGHTS_H
#define LIGHTS_H

typedef enum
{
    LIGHTS_ONCE,
    LIGHTS_LOOP,
} lights_mode_t;

typedef enum
{
    LIGHTS_STOP,
    LIGHTS_WAITING,
    LIGHTS_SPIN,
} lights_animation_t;

/** Initialize the lights thread. */
void init_lights(void);

/**
 * Interrupt the lights thread and start playing the given animation.
 *
 * If the mode is set to `LIGHTS_ONCE`, the thread will return to playing
 * the original animation after the animation is completed.
 */
void trigger_lights(lights_animation_t animation, lights_mode_t mode);

#endif
