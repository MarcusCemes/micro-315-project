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
    LIGHTS_ATTENTION,
    LIGHTS_SPIN,
    LIGHTS_EMERGENCY,
    LIGHTS_STANDBY,
} lights_animation_t;

/** Initialize resources need by the lights thread. */
void lights_init(void);

/** Start the lights thread. */
void lights_start(void);

/**
 * Set the next animation sequence, as well as the play mode.
 * If the mode is set to `LIGHTS_ONCE`, the animation will be placed once
 * before the original animation wil resume playing.
 *
 * This play the animation immediately, interrupting the current animation.
 */
void lights_trigger(lights_animation_t animation, lights_mode_t mode);

/**
 * Set the next animation sequence, as well as the play mode.
 * If the mode is set to `LIGHTS_ONCE`, the animation will be placed once
 * before the original animation wil resume playing.
 */
void lights_queue(lights_animation_t animation, lights_mode_t mode);

#endif
