#include "lights.h"

#include <ch.h>
#include <leds.h>

#define ON 1
#define OFF 0
#define MAX_INTENSITY 255

/** The animation that should be looped. */
static lights_animation_t _animation = LIGHTS_STOP;

/** The animation mode. */
static lights_mode_t _mode = LIGHTS_LOOP;

/**
 * A temporary storage of the animation to play once, without overwriting
 * the looped animation.
 */
static lights_animation_t _once_animation = LIGHTS_STOP;

/**
 * This semaphore is used to cancel the animation in progress by being
 * released. The thread will wait on it with a timeout. If the timeout
 * is triggered, the animation will continue.
 */
static binary_semaphore_t _cancellation_bsem;

/* === Private functions === */

/* == Utility == */

/**
 * Attempts to sleep for a given period of time.
 * @returns true if the wait was successful, false if cancelled.
 */
static bool try_wait(uint32_t time_ms)
{
    msg_t msg = chBSemWaitTimeout(&_cancellation_bsem, MS2ST(time_ms));
    return msg == MSG_TIMEOUT;
}

/** Utility macro for calling try_wait() and returning if cancelled. */
#define TRY_WAIT(MS)                                                                               \
    if (!try_wait(MS))                                                                             \
        return false;

/** Utility macro for looping over non-RGB LEDS. */
#define FOR_EACH_LED(i) for (uint8_t i = 0; i < NUM_LEDS; ++i)

/** Utility macro for looping over RGB LEDS. */
#define FOR_EACH_RGB(i) for (uint8_t i = 0; i < NUM_RGB_LED; ++i)

#define SET_RGB_LEDS(r, g, b)                                                                      \
    for (uint8_t i = 0; i < NUM_RGB_LED; ++i)                                                      \
        set_rgb_led(i, r, g, b);

/* == Animations == */

/** Blinks all LEDs red once. */
static bool blink_red(uint32_t delay_ms)
{
    for (uint8_t i = 0; i < NUM_LED; ++i)
    {
        set_led(i, ON);
        set_rgb_led(i, MAX_INTENSITY, OFF, OFF);
    }
    TRY_WAIT(delay_ms);

    clear_leds();
    TRY_WAIT(delay_ms);

    return true;
}

/** Does nothing. */
static bool animation_stop(void)
{
    return try_wait(TIME_INFINITE);
}

/** Blinks all LEDs red. */
static bool animation_waiting(void)
{
    return blink_red(1000);
}

static bool animation_attention(void)
{
    for (uint8_t j = 0; j < 5; ++j)
        if (!blink_red(100))
            return false;

    TRY_WAIT(500);
    return true;
}

/** Creates a clockwise LED spin effect. */
static bool animation_spin(void)
{
    for (uint8_t i = 0; i < NUM_LED; ++i)
    {
        set_led(i, ON);
        TRY_WAIT(100);
        set_led(i, OFF);

        set_rgb_led(i, MAX_INTENSITY, OFF, OFF);
        TRY_WAIT(100);
        set_rgb_led(i, OFF, OFF, OFF);
    }

    return true;
}

/** Simulates emergency-vehicle lighting. */
static bool animation_emergency(void)
{
    SET_RGB_LEDS((i >= 2) ? MAX_INTENSITY : OFF, OFF, OFF);
    TRY_WAIT(400);
    SET_RGB_LEDS(OFF, OFF, (i <= 1) ? MAX_INTENSITY : OFF);
    TRY_WAIT(400);
    return true;
}

static bool animation_standby(void)
{
    SET_RGB_LEDS(MAX_INTENSITY, 0.15 * MAX_INTENSITY, OFF);
    TRY_WAIT(TIME_INFINITE);
    return true;
}

static bool play_animation(lights_animation_t animation)
{
    switch (animation)
    {
        case LIGHTS_STOP:
            return animation_stop();

        case LIGHTS_WAITING:
            return animation_waiting();

        case LIGHTS_ATTENTION:
            return animation_attention();

        case LIGHTS_EMERGENCY:
            return animation_emergency();

        case LIGHTS_STANDBY:
            return animation_standby();

        case LIGHTS_SPIN:
            return animation_spin();
    }

    return true;
}

static THD_WORKING_AREA(lights_stack, 128);
static THD_FUNCTION(lights_thread, arg)
{
    (void)arg;

    while (!chThdShouldTerminateX())
    {
        clear_leds();
        chBSemReset(&_cancellation_bsem, true);

        switch (_mode)
        {
            case LIGHTS_ONCE:
                play_animation(_once_animation);
                _mode = LIGHTS_LOOP;
                break;

            case LIGHTS_LOOP:
                while (play_animation(_animation))
                    ;
        }
    }

    clear_leds();
}

void lights_init()
{
    chBSemObjectInit(&_cancellation_bsem, false);
}

void lights_start(void)
{
    chThdCreateStatic(lights_stack, sizeof(lights_stack), HIGHPRIO, lights_thread, NULL);
}

void lights_trigger(lights_animation_t animation, lights_mode_t mode)
{
    lights_queue(animation, mode);
    chBSemSignal(&_cancellation_bsem);
}

void lights_queue(lights_animation_t animation, lights_mode_t mode)
{
    switch (mode)
    {
        case LIGHTS_ONCE:
            _mode = LIGHTS_ONCE;
            _once_animation = animation;
            break;

        case LIGHTS_LOOP:
            _mode = LIGHTS_LOOP;
            _animation = animation;
            break;
    }
}
