#include "programs.h"

#include <ch.h>
#include <hal.h>
#include <math.h>

#include "audio.h"
#include "comms.h"
#include "lights.h"
#include "location.h"
#include "movement.h"
#include "speaker.h"

#define ANGLE_SLOW_THRESHOLD 0.2    // Angle below which slow increments happen
#define ANGLE_SLOW_INCREMENT 0.005  // The angle to increment by at slow speed
#define ANGLE_FOUND_THRESHOLD 0.05  // Angle which is considered to be zero
#define MAGNITUDE_THRESHOLD 6000    // Magnitude which is the noise ceiling
#define HIT_THRESHOLD 8             // Number of angle found hits required to validate
#define BIAS_THRESHOLD 0.2          // Allowed maximum bias correct
#define FB_DELTA_THRESHOLD -1.0     // The mid-point between determining front/back orientation

/* == Point sound == */

void program_point_sound(void)
{
    lights_trigger(LIGHTS_WAITING, LIGHTS_LOOP);
    audio_subscribe();

    while (true)
    {
        audio_wait();
        audio_data_t* data = audio_data_borrow();

        if (data->value[A_BACK].magnitude <= MAGNITUDE_THRESHOLD)
        {
            audio_data_return();
            lights_queue(LIGHTS_WAITING, LIGHTS_LOOP);
            continue;
        }

        lights_queue(LIGHTS_SPIN, LIGHTS_LOOP);

        float delta_fb
            = calculate_phase_delta(data->value[A_BACK].phase, data->value[A_FRONT].phase);
        if (delta_fb < FB_DELTA_THRESHOLD)
        {
            audio_data_return();
            rotate_by_turns_smooth(0.5, MOVE_MAX);
            continue;
        }

        float angle = loc_estimate_angle(data);
        audio_data_return();

        if (isnanf(angle))
            continue;

        float angle_abs = fabsf(angle);
        if (angle_abs < ANGLE_FOUND_THRESHOLD)
            break;

        bool is_slow = angle_abs <= ANGLE_SLOW_THRESHOLD;
        float rotate_by_angle = is_slow ? copysignf(ANGLE_SLOW_INCREMENT, angle) : angle;
        rotate_by(rotate_by_angle, MOVE_WAIT, is_slow ? MOVE_VERY_SLOW : MOVE_SLOW);
    }

    audio_unsubscribe();
    lights_trigger(LIGHTS_SUCCESS, LIGHTS_LOOP);
}

/* == Locate sound == */

/** Restrict an angle to the range [-pi, pi]. */
static float restrict_angle(float angle)
{
    float restricted_angle = fmodf(angle, M_TWOPI);
    if (fabsf(restricted_angle) > M_PI)
        return restricted_angle - copysignf(M_TWOPI, restricted_angle);
    return restricted_angle;
}

static float get_bias(void)
{
    lights_trigger(LIGHTS_STANDBY, LIGHTS_LOOP);
    uint32_t hits = 0;

    audio_subscribe();
    while (true)
    {
        audio_wait();
        audio_data_t* data = audio_data_borrow();

        if (data->value[A_BACK].magnitude <= MAGNITUDE_THRESHOLD)
        {
            audio_data_return();
            hits = 0;
            continue;
        }

        float angle = loc_estimate_angle(data);
        audio_data_return();

        if (!isnanf(angle) && ++hits >= HIT_THRESHOLD)
        {
            audio_unsubscribe();
            lights_trigger(LIGHTS_SUCCESS, LIGHTS_LOOP);
            speaker_play_wav(SPKR_WAV_PING_01, SF_FORCE_CHANGE, true);

            return fabsf(angle) < BIAS_THRESHOLD ? angle : 0.0f;
        }
    }
}

static void rotate_at_reasonable_speed(float angle)
{
    float angle_abs = fabsf(angle);
    if (angle_abs > ANGLE_SLOW_THRESHOLD)
        rotate_by_smooth(angle, MOVE_SLOW);
    else
        rotate_by(angle, MOVE_WAIT, MOVE_VERY_SLOW);
}

static float locate_sound_angle(void)
{
    float angle_acc = 0.0f;
    uint32_t hits = 0;

    lights_trigger(LIGHTS_WAITING, LIGHTS_LOOP);

    audio_subscribe();
    while (true)
    {
        audio_wait();
        audio_data_t* data = audio_data_borrow();

        if (data->value[A_BACK].magnitude <= MAGNITUDE_THRESHOLD)
        {
            audio_data_return();
            lights_queue(LIGHTS_WAITING, LIGHTS_LOOP);
            continue;
        }

        lights_queue(LIGHTS_SPIN, LIGHTS_LOOP);
        float angle = loc_estimate_angle(data);
        audio_data_return();

        float delta_fb
            = calculate_phase_delta(data->value[A_BACK].phase, data->value[A_FRONT].phase);

        if (delta_fb < FB_DELTA_THRESHOLD)
        {
            hits = 0;
            angle_acc += M_PI;
            rotate_by_smooth(M_PI, MOVE_MAX);
            continue;
        }

        if (isnanf(angle))
            continue;

        float angle_abs = fabsf(angle);
        if (angle_abs < ANGLE_FOUND_THRESHOLD)
        {
            if (++hits > HIT_THRESHOLD)
                break;
            else
                continue;
        }

        hits = 0;
        bool is_slow = angle_abs <= ANGLE_SLOW_THRESHOLD;
        float rotate_by_angle = is_slow ? copysignf(ANGLE_SLOW_INCREMENT, angle) : angle;

        angle_acc += rotate_by_angle;
        rotate_at_reasonable_speed(rotate_by_angle);

        // Stabilise the device and audio recording
        chThdSleepMilliseconds(50);
    }

    audio_unsubscribe();

    lights_trigger(LIGHTS_SUCCESS, LIGHTS_LOOP);
    speaker_play_wav(SPKR_WAV_PING_01, SF_FORCE_CHANGE, false);
    return restrict_angle(angle_acc);
}

void program_locate_sound(void)
{
    const float lateral_distance = 0.2f;
    comms_send_msg("MSG", "Estimating sound distance using three sample points...");

    comms_send_msg("MSG", "Calibrating sound bias!");
    float bias = get_bias();

    // Estimate the angle from the origin
    float angle_c = locate_sound_angle();

    // Navigate to 2nd point
    rotate_by_smooth(M_PI_2, MOVE_MAX);
    lights_trigger(LIGHTS_EMERGENCY, LIGHTS_LOOP);
    move_by_smooth(lateral_distance, MOVE_MAX);
    rotate_by_smooth(-M_PI_2, MOVE_MAX);

    lights_trigger(LIGHTS_WAITING, LIGHTS_LOOP);
    float angle_l = locate_sound_angle();

    // Navigate to 3rd point
    rotate_by_smooth(-M_PI_2 - angle_l, MOVE_MAX);
    lights_trigger(LIGHTS_EMERGENCY, LIGHTS_LOOP);
    move_by_smooth(2 * lateral_distance, MOVE_MAX);
    rotate_by_smooth(M_PI_2, MOVE_MAX);

    lights_trigger(LIGHTS_WAITING, LIGHTS_LOOP);
    float angle_r = locate_sound_angle();

    rotate_by_smooth(M_PI_2 - angle_r, MOVE_MAX);
    lights_trigger(LIGHTS_EMERGENCY, LIGHTS_LOOP);
    move_by_smooth(lateral_distance, MOVE_MAX);
    rotate_by_smooth(-M_PI_2, MOVE_MAX);

    // Navigate to sound source
    float average_angle = (fabsf(angle_l - bias) + fabsf(angle_r - bias)) / 2.0f;
    float distance = lateral_distance * tanf(M_PI_2 - average_angle);
    comms_send_msg_f("MSG", "The average intersect angle is %f", average_angle);
    comms_send_msg_f("MSG", "Distance is estimated at %f", distance);

    lights_trigger(LIGHTS_EMERGENCY, LIGHTS_LOOP);
    move_by_smooth(distance, MOVE_MAX);
    lights_trigger(LIGHTS_ATTENTION, LIGHTS_ONCE);
    speaker_play_wav(SPKR_WAV_TAUNT, SF_FORCE_CHANGE, true);

    // Return to origin
    speaker_play_melody(WE_ARE_THE_CHAMPIONS, ML_SIMPLE_PLAY);
    rotate_by_smooth(M_PI, MOVE_MAX);
    move_by_smooth(distance, MOVE_MAX);

    rotate_by_smooth(M_TWOPI, MOVE_MAX);
    rotate_by_smooth(-M_TWOPI - M_PI - angle_c, MOVE_MAX);

    waitMelodyHasFinished();
    lights_trigger(LIGHTS_STANDBY, LIGHTS_LOOP);
}
