#include "programs.h"

#include <ch.h>
#include <hal.h>
#include <math.h>

#include "audio.h"
#include "lights.h"
#include "location.h"
#include "movement.h"

#define ANGLE_THRESHOLD 0.01      // Angle which is considered to be zero
#define MAGNITUDE_THRESHOLD 8000  // Magnitude which is the noise ceiling

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
        float angle = loc_estimate_angle(data);
        audio_data_return();

        if (fabsf(angle) < ANGLE_THRESHOLD)
            break;

        rotate_by(angle, MOVE_WAIT, MOVE_SLOW);
    }

    audio_unsubscribe();
    lights_trigger(LIGHTS_SUCCESS, LIGHTS_LOOP);
}
