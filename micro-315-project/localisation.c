#include "localisation.h"

#include <ch.h>
#include <hal.h>
#include <math.h>
#include <stdlib.h>

#include "audio.h"

#define V_SOUND 343
#define D_MIC_LR 58e-3f

/** Normalises an rangle (radians) to the I and IV quadrants. */
static float normalise_phase(float number)
{
    return fmodf(number, M_PI);
}

float estimate_angle(audio_data_t* data)
{
    float phase_left = normalise_phase(data->value[A_LEFT].phase);
    float phase_right = normalise_phase(data->value[A_RIGHT].phase);
    float delta_phase = phase_right - phase_left;
    float delta = (V_SOUND / M_TWOPI) * (delta_phase / data->frequency);
    float sin_theta = delta / D_MIC_LR;
    float theta = asinf(sin_theta);
    return theta;
}
