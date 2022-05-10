#include "localisation.h"

#include <ch.h>
#include <hal.h>
#include <math.h>
#include <stdlib.h>

#include "audio.h"

#define V_SOUND 343
#define D_MIC_LR 58e-3f

#define PI_AS_DEG 180

/** Normalises an rangle (radians) to the I and IV quadrants. */
static float normalise_phase(float number)
{
    return fmodf(number, M_PI);
}

static float rad2deg(float radians)
{
    return radians * PI_AS_DEG / M_PI;
}

float estimate_angle(audio_data_t* data)
{
    float phase_left = normalise(data->value[A_LEFT].phase);
    float phase_right = normalise(data->value[A_RIGHT].phase);
    float phase_left_right = data->value[A_RIGHT].phase - data->value[A_LEFT].phase;

    float sin_theta = V_SOUND / (M_TWOPI * data->frequency);
    float theta = asinf(sin_theta);
    return theta;
}