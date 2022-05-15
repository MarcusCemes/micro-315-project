#include "location.h"

#include <ch.h>
#include <hal.h>
#include <math.h>
#include <stdlib.h>

#include "comms.h"
#include "utils.h"

#define V_SOUND 343     // The speed of sound [m]
#define D_MIC_LR 0.061  // Distance between the L/R microphones [m]

/* == Private functions == */

/** Modulo implementation that maps to the positive domain. */
static float positive_mod(float a, float n)
{
    return a - floorf(a / n) * n;
}

/* == Public functions == */

float calculate_phase_delta(float a, float b)
{
    float phase_delta = b - a;
    return positive_mod(phase_delta + M_PI, M_TWOPI) - M_PI;
}

float loc_estimate_angle(audio_data_t* data)
{
    float phase_right = data->value[A_RIGHT].phase;
    float phase_left = data->value[A_LEFT].phase;
    float phase_delta = calculate_phase_delta(phase_right, phase_left);
    float delta = (V_SOUND / M_TWOPI) * (phase_delta / data->frequency);
    float sin_theta = delta / D_MIC_LR;
    return asinf(sin_theta);
}
