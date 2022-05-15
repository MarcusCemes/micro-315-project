#ifndef LOCATION_H
#define LOCATION_H

#include "audio.h"

/** Calculates the actual phase difference between two phases in [-pi, pi]. */
float calculate_phase_delta(float a, float b);

/**
 * Estimates the angle of the sound wave using the L/R phases.
 *
 * If the phase difference exceeds the maximum possible theoretical
 * value, this function returns the special IEEE 754 NAN value.
 */
float loc_estimate_angle(audio_data_t* data);

#endif
