#ifndef LOCATION_H
#define LOCATION_H

#include "audio.h"

/** Estimates the angle of the sound wave using the L/R phases. */
float loc_estimate_angle(audio_data_t* data);

#endif
