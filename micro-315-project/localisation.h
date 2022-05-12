#ifndef LOCALISATION_H
#define LOCALISATION_H

#include "audio.h"

/** Estimates the angle of the sound wave using the L/R phases. */
float estimate_angle(audio_data_t* data);

#endif
