#ifndef AUDIO_H
#define AUDIO_H

#include <complex.h>
#include <stdint.h>

/* == Definitions == */

/** A list of audio channels, in the order as received from the microphones. */
typedef enum
{
    A_RIGHT,
    A_LEFT,
    A_BACK,
    A_FRONT,
    A_CHANNELS,
} audio_channel_t;

/** The polar components associated with a complex number. */
typedef struct
{
    float magnitude;
    float phase;
} audio_polar_t;

/**
 * Contains the magnitude and phase of each channel for a particular
 * frequency. The frequency is chosen as the peak magnitude of the
 * A_BACK channel.
 */
typedef struct
{
    size_t index;
    float frequency;
    audio_polar_t value[A_CHANNELS];
} audio_data_t;

/* == Public functions = */

/**
 * Register a new event listener. The processing thread will be scheduled
 * while there is at least one active listener. Unregister the listener
 * with `audio_unsubscribe()`.
 */
void audio_subscribe(void);

/**
 * Unregisters a single event listener. If there are no active event
 * listeners, the processing thread halt audio recording and processing.
 */
void audio_unsubscribe(void);

/** Wait until the processing thread signals that new data is ready. */
void audio_wait(void);

/** Acquire a read-lock on the audio data, returning a reference to it. */
audio_data_t* audio_data_borrow(void);

/** Release a previously-acquired read-lock. */
void audio_data_return(void);

/** Initialize audio-related resources. */
void audio_init(void);

/** Start receiving microphone data and start the audio processing thread. */
void audio_start(void);

#endif
