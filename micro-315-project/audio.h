#ifndef AUDIO_H
#define AUDIO_H

#include <complex.h>

/* == Definitions == */

/** The amount of samples to collect and process at once. */
#define AUDIO_BUFFER_SIZE 1024

/** The order of microphone streams as received by the callback. */
typedef enum
{
    RIGHT,
    LEFT,
    BACK,
    FRONT,
    CHANNELS,
} Channel;

typedef float audio_buffer_t[CHANNELS][AUDIO_BUFFER_SIZE];
typedef complex float audio_buffer_complex_t[CHANNELS][AUDIO_BUFFER_SIZE];

typedef struct
{
    audio_buffer_t pcm;
    audio_buffer_complex_t fft;
    audio_buffer_t magnitudes;
} audio_data_t;

/* == Public functions = */

/**
 * Register a new data listener, ensuring that the processing thread
 * is active. This listener must be unregsitered when no longer needed.
 */
void audio_register(void);

/**
 * Unregisters a data listener. If there are no more listeners, the
 * audio thread will go into a low-power state until a listener is
 * registered.
 */
void audio_unregister(void);

/**
 * Blocks until fresh audio data is ready. Will block indefinitely if
 * no listeners were registered.
 */
void audio_wait(void);

/** Borrow a read-only lock and reference to the audio data. */
audio_data_t* audio_data_borrow(void);

/**
 * Return a read-only reference, allowing the audio thread to acquire
 * a write lock on the data.
 */
void audio_data_return(void);

/** Initialize audio-related resources and start the processing thread. */
void audio_init(void);

#endif
