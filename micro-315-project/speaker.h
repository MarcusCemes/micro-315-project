#ifndef SPEAKER_H
#define SPEAKER_H

#include <audio/play_sound_file.h>

typedef enum
{
    SPKR_TUNE_READY,
    SPKR_NB_TUNES,
} speaker_tune_t;

typedef enum
{
    SPKR_WAV_BEEP,
    SPKR_WAVE_BEEP_01,
    SPKR_NB_WAVS,
} speaker_wav_t;

/** Play one of the custom speaker tunes. */
void speaker_play_tune(speaker_tune_t tune);

/** Play one of the custom WAV files. */
void speaker_play_wav(speaker_wav_t wav, playSoundFileOption_t mode, bool wait_finished);

/** Start the melody playing thread. */
void speaker_start(void);

#endif
