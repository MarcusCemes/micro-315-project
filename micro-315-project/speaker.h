#ifndef SPEAKER_H
#define SPEAKER_H

#include <audio/play_melody.h>
#include <audio/play_sound_file.h>

typedef enum
{
    SPKR_TUNE_READY,
    SPKR_NB_TUNES,
} speaker_tune_t;

typedef enum
{
    SPKR_WAVE_BEEP_01,
    SPKR_WAV_FUTURISTIC_SIREN_01,
    SPKR_WAV_PING_01,
    SPKR_WAV_STARTUP,
    SPKR_WAV_TAUNT,
    SPKR_NB_WAVS,
} speaker_wav_t;

/** Plays one of the e-puck2 melodies. */
void speaker_play_melody(song_selection_t choice, play_melody_option_t option);

/** Play one of the custom speaker tunes. */
void speaker_play_tune(speaker_tune_t tune);

/** Play one of the custom WAV files. */
void speaker_play_wav(speaker_wav_t wav, playSoundFileOption_t mode, bool wait_finished);

/** Stops anything that's playing and waits until the thread is ready again. */
void speaker_stop_wav(void);

/** Start the melody playing thread. */
void speaker_start(void);

#endif
