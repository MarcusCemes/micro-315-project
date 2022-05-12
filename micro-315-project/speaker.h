#ifndef SPEAKER_H
#define SPEAKER_H

typedef enum
{
    SPEAKER_READY,
    NB_TUNES,
} speaker_tune_t;

/** Play one of the custom speaker tunes. */
void speaker_play_tune(speaker_tune_t tune);

/** Start the melody playing thread. */
void speaker_start(void);

#endif