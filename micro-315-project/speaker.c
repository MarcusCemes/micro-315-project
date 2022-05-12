#include "speaker.h"

#include <audio/play_melody.h>
#include <ch.h>
#include <hal.h>

static const uint16_t speaker_ready_notes[] = {
    NOTE_C5,
    0,
    NOTE_E5,
    0,
};

static const float speaker_ready_tempo[] = {
    18,
    18,
    18,
    18,

};

static const melody_t speaker_tunes[NB_TUNES] = {
    { .notes = speaker_ready_notes,
      .tempo = speaker_ready_tempo,
      .length = sizeof(speaker_ready_notes) / sizeof(uint16_t) },
};

/* == Public functions == */

void speaker_play_tune(speaker_tune_t tune)
{
    playMelody(EXTERNAL_SONG, ML_SIMPLE_PLAY, &speaker_tunes[tune]);
}

void speaker_start(void)
{
    playMelodyStart();
}