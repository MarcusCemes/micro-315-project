#include "speaker.h"

#include <audio/audio_thread.h>
#include <audio/play_melody.h>
#include <ch.h>
#include <fat.h>
#include <hal.h>
#include <sdio.h>

/* == Definitions == */

#define SAMPLE_RATE 16000  // 16 kHz

typedef enum
{
    VOL_LOW = 4,
    VOL_MED = 8,
    VOL_HIGH = 12,
} volume_t;

typedef struct
{
    char* filename;
    volume_t volume;
} audio_file_t;

/* == Constants == */

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

static const melody_t speaker_tunes[SPKR_NB_TUNES] = {
    { .notes = speaker_ready_notes,
      .tempo = speaker_ready_tempo,
      .length = sizeof(speaker_ready_notes) / sizeof(uint16_t) },
};

/* == Static variables == */

static bool sd_connected = false;

/* == Private functions == */

static audio_file_t audio_new(char* filename, volume_t volume)
{
    return (audio_file_t){ .filename = filename, .volume = volume };
}

static audio_file_t get_wav_filename(speaker_wav_t wav)
{
    switch (wav)
    {
        case SPKR_WAVE_BEEP_01:
            return audio_new("/beep_01.wav", VOL_MED);
        case SPKR_WAV_FUTURISTIC_SIREN_01:
            return audio_new("/futuristic_siren_01.wav", VOL_HIGH);
        case SPKR_WAV_PING_01:
            return audio_new("/ping_01.wav", VOL_MED);
        case SPKR_WAV_STARTUP:
            return audio_new("/startup.wav", VOL_LOW);
        case SPKR_WAV_TAUNT:
            return audio_new("/taunt.wav", VOL_MED);

        default:
            return audio_new(NULL, VOL_LOW);
    }
}

/* == Public functions == */

void speaker_play_melody(song_selection_t choice, play_melody_option_t option)
{
    playMelody(choice, option, NULL);
}

void speaker_play_tune(speaker_tune_t tune)
{
    playMelody(EXTERNAL_SONG, ML_SIMPLE_PLAY, &speaker_tunes[tune]);
}

void speaker_play_wav(speaker_wav_t wav, playSoundFileOption_t mode, bool wait_finished)
{
    if (!sd_connected)
        return;

    audio_file_t file = get_wav_filename(wav);
    if (file.filename == NULL)
        return;

    setSoundFileVolume(file.volume);
    playSoundFile(file.filename, mode);

    if (wait_finished)
        waitSoundFileHasFinished();
}

void speaker_stop_wav(void)
{
    stopCurrentSoundFile();
    waitSoundFileHasFinished();
}

void speaker_start(void)
{
    sdio_start();          // SD card IO
    dac_start();           // Digital to analog converter
    playMelodyStart();     // Melody threads
    playSoundFileStart();  // WAV threads

    sd_connected = !sdio_connect();
}
