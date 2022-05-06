#include "audio.h"

#include <arm_const_structs.h>
#include <arm_math.h>
#include <audio/microphone.h>
#include <ch.h>
#include <hal.h>

#include "comms.h"
#include "utils.h"

/** The amount of samples received expected by the callback function. */
#define CALLBACK_SAMPLES 4 * 160

/**
 * A common lock to synchronise access to static variables.
 * This should only be used for quick access, such as checking a boolean.
 */
static mutex_t _lock;

/* == Listeners == */

/**
 * The number of external threads that are registered as listeners
 * for new data events. Audio acquisition and processing stops if
 * there are no registered listeners.
 */
static size_t _listeners;

/** All listeners that are currently waiting for a data event. */
static threads_queue_t _listeners_waiting;

/* == PCM data == */

/** Buffer for recorded PCM data. */
static audio_buffer_t _pcm;

/** Points to the free index of the acquisition buffer, indicating its filled capacity. */
static size_t _pcm_index = 0;

/** Synchronise access to the _pcm_ recording buffer. */
static mutex_t _pcm_lock;

/** Whether the PCM callback was called again before the previous completed. */
static bool _pcm_discontinuous = false;

/** Whether the _pcm buffer is in use by the processing thread. */
static bool _pcm_processing = false;

/* == Audio data == */

/**
 * Holds the PCM, FFT and Magnitude computed data. This is used during
 * computation and then broadcast to other threads. Access requires a valid
 * lock on _audio_data_lock.
 */
static audio_data_t _audio_data;

/** Synchronise access to _audio_data. */
static rw_lock_t _audio_data_lock;

/** Signal the processing thread to start processing the _pcm buffer. */
static binary_semaphore_t _process_signal;

/* === Audio recording === */

/**
 * Callback when digital microphone data is ready. The microphones
 * are sampled at 16kHz with windows of 10ms, resulting in 160 samples
 * for each of the four microphones. The data buffer receives 640
 * samples in total.
 *
 * The microphone data is in time order, the four microphone streams
 * must be seperated (R0, L0, B0, F0, R1, L1, ...) for the FFT function.
 *
 * It takes 7 buffers to completely fill the processing buffer.
 */
static void pcm_ready_cb(int16_t* pcm, uint16_t data_count)
{
    chDbgCheck(data_count == CALLBACK_SAMPLES);

    chMtxLock(&_lock);
    bool should_record = _listeners != 0 && !_pcm_processing;
    bool discontinuous = _pcm_discontinuous;
    chMtxUnlock(&_lock);

    if (!should_record)
        return;

    if (!chMtxTryLock(&_pcm_lock))
    {
        chMtxLock(&_lock);
        _pcm_discontinuous = true;
        chMtxUnlock(&_lock);
        return;
    }

    if (discontinuous)
        _pcm_index = 0;

    size_t samples_to_append = min(data_count / CHANNELS, AUDIO_BUFFER_SIZE - _pcm_index);

    // Read time-sequential data, splitting mixed audio channels into
    // their respective buffers
    for (size_t i = 0; i < samples_to_append; (++_pcm_index, ++i))
        for (uint8_t channel = 0; channel < CHANNELS; ++channel)
            _pcm[channel][_pcm_index] = pcm[CHANNELS * i + channel];

    if (_pcm_index == AUDIO_BUFFER_SIZE)
    {
        chMtxLock(&_lock);
        _pcm_processing = true;
        chMtxUnlock(&_lock);
        chBSemSignal(&_process_signal);

        _pcm_index = 0;
    }
}

/* === Audio processing === */

/**
 * Compute the in-place FFT and magnitude of PCM data using
 * hardware accelerated instructions on the Cortex-M4 DSP.
 */
static void process_audio_fft(audio_data_t* data)
{
    for (uint8_t channel = 0; channel < CHANNELS; ++channel)
    {
        arm_cfft_f32(&arm_cfft_sR_f32_len1024, (float*)data->fft[channel], false, true);
        arm_cmplx_mag_f32((float*)data->fft[channel], (float*)data->magnitudes[channel],
                          AUDIO_BUFFER_SIZE);
    }
}

/** Process the acquired microphone data once the buffers are full. */
static void process_audio(void)
{
    rw_write_lock(&_audio_data_lock);

    // Copy the data to release the acquisition buffer as soon as possible
    // Also convert float values to complex floats for processing
    for (size_t i = 0; i < CHANNELS; ++i)
        for (size_t j = 0; j < AUDIO_BUFFER_SIZE; ++j)
            _audio_data.fft[i][j] = _audio_data.pcm[i][j] = _pcm[i][j];

    process_audio_fft(&_audio_data);

    rw_write_unlock(&_audio_data_lock);
}

static THD_WORKING_AREA(processing_stack, 1024);
static THD_FUNCTION(processing_thread, arg)  // @suppress("No return")
{
    (void)arg;
    chRegSetThreadName(__FUNCTION__);

    while (!chThdShouldTerminateX())
    {
        chBSemWait(&_process_signal);

        process_audio();

        // Allow the PCM callback to write to the buffer again
        _pcm_processing = false;

        chSysLock();
        chThdDequeueAllI(&_listeners_waiting, MSG_OK);
        chSchRescheduleS();
        chSysUnlock();
    }
}

/* === Public functions === */

void audio_register()
{
    chMtxLock(&_lock);
    _listeners += 1;
    chMtxUnlock(&_lock);
}

void audio_unregister()
{
    chMtxLock(&_lock);
    _listeners -= 1;
    chMtxUnlock(&_lock);
}

void audio_wait()
{
    chSysLock();
    chThdEnqueueTimeoutS(&_listeners_waiting, TIME_INFINITE);
    chSysUnlock();
}

audio_data_t* audio_data_borrow()
{
    rw_read_lock(&_audio_data_lock);
    return &_audio_data;
}

void audio_data_return()
{
    rw_read_unlock(&_audio_data_lock);
}

void audio_init(void)
{
    chMtxObjectInit(&_lock);
    chMtxObjectInit(&_pcm_lock);
    chThdQueueObjectInit(&_listeners_waiting);
    chBSemObjectInit(&_process_signal, true);

    rw_init(&_audio_data_lock);

    mic_start(pcm_ready_cb);

    (void)chThdCreateStatic(processing_stack, sizeof(processing_stack), LOWPRIO, processing_thread,
                            NULL);
}
