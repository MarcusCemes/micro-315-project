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
 * To reduce the amount of data created, this is how many
 * samples will be skipped once the buffer is filled. It takes
 * 7 cycles ot fill the buffer, so a gap of 7 will ensure
 * that only 50% of microphone data is processed (so every ~128ms).
 */
#define ACQUISITION_GAP 7
/**
 * The number of data listeners. Audio processing stops if there
 * are no registered listeners.
 */
static semaphore_t _data_listeners;

/** Synchronised read/write access to the processed data. */
static rw_lock_t _data_lock;

/** Signal to any waiting threads that new processed data is ready. */
static semaphore_t _data_ready;

/** Synchronise access to the acquisition buffer. */
static mutex_t _acquisition_lock;

/** The counter used to skip over certain microphone samples. */
static uint8_t _gap_counter = 0;

/** Signal to the audio processing thread that new data is ready for procesing. */
static binary_semaphore_t _data_acquired_signal;

/** Points to the free index of the acquisition buffer, indicating its filled capacity. */
static size_t _acquisition_index = 0;

/** Buffer that holds sampled microphone data as it's acquired. */
static audio_buffer_t _acquisition;

/**
 * Buffers that hold the complex FFT transform data as well as
 * the computed magnitude data. Accessing this variable requires
 * that the _data_lock be acquired.
 */
static audio_data_t _audio_data;

/**
 * Finds the peak value of a buffer of float values.
 * @returns the index of the peak value
 */
static size_t find_peak_f32(float buffer[], size_t count)
{
    size_t max_index = 0;

    for (size_t i = 0; i < count; ++i)
        if (buffer[i] > buffer[max_index])
            max_index = i;

    return max_index;
}

/** Carries out a in-place hardware-accelerated FFT on PCM data. */
static void process_audio_fft(audio_data_t* data)
{
    // Compute the in-place FFT and magnitude of the FFT result using
    // hardware accelerated instructions on the Cortex-M4 DSP.
    for (uint8_t channel = 0; channel < CHANNELS; ++channel)
    {
        arm_cfft_f32(&arm_cfft_sR_f32_len1024, (float*)data->fft[channel], 0, 1);
        arm_cmplx_mag_f32((float*)data->fft[channel], (float*)data->magnitudes[channel],
                          AUDIO_BUFFER_SIZE);
    }
}

/** Process the acquired microphone data once the buffers are full. */
static void process_audio(void)
{
    chMtxLock(&_acquisition_lock);
    rw_write_lock(&_data_lock);

    // Copy the data to release the acquisition buffer as soon as possible
    // Also convert float values to complex floats for processing
    for (size_t i = 0; i < CHANNELS; ++i)
        for (size_t j = 0; j < AUDIO_BUFFER_SIZE; ++j)
            _audio_data.fft[i][j] = _audio_data.pcm[i][j] = _acquisition[i][j];

    chMtxUnlock(&_acquisition_lock);

    process_audio_fft(&_audio_data);
    rw_write_unlock(&_data_lock);
}

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
static void audio_data_ready_cb(int16_t* data, uint16_t data_count)
{
    chDbgCheck(data_count == CALLBACK_SAMPLES);

    if (_gap_counter > 0)
    {
        --_gap_counter;
        return;
    }

    // If there are no listeners, don't record data.
    chSysLock();
    size_t listeners = chSemGetCounterI(&_data_listeners);
    chSysUnlock();

    if (listeners == 0)
        return;

    bool locked = chMtxTryLock(&_acquisition_lock);

    // If the processing thread has still not finished with the acquisition buffer
    // (the mutex could not be acquired), start recording from the beginning.
    if (!locked)
        _acquisition_index = 0;

    // Read time-sequential data, splitting mixed audio channels into
    // their respective buffers
    size_t samples_to_append = min(data_count / CHANNELS, AUDIO_BUFFER_SIZE - _acquisition_index);
    for (size_t i = 0; i < samples_to_append; ++i)
        for (uint8_t channel = 0; channel < CHANNELS; ++channel)
            _acquisition[channel][_acquisition_index + i] = data[CHANNELS * i + channel];

    _acquisition_index += samples_to_append;

    if (locked)
        chMtxUnlock(&_acquisition_lock);

    // Notify the processing thread that the acquisition buffer is full
    if (_acquisition_index == AUDIO_BUFFER_SIZE)
    {
        chBSemSignal(&_data_acquired_signal);
        _acquisition_index = 0;
        _gap_counter = ACQUISITION_GAP;
    }
}

static THD_WORKING_AREA(processing_stack, 1024);
static THD_FUNCTION(processing_thread, arg)  // @suppress("No return")
{
    (void)arg;
    chRegSetThreadName(__FUNCTION__);

    while (!chThdShouldTerminateX())
    {
        // Run if there are active listeners
        chSemWait(&_data_listeners);
        chSemSignal(&_data_listeners);

        process_audio();

        // Notify any waiting listeners
        chSemReset(&_data_ready, 0);
    }
}

/* == Public functions == */

void audio_register()
{
    chSemSignal(&_data_listeners);
}

void audio_unregister()
{
    chSemWait(&_data_listeners);
}

void audio_wait()
{
    chSemWait(&_data_ready);
}

audio_data_t* audio_data_borrow()
{
    rw_read_lock(&_data_lock);
    return &_audio_data;
}

void audio_data_return()
{
    rw_read_unlock(&_data_lock);
}

void audio_init(void)
{
    rw_init(&_data_lock);
    chSemObjectInit(&_data_ready, 0);
    chSemObjectInit(&_data_listeners, 0);
    chMtxObjectInit(&_acquisition_lock);
    chBSemObjectInit(&_data_acquired_signal, true);

    mic_start(audio_data_ready_cb);

    (void)chThdCreateStatic(processing_stack, sizeof(processing_stack), LOWPRIO, processing_thread,
                            NULL);
}
