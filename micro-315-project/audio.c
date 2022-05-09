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

/** The size of an audio buffer that is processed all at once. */
#define AUDIO_BUFFER_SIZE 1024

/** Microphone sample rate is 16 kHz. */
#define SAMPLE_RATE 16000

typedef float audio_buffer_t[AUDIO_BUFFER_SIZE];
typedef complex float audio_buffer_complex_t[AUDIO_BUFFER_SIZE];
typedef audio_buffer_t audio_buffers_t[A_CHANNELS];

/* == Listeners == */

/**
 * The number of subscribed event listeners. This is used to determine
 * whether the audio should be recorded and processed.
 */
static size_t _listeners;

/**
 * Threads that are registered as event listeners may wait for an
 * audio data event. When processing is completed, all enqueued threads
 * will be enqueued. Waiting without being registered may result in a deadlock.
 */
static threads_queue_t _listeners_waiting;

/* == PCM & processing == */

/** The amount of data already stored in the audio buffer. */
static size_t _pcm_index = 0;

/** Stores the PCM data of seperated microphone channels. */
static audio_buffers_t _pcm_buffer;

/** Signal the processing thread to start processing the _pcm buffer. */
static binary_semaphore_t _process_signal;

/** Whether the processing thread is busy. Used as cheap synchronisation. */
static bool _processing_active = false;

/** Workspace for calculating the FFT. */
static audio_buffer_complex_t fft_buffer;

/* == Audio data == */

/**
 * Holds computed peak magnitude and the corresponding phase.
 * Access to this buffer requires an _audio_data_lock lock.
 */
static audio_data_t _audio_data;

/** Inter-thread readers-writer lock for _audio_data. */
static rw_lock_t _audio_data_lock;

/* === Audio recording === */

static bool audio_buffer_full(void)
{
    return _pcm_index == AUDIO_BUFFER_SIZE;
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
static void pcm_ready_cb(int16_t* pcm, uint16_t data_count)
{
    chDbgCheck(data_count == CALLBACK_SAMPLES);

    // Simple check should not require atomics or locks, as the
    // data acquisition is simply reset and can be restarted
    if (_listeners == 0 || _processing_active)
    {
        _pcm_index = 0;
        return;
    }

    // The number of samples that can be appended to the PCM buffer
    size_t samples_to_append = min(data_count / A_CHANNELS, AUDIO_BUFFER_SIZE - _pcm_index);

    // Read time-sequential data, seperating each channel
    for (size_t i = 0; i < samples_to_append; (++_pcm_index, ++i))
        for (uint8_t channel = 0; channel < A_CHANNELS; ++channel)
            _pcm_buffer[channel][_pcm_index] = pcm[i * A_CHANNELS + channel];

    if (audio_buffer_full())
    {
        _processing_active = true;
        _pcm_index = 0;

        chSysLock();
        chBSemSignalI(&_process_signal);
        chSchRescheduleS();
        chSysUnlock();
    }
}

/* === Audio processing === */

/** Copies PCM buffer data to a PCM complex buffer (zero imaginary component). */
static void pcm_to_complex(audio_buffer_t pcm, audio_buffer_complex_t buffer)
{
    for (size_t i = 0; i < AUDIO_BUFFER_SIZE; ++i)
        buffer[i] = pcm[i];
}

/** Calculates the FFT of a complex buffer contaning real PCM data. */
static void process_fft(audio_buffer_complex_t buffer)
{
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, (float*)buffer, false, true);
}

/** Calculates the magnitude of every index of an FFT. */
static void process_magnitude(audio_buffer_complex_t fft, audio_buffer_t magnitude)
{
    arm_cmplx_mag_f32((float*)fft, (float*)magnitude, AUDIO_BUFFER_SIZE);
}

/** Finds the index of the maximum float value in an audio buffer. */
static void find_peak_index(audio_buffer_t magnitude, size_t* index)
{
    float value;
    arm_max_f32((float*)magnitude, AUDIO_BUFFER_SIZE, &value, (uint32_t*)index);
}

/** Calculates the frequency associated with a FFT bin index. */
static float bin_index_to_freq(size_t index)
{
    return (float)index * SAMPLE_RATE / AUDIO_BUFFER_SIZE;
}

/**
 * Calculates the magnitude and phase of a PCM channel using
 * hardware-accelerated instructions on the Cortex-M4 DSP.
 *
 * If `write_peak` is true, the FFT buffer will also be analysed
 * to find the maximum magnitude and associated index. This index
 * will be written to the _audio_data `peak` field. This operation
 * wil overwrite the original PCM buffer, as it's no longer needed!
 *
 * If `write_peak` is false, the existing peak index in `_audio_data`
 * will be used.
 *
 * The resulting magnitude and phase are written directly to `_audio_data`.
 */
static void analyse_audio_channel(audio_channel_t channel, bool write_peak)
{
    pcm_to_complex(_pcm_buffer[channel], fft_buffer);
    process_fft(fft_buffer);

    audio_polar_t* value = &_audio_data.value[channel];

    if (write_peak)
    {
        process_magnitude(fft_buffer, _pcm_buffer[channel]);
        find_peak_index(_pcm_buffer[channel], &_audio_data.index);
        value->magnitude = _pcm_buffer[channel][_audio_data.index];
    }
    else
    {
        value->magnitude = cabsf(fft_buffer[_audio_data.index]);
    }

    value->phase = cargf(fft_buffer[_audio_data.index]);
}

/**
 * Efficient audio processing that performs a FFT and magnitude analysis
 * on the PCM data.
 *
 * Populates the _audio_data buffer with peak index, associated frequency
 * and the magnitude and phase of each channel at that index.
 *
 * The peak index is selected as the index of the maximum of the A_BACK
 * channel magnitude buffer. This index is then used for the remaining
 * channels to guarantee that they are analysed at the same frequency.
 */
static void process_audio(void)
{
    chDbgCheck(_processing_active == true);
    rw_write_lock(&_audio_data_lock);

    analyse_audio_channel(A_BACK, true);
    analyse_audio_channel(A_RIGHT, false);
    analyse_audio_channel(A_LEFT, false);
    analyse_audio_channel(A_FRONT, false);

    _audio_data.frequency = bin_index_to_freq(_audio_data.index);

    rw_write_unlock(&_audio_data_lock);
}

static THD_WORKING_AREA(processing_stack, 1024);
static THD_FUNCTION(processing_thread, arg)
{
    (void)arg;
    chRegSetThreadName(__FUNCTION__);

    while (!chThdShouldTerminateX())
    {
        chBSemWait(&_process_signal);

        process_audio();
        _processing_active = false;

        chSysLock();
        chThdDequeueAllI(&_listeners_waiting, MSG_OK);
        chSchRescheduleS();
        chSysUnlock();
    }
}

/* === Public functions === */

void audio_subscribe()
{
    chSysLock();
    _listeners += 1;
    chSysUnlock();
}

void audio_unsubscribe()
{
    chSysLock();
    _listeners -= 1;
    chSysUnlock();
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
    chThdQueueObjectInit(&_listeners_waiting);
    chBSemObjectInit(&_process_signal, true);
    rw_init(&_audio_data_lock);
}

void audio_start(void)
{
    mic_start(pcm_ready_cb);
    chThdCreateStatic(processing_stack, sizeof(processing_stack), LOWPRIO, processing_thread, NULL);
}
