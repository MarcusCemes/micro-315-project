#include "audio.h"

#include <arm_const_structs.h>
#include <arm_math.h>
#include <audio/microphone.h>
#include <ch.h>
#include <complex.h>
#include <hal.h>

#include "comms.h"
#include "utils.h"

/** The amount of samples to collect and process at once. */
#define BUFFER_SIZE 1024

/** The amount of samples received expected by the callback function. */
#define CALLBACK_SAMPLES 4 * 160

/**
 * To reduce the amount of data created, this is how many
 * samples will be skipped once the buffer is filled. It takes
 * 7 cycles ot fill the buffer, so a gap of 7 will ensure
 * that only 50% of microphone data is processed.
 */
#define ACQUISITION_GAP 7

/** The order of microphone streams as received by the callback. */
enum Channel
{
    RIGHT,
    LEFT,
    BACK,
    FRONT,
    CHANNELS,
};

/** Synchronise access to the acquisition buffer .*/
static mutex_t _acquisition_lock;
/** Signal that data has been acquired and can be processed. */
static binary_semaphore_t _data_acquired_signal;

/** Pointer to the first free cell in teh acquisition buffer. */
static size_t _acquisition_index = 0;
/** Buffer to hold sampled microphone data. */
static float complex _acquisition[CHANNELS][BUFFER_SIZE];
/** Buffer to hold the calculated magnitudes of data. */
static float _magnitudes[CHANNELS][BUFFER_SIZE];

/** The counter used to skip over certain microphone samples. */
static uint8_t _gap_counter = 0;

/**
 * Finds the peak value of a buffer of float values.
 * @returns the index of the peak value
 */
size_t find_peak_f32(float buffer[], size_t count)
{
    size_t max_index = 0;

    for (size_t i = 0; i < count; ++i)
        if (buffer[i] > buffer[max_index])
            max_index = i;

    return max_index;
}

/**
 * Carries out a in-place hardware-accelerated FFT on the acquired data,
 * transmitting the results of each step of the operation to the remote device.
 */
static void process_audio_fft(void)
{
    chMtxLock(&_acquisition_lock);
    comms_send_buffer("PCM", (uint8_t*)_acquisition[FRONT],
                      sizeof(_acquisition[FRONT][0]) * BUFFER_SIZE);

    // Compute the in-place FFT and magnitude of the FFT result using
    // hardware accelerated instructions on the Cortex-M4 DSP.
    for (uint8_t channel = 0; channel < CHANNELS; ++channel)
    {
        arm_cfft_f32(&arm_cfft_sR_f32_len1024, (uint8_t*)_acquisition[channel], 0, 1);
        arm_cmplx_mag_f32(_acquisition[channel], _magnitudes[channel], BUFFER_SIZE);
    }

    comms_send_buffer("FFT", (uint8_t*)_acquisition[FRONT],
                      sizeof(_acquisition[FRONT][0]) * BUFFER_SIZE);
    comms_send_buffer("MAG", _magnitudes[FRONT], sizeof(_magnitudes[FRONT][0]) * BUFFER_SIZE);
    chMtxUnlock(&_acquisition_lock);
}

/** Calculate the phase difference between left-right, using the front peak. */
static void process_lr_phase(void)
{
    size_t peak_index = find_peak_32(_magnitudes[FRONT], BUFFER_SIZE);
    float phase_l = carg(_acquisition[LEFT][peak_index]);
    float phase_r = carg(_acquisition[RIGHT][peak_index]);
    float phase_delta = phase_r - phase_l;
    comms_send_msg_f("PHASE", "Delta: %f", phase_delta);
}

/** Process the acquired microphone data once the buffers are full. */
static void process_audio(void)
{
    process_audio_fft();
    process_lr_phase();
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

    // Although extremely unlikely, if the processing thread has not yet
    // finished with teh buffer, reset the index to ensure recent continuous data.
    if (!chMtxTryLock(&_acquisition_lock))
    {
        _acquisition_index = 0;
        return;
    }

    // Read data sequentially, splitting the mixed channels into their
    // respective complex float buffers.
    size_t samples_to_append = min(data_count / CHANNELS, BUFFER_SIZE - _acquisition_index);
    for (size_t i = 0; i < samples_to_append; ++i)
        for (uint8_t channel = 0; channel < CHANNELS; ++channel)
            _acquisition[channel][_acquisition_index + i] = data[CHANNELS * i + channel];

    _acquisition_index += samples_to_append;
    chMtxUnlock(&_acquisition_lock);

    // Once the buffer is full, notify the processing thread
    if (_acquisition_index == BUFFER_SIZE)
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

    while (!chThdShouldTerminateX())
    {
        chBSemWait(&_data_acquired_signal);
        process_audio();
    }
}

void init_audio(void)
{
    chMtxObjectInit(&_acquisition_lock);
    chBSemObjectInit(&_data_acquired_signal, true);
}

void audio_start(void)
{
    mic_start(audio_data_ready_cb);
    (void)chThdCreateStatic(processing_stack, sizeof(processing_stack), LOWPRIO, processing_thread,
                            NULL);
}
