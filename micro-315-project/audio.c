#include "audio.h"

#include <arm_const_structs.h>
#include <arm_math.h>
#include <audio/microphone.h>
#include <ch.h>
#include <complex.h>
#include <hal.h>

#include "comms.h"
#include "utils.h"

#define BUFFER_SIZE 1024
#define CALLBACK_SAMPLES 4 * 160

enum Channel
{
    RIGHT,
    LEFT,
    BACK,
    FRONT,
    CHANNELS,
};

/**
 * A single microphone channel holding BUFFER_SIZE complex float
 * samples. This is the native type that is accepted by
 * the hardware accelerated FFT functions.
 */
typedef complex float channel_buffer_t[BUFFER_SIZE];

/**
 * An array of channel buffers, in the order that the microphones
 * are passed to audio_data_ready_cb() (see enum Channel).
 */
typedef channel_buffer_t channel_buffers_t[CHANNELS];

static mutex_t buffer_lock;
static binary_semaphore_t process_signal;

static size_t buffer_index = 0;
static float complex acquisition[CHANNELS][BUFFER_SIZE];
static float complex processing[CHANNELS][BUFFER_SIZE];

/**
 * Assumes that the buffer has already been transformed from
 * a complex number into a pair of magnitude/phase floats.
 */
size_t find_peak(complex float buffer[], size_t count)
{
    size_t max_index = 0;

    for (size_t i = 0; i < count; ++i)
        if (creal(buffer[i]) > creal(buffer[i]))
            max_index = i;

    return max_index;
}

static void process_audio(void)
{
    // Only send a subset of results back to the computer
    static uint8_t send_counter = 0;
    send_counter = (send_counter + 1) % 64;

    if (send_counter != 1)
        return;

    chMtxLock(&buffer_lock);

    // Generate a SINE wave
    //    for (size_t i = 0; i < BUFFER_SIZE; ++i) {
    //        acquisition[FRONT][i] = sinf(0.4 * i);
    //    }

    comms_send_buffer("PCM", (uint8_t*)acquisition[FRONT], BUFFER_SIZE);

    for (uint8_t channel = 0; channel < CHANNELS; ++channel)
    {
        // In-place FFT function using hardware acceleration (Cortex M4 DSP)
        arm_cfft_f32(&arm_cfft_sR_f32_len1024, (uint8_t*)acquisition[channel], 0, 1);

        if (channel == FRONT)
            comms_send_buffer("FFT", (uint8_t*)acquisition[FRONT], BUFFER_SIZE);

        // Compute the magnitude and phase
        arm_cmplx_mag_f32((float*)acquisition[channel], (float*)processing[channel], BUFFER_SIZE);

        if (channel == FRONT)
            comms_send_buffer("MAG/PHA", (float*)processing[FRONT], BUFFER_SIZE);
    }

    chMtxUnlock(&buffer_lock);

    //    complex float peak_values[CHANNELS];
    //
    //    for (uint8_t channel = 0; channel < CHANNELS; ++channel)
    //    {
    //        size_t peak_index = find_peak(&processing[channel][128], BUFFER_SIZE);
    //        peak_values[channel] = processing[channel][peak_index];
    //
    //        float magnitude = creal(peak_values[channel]), phase = cimag(peak_values[channel]);
    //
    //        if (send_counter == 0)
    //            comms_send_msg_f("AUDIO", "Channel: %i  Magnitude: %f  Phase: %f", channel,
    //            magnitude,
    //                             phase);
    //    }
    //
    //    complex float diff = peak_values[RIGHT] - peak_values[LEFT];
    //    float mag_diff = creal(diff), phase_diff = cimag(diff);
    //
    //    if (send_counter == 0)
    //        comms_send_msg_f("AUDIO", "Magnitude delta: %f  Phase delta: %f", mag_diff,
    //        phase_diff);
}

/**
 * Callback when digital microphone data is ready. The microphones
 * are sampled at 16kHz with windows of 10ms, resulting in 160 samples
 * for each of the four microphones. The data buffer receives 640
 * samples in total.
 *
 * The microphone is in time order, the four microphone streams must
 * be seperated (R0, L0, B0, F0, R1, L1, ...) for the FFT function.
 */
static void audio_data_ready_cb(int16_t* data, uint16_t data_count)
{
    chDbgAssert(data_count == CALLBACK_SAMPLES, "Wrong number of audio samples!");

    // The processing thread may not have finished yet
    if (!chMtxTryLock(&buffer_lock))
    {
        buffer_index = 0;  // reset to ensure continuous data
        return;
    }

    size_t samples_to_append = min(data_count, BUFFER_SIZE - buffer_index);
    size_t stop_index = buffer_index + samples_to_append;

    // Transform the values into complex floats and append to the buffer
    for (; buffer_index < stop_index; ++buffer_index)
        for (uint8_t channel = 0; channel < CHANNELS; ++channel)
            acquisition[channel][buffer_index] = data[buffer_index + channel];

    chMtxUnlock(&buffer_lock);

    if (buffer_index == BUFFER_SIZE)
    {
        chBSemSignal(&process_signal);
        buffer_index = 0;
    }
}

static THD_WORKING_AREA(processing_stack, 1024);
static THD_FUNCTION(processing_thread, arg)  // @suppress("No return")
{
    (void)arg;

    while (!chThdShouldTerminateX())
    {
        chBSemWait(&process_signal);
        process_audio();
    }
}

void init_audio(void)
{
    chMtxObjectInit(&buffer_lock);
    chBSemObjectInit(&process_signal, true);
    mic_start(audio_data_ready_cb);
    (void)chThdCreateStatic(processing_stack, sizeof(processing_stack), HIGHPRIO, processing_thread,
                            NULL);
}
