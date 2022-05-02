#include "comms.h"

#include <ch.h>
#include <hal.h>
#include <stdarg.h>

#include "utils.h"
#include "vendor/mpack.h"

#define COMMS_BUFFER_SIZE 256
#define FORMAT_BUFFER_SIZE 256

/** Ensures that one one thread can send an uninterrupted message. */
static mutex_t _comms_lock;
/** A statically allocated buffer of reasonable size for serialization. */
static uint8_t _comms_buffer[COMMS_BUFFER_SIZE];

/** Debug assertion. */
static bool initialised = false;

/** Start Frame Delimiter character. */
#define SFD 0x7E
/** End of Transmission character. */
#define ETX 0x03
/** The escape character. */
#define ESC 0x7D
/** A magic constant for concealing uncaught escaped characters. */
#define ESC_XOR 0x20

/** The maximum payload size (three byte integer). */
#define MAX_SIZE 16777216

/** Provide byte-level access to a uint16_t. */
union payload_size_t
{
    size_t integer;
    uint8_t bytes[4];
};

/**
 * Attempts to transmit a single byte over the serial port SD3.
 * @returns true if the transmission was successful
 */
static bool try_send(uint8_t byte)
{
    msg_t msg = chSequentialStreamPut(&SD3, byte);
    return msg == STM_OK;
}

/**
 * Sends a 4-byte header indicating the start of a new data frame,
 * composed of the SDF (1 byte) and a size hint (3 bytes).
 *
 * The hint must be smaller than or equal to the size of the payload.
 * It's present to allow the receiver to efficiently read at least that
 * many bytes, before checking each subsequent byte for the ETX delimiter.
 * If the size is unknown, provide a size_hint of 0.
 *
 * @returns true if the transmission was successful
 */
static bool transmit_start(size_t size_hint)
{
    union payload_size_t size = { .integer = min(size_hint, MAX_SIZE) };

    return try_send(SFD) && try_send(size.bytes[2]) && try_send(size.bytes[1]) &&
           try_send(size.bytes[0]);
}

/**
 * Transmits the ETX character, signifying the end of transmission.
 * @returns true if the transmission was successful
 */
static bool transmit_end(void)
{
    return try_send(ETX);
}

/**
 * Any instances of the SFD, ETX or ESC in the payload will
 * be prepended with the escape character (0x7D) and also
 * transformed with an XOR with the constant 0x20.
 * This ensures that the SFD never appears in the payload,
 * and can always be reliably understood by the receiver
 * as the start of a new frame.
 *
 * Inspired by https://jgtao.me/content/04-02-16/
 *
 * @param data A pointer to the bytes to send
 * @param count The number of bytes to send
 * @returns The number of bytes sent successfully
 */
static size_t transmit_encoded(const char *data, size_t count)
{
    for (size_t sent = 0; sent < count; ++sent)
        switch (data[sent])
        {
            case SFD:
            case ETX:
            case ESC:
                if (!try_send(ESC) || !try_send(data[sent] ^ ESC_XOR))
                    return sent;
                break;
            default:
                if (!try_send(data[sent]))
                    return sent;
        }

    return count;
}

/** Callback for mpack to flush the serialisation buffer. */
static void comms_writer_flush(mpack_writer_t *writer, const char *buffer, size_t count)
{
    size_t written = transmit_encoded(buffer, count);
    if (written != count)
        mpack_writer_flag_error(writer, mpack_error_io);
}

bool comms_send_buffer(uint8_t *type, uint8_t *data, size_t count)
{
    chDbgCheck(initialised);
    chMtxLock(&_comms_lock);
    bool started = transmit_start(strlen((char *)type) + count);

    mpack_writer_t writer;
    mpack_writer_init(&writer, (char *)&_comms_buffer, COMMS_BUFFER_SIZE);
    mpack_writer_set_flush(&writer, comms_writer_flush);

    if (started)
    {
        mpack_start_array(&writer, 2);
        mpack_write_cstr(&writer, (char *)type);
        mpack_write_bin(&writer, (char *)data, count);
        mpack_finish_array(&writer);
    }

    bool delivered = mpack_writer_destroy(&writer) == mpack_ok;

    transmit_end();
    chMtxUnlock(&_comms_lock);
    return delivered;
}

bool comms_send_msg(char *type, char *data)
{
    chDbgCheck(initialised);
    chMtxLock(&_comms_lock);
    bool started = transmit_start(strlen(type) + strlen(data));

    mpack_writer_t writer;
    mpack_writer_init(&writer, (char *)&_comms_buffer, COMMS_BUFFER_SIZE);
    mpack_writer_set_flush(&writer, comms_writer_flush);

    if (started)
    {
        mpack_start_array(&writer, 2);
        mpack_write_cstr(&writer, type);
        mpack_write_cstr(&writer, data);
        mpack_finish_array(&writer);
    }

    bool delivered = mpack_writer_destroy(&writer) == mpack_ok;

    transmit_end();
    chMtxUnlock(&_comms_lock);
    return delivered;
}

bool comms_send_msg_f(char *type, const char *fmt, ...)
{
    va_list args;

    // Some threads have a very limited stack size,
    // the string should be formatted on the heap
    char *buffer = (char *)malloc(FORMAT_BUFFER_SIZE);

    // Format the string
    va_start(args, fmt);
    vsnprintf(buffer, FORMAT_BUFFER_SIZE, fmt, args);
    va_end(args);

    bool result = comms_send_msg(type, buffer);

    free(buffer);
    return result;
}

void init_comms(void)
{
    chMtxObjectInit(&_comms_lock);
    initialised = true;
}
