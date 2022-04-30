#include <ch.h>
#include <hal.h>

#include "comms.h"
#include "vendor/mpack.h"

#define COMMS_BUFFER_SIZE 256

/** Ensures that one one thread can send an uninterrupted message. */
static mutex_t _comms_lock;
/** A statically allocated buffer of reasonable size for serialization. */
static uint8_t _comms_buffer[COMMS_BUFFER_SIZE];

/** Start Frame Delimiter character. */
const uint8_t SFD = 0x7E;
/** The escape character. */
const uint8_t ESC = 0x7D;
/** A magic constant for concealing uncaught escaped characters. */
const uint8_t ESC_XOR = 0x20;

/** The maximum payload size (limited by the header). */
const size_t MAX_SIZE = INT16_MAX;

/** Provide byte-level access to a uint16_t. */
union payload_size_t
{
	uint16_t integer;
	uint8_t bytes[2];
};

/** Attempts to transmit a single byte over the serial port SD3. */
static bool try_send(uint8_t byte)
{
	msg_t msg = chSequentialStreamPut(&SD3, byte);
	return msg == STM_OK;
}

/**
 * Encodes the given data for a custom communication protocol,
 * so that it may be identified and isolated by the receiver.
 *
 * A frame consists of a header with a Start Frame Delimiter
 * (0x7E, 1 byte), followed by a header containing the size of
 * the payload (2 bytes), and finally the payload itself.
 *
 * Any instances of the SFD in the payload will be prepended with
 * the escape character (0x7D) and also transformed with an XOR
 * with the constant 0x20. This ensures that the SDF never
 * appears in the payload, and can always be reliably understood
 * by the receiver as the start of a new frame.
 *
 * The payload may not exceed 65536 bytes!
 *
 * Inspired by https://jgtao.me/content/04-02-16/
 *
 * @param data A pointer to the bytes to send
 * @param count The number of bytes to send
 * @returns The number of bytes sent successfully
 */
static uint16_t send_encoded(const char *data, uint16_t count)
{
	uint16_t sent = 0;

	// Send the Start Frame Delimiter character (1 byte)
	if (!try_send(SFD))
		return sent;

	// Transmit the expected payload size (two bytes)
	union payload_size_t payload_size = {.integer = count};
	if (!try_send(payload_size.bytes[1]) || !try_send(payload_size.bytes[0]))
		return sent;

	// Transmit the payload, escaping and transforming any
	// encountered SFD characters
	for (; sent < count; ++sent)
	{
		if (data[sent] == SFD || data[sent] == ESC)
		{
			if (!try_send(ESC) || !try_send(data[sent] ^ ESC_XOR))
				return sent;
		}
		else
		{
			if (!try_send(data[sent]))
				return sent;
		}
	}

	return sent;
}

/** Callback for mpack to flush the serialisation buffer. */
static void comms_writer_flush(mpack_writer_t *writer, const char *buffer, size_t count)
{
	size_t written = send_encoded(buffer, count);
	if (written != count)
		mpack_writer_flag_error(writer, mpack_error_io);
}

bool comms_send_msg(char *type, char *data)
{
	chMtxLock(&_comms_lock);

	mpack_writer_t writer;

	mpack_writer_init(&writer, (char *)&_comms_buffer, COMMS_BUFFER_SIZE);
	mpack_writer_set_flush(&writer, comms_writer_flush);

	mpack_start_array(&writer, 2);
	mpack_write_cstr(&writer, type);
	mpack_write_cstr(&writer, data);
	mpack_finish_array(&writer);

	chMtxUnlock(&_comms_lock);
	return mpack_writer_destroy(&writer) == mpack_ok;
}

/** Initialises the mutex needed for communication. */
void init_comms(void)
{
	chMtxObjectInit(&_comms_lock);
}
