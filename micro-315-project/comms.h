#ifndef COMMS_H
#define COMMS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * Send a MessagePack-serialised binary payload to the remote
 * device over Bluetooth.
 *
 * The message will be serialized as a [type, data] array.
 * Type must be a null terminated string.
 *
 * @returns true if the transmission was successful
 */
bool comms_send_buffer(char *type, uint8_t *data, size_t count);

/**
 * Send a MessagePack-serialised message to the remote
 * device over Bluetooth.
 *
 * The message will be serialized as a [type, data] array.
 * Both type and data must be null-terminated strings.
 *
 * @returns true if the transmission was successful
 *
 * ## Examples
 * bool all_sent = comms_send_msg("STATUS", "Ok");
 */
bool comms_send_msg(char *type, char *data);

/**
 * Similar to comms_send_msg(), but formats the string in
 * a static buffer first. MUST NOT EXCEED 1kB!
 *
 * @returns true if the transmission was successful
 */
bool comms_send_msg_f(char *type, const char *fmt, ...);

/** Initialises the communications mutex. */
void init_comms(void);

#endif
