#ifndef COMMS_H
#define COMMS_H

#include <stdbool.h>

/**
 * Send a MessagePack-serialised message to the remote
 * device over Bluetooth.
 *
 * The message will be serialized as a [type, data] array.
 * Both type and data must be null-terminated strings.
 *
 * ## Examples
 * bool all_sent = comms_send_msg("STATUS", "Ok");
 */
bool comms_send_msg(char *type, char *data);

/** Initialises the communications mutex. */
void init_comms(void);

#endif
