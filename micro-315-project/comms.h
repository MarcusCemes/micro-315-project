#ifndef COMMS_H
#define COMMS_H

/**
 * Send a message to the remote device over Bluetooth.
 *
 * The type and data must be null-terminated strings, they
 * will be serialized using MessagePack as an array pair:
 * `[type, data]`;
 *
 * The maximum size of of the payload is 65536 bytes, including
 * the overhead needed for serializing data structure.
 *
 * ## Examples
 * bool all_sent = comms_send_msg("STATUS", "Ok");
 */
bool comms_send_msg(char *type, char *data);

void init_comms(void);

#endif
