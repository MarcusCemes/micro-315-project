#ifndef SENSORS_H
#define SENSORS_H

#include <msgbus/messagebus.h>
#include <parameter/parameter.h>

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

/** Initialise the sensors thread to detect objects. */
void init_sensors(void);

#endif
