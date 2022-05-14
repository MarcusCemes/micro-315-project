#include "sensors.h"

#include <ch.h>
#include <hal.h>
#include <leds.h>
#include <sensors/proximity.h>

#include "comms.h"
#include "lights.h"

const uint16_t THRESHOLD = 500;

/** Checks whether any of the sensors are above the threshold. */
static bool in_proximity(proximity_msg_t* values)
{
    for (u_int i = 0; i < PROXIMITY_NB_CHANNELS; ++i)
        if (abs((int32_t)(values->delta[i] - values->initValue[i])) > THRESHOLD)
            return true;

    return false;
}

// This thread seems to require a larger stack, likely due
// to the messagebus.
static THD_WORKING_AREA(sensors_stack, 256);
static THD_FUNCTION(sensors_thread, arg)  // @suppress("No return")
{
    (void)arg;
    chRegSetThreadName(__FUNCTION__);

    messagebus_topic_t* prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    proximity_msg_t prox_values;

    while (true)
    {
        messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

        if (!in_proximity(&prox_values))
            continue;

        lights_trigger(LIGHTS_ATTENTION, LIGHTS_ONCE);

        // Wait until the object leaves the periphery
        do
            messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
        while (in_proximity(&prox_values));

        // Extra delay to avoid immediate replays
        chThdSleepMilliseconds(1000);
    }
}

void init_sensors(void)
{
    (void)chThdCreateStatic(sensors_stack, sizeof(sensors_stack), HIGHPRIO, sensors_thread, NULL);

    proximity_start();
    calibrate_ir();
}
