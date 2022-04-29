#include <ch.h>
#include <leds.h>

#include "lights.h"

#define ON 1
#define OFF 0
#define MAX_INTENSITY 255

static THD_WORKING_AREA(lights_stack, 128);
static THD_FUNCTION(lights_thread, arg)
{
	(void)arg;

	while (1)
	{
		set_led(NUM_LED, ON);
		set_rgb_led(LED2, MAX_INTENSITY, OFF, OFF);
		set_rgb_led(LED4, MAX_INTENSITY, OFF, OFF);
		set_rgb_led(LED6, MAX_INTENSITY, OFF, OFF);
		set_rgb_led(LED8, MAX_INTENSITY, OFF, OFF);

		chThdSleepMilliseconds(1000);

		clear_leds();

		chThdSleepMilliseconds(1000);
	}
}

void start_lights()
{
	(void)chThdCreateStatic(lights_stack, sizeof(lights_stack), NORMALPRIO, lights_thread, NULL);
}
