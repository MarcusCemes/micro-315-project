#include "main.h"

#include <audio/audio_thread.h>
#include <audio/microphone.h>
#include <audio/play_melody.h>
#include <ch.h>
#include <hal.h>
#include <i2c_bus.h>
#include <leds.h>
#include <math.h>
#include <memory_protection.h>
#include <motors.h>
#include <sensors/proximity.h>
#include <spi_comm.h>
#include <stdlib.h>
#include <usbcfg.h>

#include "audio.h"
#include "comms.h"
#include "lights.h"
#include "sensors.h"
#include "speaker.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);  // @suppress("Field cannot be resolved")
CONDVAR_DECL(bus_condvar);

static void init(void);

void send_audio(void)
{
    audio_subscribe();

    while (true)
        audio_wait();

    audio_unsubscribe();
}

/* == Entry point == */

int main(void)
{
    init();
    trigger_lights(LIGHTS_WAITING, LIGHTS_LOOP);
    comms_send_msg("EVENT", "READY");
    send_audio();
    chThdSleep(TIME_INFINITE);
}

/* == Initialisation == */

static void serial_start(void)
{
    static SerialConfig ser_cfg = { 115200, 0, 0, 0 };
    sdStart(&SD3, &ser_cfg);
}

static void init(void)
{
    halInit();    // HAL initialization
    chSysInit();  // ChibiOS/RT initialization
    mpu_init();   // Memory Protection initialization

    // Initialize the inter-process communication bus
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    // Reset to a known state
    clear_leds();       // LEDS
    set_body_led(0);    // LEDS
    set_front_led(0);   // LEDS
    usb_start();        // USB
    i2c_start();        // IC2
    motors_init();      // Motors
    proximity_start();  // Proximity sensors
    dac_start();        // Speaker
    spi_comm_start();   // Serial Peripheral Interface
    serial_start();     // UART3

    audio_init();    // Audio
    init_comms();    // Communication
    lights_init();   // Lights
    init_sensors();  // Sensors

    audio_start();    // DSP thread
    lights_start();   // Lights thread
    speaker_start();  // Speaker thread
}

/* == Stack guard == */

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
