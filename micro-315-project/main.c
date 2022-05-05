#include "main.h"

#include <arm_math.h>
#include <audio/audio_thread.h>
#include <audio/microphone.h>
#include <ch.h>
#include <chprintf.h>
#include <hal.h>
#include <leds.h>
#include <memory_protection.h>
#include <motors.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <sensors/mpu9250.h>
#include <sensors/proximity.h>
#include <spi_comm.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <usbcfg.h>

#include "audio.h"
#include "comms.h"
#include "lights.h"
#include "sensors.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);  // @suppress("Field cannot be resolved")
CONDVAR_DECL(bus_condvar);

static void init(void);

/** Send audio back every 500ms to the remote device for visualisation. */
void send_audio(void)
{
    audio_register();

    while (true)
    {
        audio_wait();

        audio_data_t* data = audio_data_borrow();
        comms_send_buffer("PCM", (uint8_t*)data->pcm[FRONT], AUDIO_BUFFER_SIZE);
        comms_send_buffer("FFT", (uint8_t*)data->fft[FRONT], AUDIO_BUFFER_SIZE);
        comms_send_buffer("MAG", (uint8_t*)data->magnitudes[FRONT], AUDIO_BUFFER_SIZE);

        audio_data_return();

        chThdSleepMilliseconds(500);
    }

    audio_unregister();
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
    static SerialConfig ser_cfg = {
        115200,
        0,
        0,
        0,
    };

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
    dcmi_start();       // Camera
    po8030_start();     // IC2, Camera
    motors_init();      // Motors
    proximity_start();  // Proximity sensors
    dac_start();        // Speaker
    spi_comm_start();   // Serial Peripheral Interface
    serial_start();     // UART3

    audio_init();    // Audio
    init_comms();    // Communication
    lights_init();   // Lights
    init_sensors();  // Sensors

    lights_start();  // Lights thread
}

/* == Stack guard == */

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
