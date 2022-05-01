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

#include "comms.h"
#include "lights.h"

messagebus_t bus;
MUTEX_DECL(bus_lock); // @suppress("Field cannot be resolved")
CONDVAR_DECL(bus_condvar);

static void init(void);

int main(void)
{
    init();
    comms_send_msg("EVENT", "INIT");
    trigger_lights(LIGHTS_WAITING, LIGHTS_LOOP);
    chThdSleep(TIME_INFINITE);
}

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
    halInit();   // HAL initialization
    chSysInit(); // ChibiOS/RT initialization
    mpu_init();  // Memory Protection initialization

    // Initialize the inter-process communication bus
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    // Reset to a known state
    clear_leds();      // LEDS
    set_body_led(0);   // LEDS
    set_front_led(0);  // LEDS
    usb_start();       // USB
    dcmi_start();      // Camera
    po8030_start();    // IC2, Camera
    motors_init();     // Motors
    proximity_start(); // Proximity sensors
    dac_start();       // Speaker
    spi_comm_start();  // Serial Peripheral Interface
    serial_start();    // UART3
    mic_start(NULL);   // Microphone

    init_lights(); // Light animations
    init_comms();  // Communication
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
