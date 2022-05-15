#include "main.h"

#include <ch.h>
#include <hal.h>
#include <i2c_bus.h>
#include <leds.h>
#include <memory_protection.h>
#include <motors.h>
#include <selector.h>
#include <sensors/proximity.h>
#include <spi_comm.h>
#include <usbcfg.h>

#include "audio.h"
#include "comms.h"
#include "lights.h"
#include "motor_control.h"
#include "programs.h"
#include "sensors.h"
#include "speaker.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void init(void);
static void run(void);

/* == Entry point == */

int main(void)
{
    init();
    run();
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
    halInit();    // HAL
    chSysInit();  // ChibiOS/RT
    mpu_init();   // Memory protection

    // Initialize the inter-process communication bus
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    // Reset to a known state
    clear_leds();       // LEDS
    set_body_led(0);    // LEDS
    set_front_led(0);   // LEDS
    usb_start();        // USB
    i2c_start();        // IC2
    spi_comm_start();   // Serial Peripheral Interface
    serial_start();     // UART3
    motors_init();      // Motors
    proximity_start();  // Proximity sensors

    audio_init();    // Audio
    comms_init();    // Communication
    lights_init();   // Lights
    mctl_init();     // Motor control
    sensors_init();  // Sensors

    audio_start();    // DSP thread
    lights_start();   // Lights thread
    mctl_start();     // Motor timers
    speaker_start();  // Speaker threads
}

// Some functions such as playSoundFile() seem to cause kernel
// panics on main, but work on a separate thread with a larger stack.
static THD_WORKING_AREA(run_stack, 2048);
static THD_FUNCTION(run_thread, arg)
{
    (void)arg;
    chRegSetThreadName(__FUNCTION__);

    comms_send_msg("STATUS", "init");
    speaker_play_wav(SPKR_WAV_STARTUP, SF_FORCE_CHANGE, true);
    chThdSleepMilliseconds(500);

    switch (get_selector())
    {
        case 0:
            program_point_sound();
            break;

        case 1:
            program_locate_sound();
            break;

        default:
            lights_queue(LIGHTS_STOP, LIGHTS_LOOP);
            lights_trigger(LIGHTS_ATTENTION, LIGHTS_ONCE);
            speaker_play_wav(SPKR_WAVE_BEEP_01, SF_FORCE_CHANGE, true);
            break;
    }

    comms_send_msg("STATUS", "complete");
}

// Has scope visibility of run_stack and run_thread
static void run(void)
{
    chThdCreateStatic(run_stack, sizeof(run_stack), NORMALPRIO, run_thread, NULL);
}

/* == Stack guard == */

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
