#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <arm_math.h>
#include <audio/microphone.h>
#include <ch.h>
#include <chprintf.h>
#include <hal.h>
#include <motors.h>
#include <memory_protection.h>
#include <usbcfg.h>

#include "main.h"

static void init(void);

int main(void)
{
    init();

    while (1)
    {
        chThdSleepMilliseconds(1000);
    }
}

static void init(void)
{
    halInit();   // HAL initialization
    chSysInit(); // ChibiOS/RT initialization
    mpu_init();  // Memory Protection initialization
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
