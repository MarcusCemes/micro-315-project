#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ch.h>
#include <chprintf.h>
#include <hal.h>

#include "audio_processing.h"

void micDataReadyCallback(int16_t *_data, uint16_t _num_samples)
{
    // Process received data
}
