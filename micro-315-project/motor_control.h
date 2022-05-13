#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stdint.h"

void motor_set_position(float position_r, float position_l, float speed_r, float speed_l);

uint8_t motor_position_reached(void);

/** Rotate the robot (counter-clockwise) by `angle` radians (blocking). */
void motor_ctl_rotate_slow(float angle);

void motor_ctl_rotate_fast(float angle);

/** Translate the robot (forwards/backwards) by `distance` metres (blocking). */
void motor_ctl_translate_forward(float distance);

void motor_ctl_translate_backward(float distance);

#endif
