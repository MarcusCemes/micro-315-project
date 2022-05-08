#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stdint.h"

void motor_set_position(float position_r, float position_l, float speed_r, float speed_l);

uint8_t motor_position_reached(void);

/** Rotate the robot (counter-clockwise) by `angle` radians (blocking). */
void motor_ctl_rotate(float angle);

void motor_rotation_bis(int32_t angle);

/** Translate the robot (forwards/backwards) by `distance` metres (blocking). */
void motor_ctl_translate(float distance);

void motor_translation_forward_bis(int32_t distance);

void motor_translation_backward_bis(int32_t distance);

#endif
