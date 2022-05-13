#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>

void motor_set_position(float position_r, float position_l, float speed_r, float speed_l);

/** Return whether the translation has finished. */
bool motor_position_reached(void);

/** Rotate the robot (counter-clockwise) by `angle` radians (blocking). */
void motor_ctl_rotate_slow(float angle);

/** Rotate the robot (counter-clockwise) by `angle` radians (blocking). */
void motor_ctl_rotate_fast(float angle);

/** Translate the robot forwards by `distance` metres (blocking). */
void motor_ctl_translate_forward(float distance);

/** Translate the robot forwards by `distance` metres (blocking). */
void motor_ctl_translate_backward(float distance);

#endif
