#include <motors.h>


void motor_set_position(float position_r, float position_l, float speed_r, float speed_l);

uint8_t motor_position_reached(void);

void motor_rotation(int angle);

void motor_rotation_bis(int angle);

void motor_translation_forward (int distance);

void motor_translation_backward (int distance);

void motor_translation_forward_bis (int distance);

void motor_translation_backward_bis (int distance);
