#include <ch.h>
#include <math.h>
#include <motors.h>

#include "movement.h"

#define TIMER_CLOCK 84000000
#define TIMER_FREQ 100000                // [Hz]
#define MOTOR_SPEED_LIMIT_CM 13          // [cm/s]
#define NSTEP_ONE_EL_TURN 4              // number of steps to do 1 electrical turn
#define NB_OF_PHASES 4                   // number of phases of the motors
#define SLOW_ROTATION_SPEED 200          // [step/s]
#define FAST_ROTATION_SPEED 800          // [step/s]
#define TRANSLATION_SPEED 800            // [step/s]
#define ROTATION_RADIUS 0.0265           // [m]
#define WHEEL_RADIUS 0.0205              // [m]
#define NSTEP_ONE_TURN 1000              // number of step for 1 turn of the motor
#define WHEEL_PERIMETER 13               // [cm]
#define TIME_ONE_TURN_AT_200 6.46        // [sec]
#define TIME_ONE_TURN_AT_800 1.615       // [sec]
#define TIME_TENTH_A_METER_AT_800 0.97   // [sec]
#define TENTH_A_METER 0.1                // [m]
#define OFFSET_AT_200 0.30               // [sec]
#define OFFSET_AT_800 0.265              // [sec]
#define OFFSET_TRANSLATION_AT_800 0.275  // [sec]
#define DISTANCE_BETWEEN_WHEELS 0.053    // [m]

#define MOTOR_RIGHT_A GPIOE, 13
#define MOTOR_RIGHT_B GPIOE, 12
#define MOTOR_RIGHT_C GPIOE, 14
#define MOTOR_RIGHT_D GPIOE, 15

#define MOTOR_LEFT_A GPIOE, 9
#define MOTOR_LEFT_B GPIOE, 8
#define MOTOR_LEFT_C GPIOE, 11
#define MOTOR_LEFT_D GPIOE, 10

#define MOTOR_RIGHT_TIMER TIM6
#define MOTOR_RIGHT_TIMER_EN RCC_APB1ENR_TIM6EN
#define MOTOR_RIGHT_IRQHandler TIM6_DAC_IRQHandler
#define MOTOR_RIGHT_IRQ TIM6_DAC_IRQn

#define MOTOR_LEFT_TIMER TIM7
#define MOTOR_LEFT_TIMER_EN RCC_APB1ENR_TIM7EN
#define MOTOR_LEFT_IRQ TIM7_IRQn
#define MOTOR_LEFT_IRQHandler TIM7_IRQHandler

#define SPEED_CONTROL 0
#define POSITION_CONTROL 1

#define STOP_SPEED 0
#define CHECK_INTERVAL_MS 100

static int16_t counter_step_right = 0;       // in [step]
static int16_t counter_step_left = 0;        // in [step]
static int16_t position_to_reach_right = 0;  // in [step]
static int16_t position_to_reach_left = 0;   // in [step]
static uint8_t position_right_reached = 0;
static uint8_t position_left_reached = 0;
static uint8_t state_motor = 0;

/* === Private functions === */

/** Applies the given speeds to the left and right motors, respectively. */
static void motor_ctl_individual_speed(int32_t speed_left, int32_t speed_right)
{
    left_motor_set_speed(speed_left);
    right_motor_set_speed(speed_right);
}

/** Applies the same speed to both the left and right motors. */
static void motor_ctl_common_speed(int32_t speed)
{
    motor_ctl_individual_speed(speed, speed);
}

/**
 * Applies the the speed to the left motor, and the
 * (additive) inverse to the right motor.
 */
static void motor_ctl_inverse_speed(int32_t speed)
{
    motor_ctl_individual_speed(speed, -speed);
}

/** Stops both motors. */
static void motor_ctl_stop(void)
{
    motor_ctl_common_speed(STOP_SPEED);
}

/* === Public functions === */

void motor_set_position(float position_r, float position_l, float speed_r, float speed_l)
{
    // Reinit global variable
    counter_step_left = 0;
    counter_step_right = 0;

    position_right_reached = 0;
    position_left_reached = 0;

    // Set global variable with position to reach in step
    position_to_reach_left = position_l * (NSTEP_ONE_TURN / WHEEL_PERIMETER);
    position_to_reach_right = -position_r * (NSTEP_ONE_TURN / WHEEL_PERIMETER);

    motor_ctl_individual_speed(speed_l, speed_r);

    // flag for position control, will erase flag for speed control only
    state_motor = POSITION_CONTROL;
}

bool motor_position_reached(void)
{
    return (state_motor == POSITION_CONTROL && position_right_reached && position_left_reached);
}

void motor_ctl_rotate_slow(float angle)
{
    float time = angle * (TIME_ONE_TURN_AT_200 / M_TWOPI);
    motor_ctl_inverse_speed(-SLOW_ROTATION_SPEED);
    chThdSleepMilliseconds(1000 * (time + OFFSET_AT_200));
    motor_ctl_stop();
}

void motor_ctl_rotate_fast(float angle)
{
    float time = angle * (TIME_ONE_TURN_AT_800 / M_TWOPI);
    motor_ctl_inverse_speed(-FAST_ROTATION_SPEED);
    chThdSleepMilliseconds(1000 * (time + OFFSET_AT_800));
    motor_ctl_stop();
}

void motor_ctl_translate_forward(float distance)
{
    float time = TIME_TENTH_A_METER_AT_800 * distance / TENTH_A_METER;
    motor_ctl_common_speed(TRANSLATION_SPEED);
    chThdSleepMilliseconds(1000 * (time + OFFSET_TRANSLATION_AT_800));
    motor_ctl_stop();
}

void motor_ctl_translate_backward(float distance)
{
    float time = TIME_TENTH_A_METER_AT_800 * distance / TENTH_A_METER;
    motor_ctl_common_speed(-TRANSLATION_SPEED);
    chThdSleepMilliseconds(1000 * (time + OFFSET_TRANSLATION_AT_800));
    motor_ctl_stop();
}
