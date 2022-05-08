#include <ch.h>
#include <math.h>
#include <motors.h>

#define TIMER_CLOCK 84000000
#define TIMER_FREQ 100000          // [Hz]
#define MOTOR_SPEED_LIMIT 13       // [cm/s]
#define NSTEP_ONE_EL_TURN 4        // number of steps to do 1 electrical turn
#define NB_OF_PHASES 4             // number of phases of the motors
#define ROTATION_SPEED 200         // [step/s]
#define TRANSLATION_SPEED 200      // [step/s]
#define ROTATION_RADIUS 0.0265     // [m]
#define WHEEL_RADIUS 0.02          // [m]
#define NSTEP_ONE_TURN 1000        // number of step for 1 turn of the motor
#define WHEEL_PERIMETER 13         // [cm]
#define TIME_ONE_TURN_AT_200 7.15  // [sec]

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

// some static global variables
static int16_t right_speed = 0;              // in [step/s]
static int16_t left_speed = 0;               // in [step/s]
static int16_t counter_step_right = 0;       // in [step]
static int16_t counter_step_left = 0;        // in [step]
static int16_t position_to_reach_right = 0;  // in [step]
static int16_t position_to_reach_left = 0;   // in [step]
static uint8_t position_right_reached = 0;
static uint8_t position_left_reached = 0;
static uint8_t state_motor = 0;

/* == Private functions == */

/** Applies the given speeds to the left and right motors, respectively. */
static void motor_ctl_individual_speed(int32_t speed_left, int32_t speed_right)
{
    left_motor_set_speed(speed_left);
    right_motor_set_speed(speed_right);
}

/** Applies the same speed to both the left and right motors. */
static void motor_ctl_common_speed(int32_t speed)
{
    ctrl_different_speed(speed, speed);
}

/**
 * Applies the the speed to the left motor, and the
 * (additive) inverse to the right motor.
 */
static void motor_ctl_inverse_speed(int32_t speed)
{
    ctrl_different_speed(speed, -speed);
}

/** Stops both motors. */
static void motor_ctl_stop(void)
{
    motor_ctl_common_speed(STOP_SPEED);
}

/* == Public functions == */

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
    state_motor = POSITION_CONTROL;  //! ca sert a quelque chose ?
}

bool motor_position_reached(void)
{
    return (state_motor == POSITION_CONTROL && position_right_reached && position_left_reached);
}

void motor_ctl_rotate(float angle)
{
    // time = NSTEP_ONE_TURN*angle*ROTATION_RADIUS/(4*M_PI*ROTATION_SPEED*WHEEL_RADIUS);
    uint8_t time = angle * (TIME_ONE_TURN_AT_200 / M_TWOPI);
    motor_ctl_inverse_speed(-ROTATION_SPEED);
    chThdSleep(S2ST(time));
    motor_ctl_stop();
}

void motor_rotation_bis(int32_t angle)
{  // we use the trigonometric orientation for the angle and 0<angle<2PI
    // motor_set_position(angle*ROTATION_RADIUS, angle*ROTATION_RADIUS, ROTATION_SPEED,
    // -ROTATION_SPEED);
    motor_set_position(20, 20, 5, -5);  //! magic numbers

    while (!motor_position_reached())
        chThdSleepMilliseconds(CHECK_INTERVAL_MS);
}

void motor_ctl_translate(float distance)
{
    uint32_t time = distance / (TRANSLATION_SPEED * WHEEL_RADIUS);
    ctrl_common_speed(TRANSLATION_SPEED);
    chThdSleep(S2ST(time));
    motor_ctl_stop();
}

void motor_translation_forward_bis(int32_t distance)
{
    motor_set_position(distance, distance, TRANSLATION_SPEED, TRANSLATION_SPEED);
}

void motor_translation_backward_bis(int32_t distance)
{
    motor_set_position(distance, distance, -TRANSLATION_SPEED, -TRANSLATION_SPEED);
}
