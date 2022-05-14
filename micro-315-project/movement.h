#ifndef MOVEMENT_H
#define MOVEMENT_H

typedef enum
{
    MOVE_IMMEDIATE,
    MOVE_WAIT,
} move_mode_t;

typedef enum
{
    MOVE_MAX = 1100,
    MOVE_FAST = 800,
    MOVE_NORMAL = 650,
    MOVE_MEDIUM = 400,
    MOVE_SLOW = 200,
    MOVE_VERY_SLOW = 100,
} move_speed_t;

typedef enum
{
    ANTICLOCKWISE,
    CLOCKWISE,
} move_orientation_t;

/** Move forwards (or backwards) by `distance` metres. */
void move_by(float distance, move_mode_t mode, move_speed_t speed);

/** Move forwards with acceleration/deceleration curves. */
void move_by_smooth(float distance, move_speed_t speed);

/** Rotate counterclockwise (or clockwise) by `angle` radians. */
void rotate_by(float angle, move_mode_t mode, move_speed_t speed);

/** Rotate by `angle` radians with acceleration/deceleration curves. */
void rotate_by_smooth(float angle, move_speed_t speed);

/** Rotate counterlockwise (or clockwise) by `turns` full turns. */
void rotate_by_turns(float turns, move_mode_t mode, move_speed_t speed);

/** Rotate by `turns` full turns with acceleration/deceleration curves. */
void rotate_by_turns_smooth(float turns, move_speed_t speed);

/** Move forwards (or backwards) by `distance` metres. */
void move_arc(float radius, float angle, move_orientation_t orientation, move_mode_t mode,
              move_speed_t speed);

#endif
