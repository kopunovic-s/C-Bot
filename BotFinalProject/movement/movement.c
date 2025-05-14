/*
 * movement.c
 *
 *  Created on: Sep 12, 2024
 *      Author: mdrobot7
 */

#include "movement.h"
#include "../utils/Timer.h"

// !!! CYBOT 2 !!!
// Should probably be calibrated by bot
// Gray seems to be ~1300 to ~2500
#define CLIFF_HOLE_THRESH (500)
#define CLIFF_BORDER_THRESH (2700)

void mvmt_move(oi_t *sensor_data, int distance_mm)
{
    double sum = 0;

    if (distance_mm > 0)
    {
        oi_setWheels(MOVEMENT_DEFAULT_FORWARD_SPEED,
                     MOVEMENT_MAGIC_LEFT_MOTOR_FACTOR * MOVEMENT_DEFAULT_FORWARD_SPEED);
    }
    else
    {
        oi_setWheels(-MOVEMENT_DEFAULT_FORWARD_SPEED,
                     MOVEMENT_MAGIC_LEFT_MOTOR_FACTOR * -MOVEMENT_DEFAULT_FORWARD_SPEED);
    }

    while (abs(sum) < abs(distance_mm))
    {
        oi_update(sensor_data);
        sum += sensor_data->distance;
    }
    oi_setWheels(0, 0); // stop
    timer_waitMillis(250);
}

void mvmt_turn(oi_t *sensor_data, int angle_deg)
{
    double sum = 0;
    angle_deg += (angle_deg > 0) ? MOVEMENT_MAGIC_TURN_OFFSET_LEFT : -MOVEMENT_MAGIC_TURN_OFFSET_LEFT;
    if (angle_deg > 0)
    { // left
        oi_setWheels(MOVEMENT_DEFAULT_TURN_SPEED, MOVEMENT_MAGIC_LEFT_MOTOR_FACTOR * -MOVEMENT_DEFAULT_TURN_SPEED);
    }
    else
    { // right
        oi_setWheels(-MOVEMENT_DEFAULT_TURN_SPEED, MOVEMENT_MAGIC_LEFT_MOTOR_FACTOR * MOVEMENT_DEFAULT_TURN_SPEED);
    }

    while (abs(sum) < abs(angle_deg))
    {
        oi_update(sensor_data);
        sum += sensor_data->angle;
    }
    oi_setWheels(0, 0);
    timer_waitMillis(250);
}

mvmt_low_object_t mvmt_driveUntilObject(oi_t *sensor_data, int distance_mm, float *total_forward_dist)
{
    oi_setWheels(MOVEMENT_DEFAULT_FORWARD_SPEED, MOVEMENT_DEFAULT_FORWARD_SPEED);

    mvmt_low_object_t obj = MVMT_OBJ_NONE;
    while (obj == MVMT_OBJ_NONE && *total_forward_dist < distance_mm)
    {
        // Check every time oi updates
        oi_update(sensor_data);
        *total_forward_dist += sensor_data->distance; // sensor_data->distance is a double, I know it's a crime

        if (sensor_data->bumpLeft && sensor_data->bumpRight)
        {
            obj = MVMT_OBJ_BUMP_BOTH;
        }
        else if (sensor_data->bumpLeft)
        {
            obj = MVMT_OBJ_BUMP_LEFT;
        }
        else if (sensor_data->bumpRight)
        {
            obj = MVMT_OBJ_BUMP_RIGHT;
        }

        if ((sensor_data->cliffFrontLeftSignal < CLIFF_HOLE_THRESH) || (sensor_data->cliffFrontRightSignal < CLIFF_HOLE_THRESH))
        {
            obj = MVMT_OBJ_HOLE_FRONT;
        }
        if (sensor_data->cliffLeftSignal < CLIFF_HOLE_THRESH)
        {
            obj = MVMT_OBJ_HOLE_LEFT;
        }
        if (sensor_data->cliffRightSignal < CLIFF_HOLE_THRESH)
        {
            obj = MVMT_OBJ_HOLE_RIGHT;
        }

        if ((sensor_data->cliffFrontLeftSignal > CLIFF_BORDER_THRESH) || (sensor_data->cliffFrontRightSignal > CLIFF_BORDER_THRESH))
        {
            obj = MVMT_OBJ_BORDER_FRONT;
        }
        if (sensor_data->cliffLeftSignal > CLIFF_BORDER_THRESH)
        {
            obj = MVMT_OBJ_BORDER_LEFT;
        }
        if (sensor_data->cliffRightSignal > CLIFF_BORDER_THRESH)
        {
            obj = MVMT_OBJ_BORDER_RIGHT;
        }
    }

    oi_setWheels(0, 0); // stop

    return obj;
}

void mvmt_avoidObject(oi_t *sensor_data, bool is_left)
{

    // Backup 5 cm
    mvmt_move(sensor_data, MVMT_OBJECT_BACKUP_DIST_cm * 10);

    // The left bumper is hit so turn right
    if (is_left)
    {
        mvmt_turn(sensor_data, -45);
    }
    // The right button is hit so turn left
    else
    {
        mvmt_turn(sensor_data, 45);
    }

    // Go forward 5 cm
    mvmt_move(sensor_data, 50);

//    // Turn left
//    if (is_left)
//    {
//        mvmt_turn(sensor_data, 90);
//    }
//    // Turn right
//    else
//    {
//        mvmt_turn(sensor_data, -90);
//    }
//
//    // Move forward 5 cm to compensate for the backup
//    mvmt_move(sensor_data, -MVMT_OBJECT_BACKUP_DIST_cm);
}

void mvmt_moveWhileAvoiding(oi_t *sensor_data, float distance_mm)
{
    float current = 0;
    float dist_left = distance_mm;
    while (current < distance_mm)
    {
        dist_left = distance_mm - current;
        current += mvmt_driveUntilObject(sensor_data, dist_left, &current);

        if (sensor_data->bumpLeft)
        {
            mvmt_avoidObject(sensor_data, true);
        }
        if (sensor_data->bumpRight)
        {
            mvmt_avoidObject(sensor_data, false);
        }
        // 1200 is the value for the black felt or tape
        if ((sensor_data->cliffFrontLeftSignal < 1200) || (sensor_data->cliffFrontRightSignal < 1200))
        {
            mvmt_cliffAvoidance(sensor_data, 0);
        }
        if (sensor_data->cliffLeftSignal < 1200)
        {
            mvmt_cliffAvoidance(sensor_data, 1);
        }
        if (sensor_data->cliffRightSignal < 1200)
        {
            mvmt_cliffAvoidance(sensor_data, 2);
        }
        // 2500 is the value for the white tape
        if ((sensor_data->cliffFrontLeftSignal > 2500) || (sensor_data->cliffFrontRightSignal > 2500))
        {
            mvmt_cliffAvoidance(sensor_data, 3);
        }
        if (sensor_data->cliffLeftSignal > 2500)
        {
            mvmt_cliffAvoidance(sensor_data, 4);
        }
        if (sensor_data->cliffRightSignal > 2500)
        {
            mvmt_cliffAvoidance(sensor_data, 5);
        }
    }
}

void mvmt_cliffAvoidance(oi_t *sensor_data, bool is_left)
{
    // Stop the bot to avoid going over the cliff
    mvmt_move(sensor_data, MVMT_OBJECT_BACKUP_DIST_cm * 10);
    mvmt_turn(sensor_data, (is_left ? -45 : 45));
}
