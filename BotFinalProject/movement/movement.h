/*
 * movement.h
 *
 *  Created on: Sep 12, 2024
 *      Author: mdrobot7
 */

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include "open_interface.h"

#define MOVEMENT_DEFAULT_FORWARD_SPEED (125) // quarter speed, for accuracy
#define MOVEMENT_DEFAULT_TURN_SPEED (25)

// !!! CYBOT 2 !!!
#define MOVEMENT_MAGIC_TURN_OFFSET_LEFT (-4)
#define MOVEMENT_MAGIC_FORWARD_OFFSET_mm (165)
#define MOVEMENT_MAGIC_LEFT_MOTOR_FACTOR (1.0)


#define MVMT_OBJECT_BACKUP_DIST_cm (-5)

typedef enum
{
    MVMT_OBJ_NONE = 0,
    MVMT_OBJ_BUMP_BOTH,
    MVMT_OBJ_BUMP_LEFT,
    MVMT_OBJ_BUMP_RIGHT,
    MVMT_OBJ_HOLE_FRONT,
    MVMT_OBJ_HOLE_LEFT,
    MVMT_OBJ_HOLE_RIGHT,
    MVMT_OBJ_BORDER_FRONT,
    MVMT_OBJ_BORDER_LEFT,
    MVMT_OBJ_BORDER_RIGHT,
} mvmt_low_object_t;

#define MVMT_OBJ_IS_BUMP(x) (x >= MVMT_OBJ_BUMP_BOTH && x <= MVMT_OBJ_BUMP_RIGHT)
#define MVMT_OBJ_IS_HOLE(x) (x >= MVMT_OBJ_HOLE_FRONT && x <= MVMT_OBJ_HOLE_RIGHT)
#define MVMT_OBJ_IS_BORDER(x) (x >= MVMT_OBJ_BORDER_FRONT && x <= MVMT_OBJ_BORDER_RIGHT)
#define MVMT_OBJ_IS_LEFT(x) (x == MVMT_OBJ_BUMP_LEFT || x == MVMT_OBJ_HOLE_LEFT || x == MVMT_OBJ_BORDER_LEFT)
#define MVMT_OBJ_IS_RIGHT(x) (x == MVMT_OBJ_BUMP_RIGHT || x == MVMT_OBJ_HOLE_RIGHT || x == MVMT_OBJ_BORDER_RIGHT)

// Move forward some number of millimeters
void mvmt_move(oi_t *sensor_data, int distance_mm);

// Turn some number of degrees, positive = left, negative = right
void mvmt_turn(oi_t *sensor_data, int angle_deg);

// Tracks the total distance forward in millimeters
mvmt_low_object_t mvmt_driveUntilObject(oi_t *sensor_data, int distance_mm, float *total_forward_dist);

// Moves the bot around the object
void mvmt_avoidObject(oi_t *sensor_data, bool is_left);

// Drive some amount of distance while avoiding objects
void mvmt_moveWhileAvoiding(oi_t *sensor_data, float distance_mm);

// Back up from the cliff and turn away from it
void mvmt_cliffAvoidance(oi_t *sensor_data, bool is_left);

#endif /* MOVEMENT_H_ */
