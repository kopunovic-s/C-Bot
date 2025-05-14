#include "navigation.h"

#include "../movement/movement.h"
#include "../scan/scan.h"
#include "../uart/uart.h"
#include "../sound/sound.h"
#include "map.h"

static oi_t *cybot;
static object_t *objects;
static char uartbuf[100];
static bool emergency_stop = false;

/*
 *
 * STATIC FUNCTIONS
 *
 */

static void scan_and_map()
{
    static int num_objects;
    scan_scan180(true);
    num_objects = scan_detectObjects(true);
    map_addTallObjects(objects, num_objects);
    sprintf(uartbuf, "Heading: %.2f", map_getBotHeading());
    uart_sendLine(uartbuf);
    map_sendMapToUart();
}

static void explore(float target_dist_mm)
{
    float total_distance_mm = 0.0; // Total distance traveled so far
    float last_total_distance_mm = 0.0;
    while (total_distance_mm < target_dist_mm)
    {
        mvmt_low_object_t low_object = mvmt_driveUntilObject(cybot, target_dist_mm - total_distance_mm, &total_distance_mm);
        if (low_object)
        {
            map_updateBotPosition((total_distance_mm - last_total_distance_mm) / 10.0 + MVMT_OBJECT_BACKUP_DIST_cm, 0);
        }
        else
        {
            map_updateBotPosition((total_distance_mm - last_total_distance_mm) / 10.0, 0);
        }

        if (MVMT_OBJ_IS_BUMP(low_object))
        {
            switch (low_object)
            {
            case MVMT_OBJ_BUMP_BOTH:
                uart_sendLine("Bump Front");
                map_addShortObject(true, true);
                map_updateBotPosition(5, -45);                         // Avoid right
                break;
            case MVMT_OBJ_BUMP_LEFT:
                uart_sendLine("Bump Left");
                map_addShortObject(true, false);
                map_updateBotPosition(5, -45);                         // Avoid right
                break;
            case MVMT_OBJ_BUMP_RIGHT:
                uart_sendLine("Bump Right");
                map_addShortObject(false, true);
                map_updateBotPosition(5, 45);                           // Avoid left
                break;
            }

            // Avoid the object. We recorded where we *should* be at the end
            mvmt_avoidObject(cybot, (low_object != MVMT_OBJ_BUMP_RIGHT));
        }
        else if (MVMT_OBJ_IS_HOLE(low_object))
        {
            switch (low_object)
            {
            case MVMT_OBJ_HOLE_FRONT:
                uart_sendLine("Hole Front");
                break;
            case MVMT_OBJ_HOLE_LEFT:
                uart_sendLine("Hole Left");
                break;
            case MVMT_OBJ_HOLE_RIGHT:
                uart_sendLine("Hole Right");
                break;
            }
            map_updateBotPosition(0, (MVMT_OBJ_IS_LEFT(low_object) ? -45 : 45));
            // Avoid the object. We recorded where we *should* be at the end
            mvmt_cliffAvoidance(cybot, MVMT_OBJ_IS_LEFT(low_object));
        }
        else if (MVMT_OBJ_IS_BORDER(low_object))
        {
            // Avoid it, while recording where we're going
            switch (low_object)
            {
            case MVMT_OBJ_BORDER_FRONT:
                uart_sendLine("Border Front");
                break;
            case MVMT_OBJ_BORDER_LEFT:
                uart_sendLine("Border Left");
                break;
            case MVMT_OBJ_BORDER_RIGHT:
                uart_sendLine("Border Right");
                break;
            }
            map_updateBotPosition(0, (MVMT_OBJ_IS_LEFT(low_object) ? -45 : 45));
            // Avoid the object. We recorded where we *should* be at the end
            mvmt_cliffAvoidance(cybot, MVMT_OBJ_IS_LEFT(low_object));
        }
        else
        {
            // Didn't run into anything, so we must have reached the target dist
            break;
        }

        // Recheck that we're not going to run into anything
        if (map_getDistanceToNextObj() < (target_dist_mm - total_distance_mm))
        {
            target_dist_mm = total_distance_mm + map_getDistanceToNextObj() * 10.0;
        }
        last_total_distance_mm = total_distance_mm;
        if(emergency_stop) return;
    }
}

/*
 *
 * GLOBAL FUNCTIONS
 *
 */

void nav_init(oi_t *cybot_ptr)
{
    cybot = cybot_ptr;
    map_init();
    scan_init(cybot_ptr);
    sound_init();
}

void nav_run()
{
    emergency_stop = false;
    objects = scan_getObjectBuf();

    // Wake up and figure out where we are, scan all sides
    int i;
    for (i = 0; i < 4; i++)
    {
        scan_and_map();
        mvmt_turn(cybot, 90);
        map_updateBotPosition(0, 90);
        if(emergency_stop) return;
    }

    float turn_angle = 0.0;
    while (!map_foundLargestObj())
    {
        mvmt_turn(cybot, turn_angle);
        map_updateBotPosition(0, turn_angle);
        if (turn_angle == 0.0)
        {
            turn_angle = -20;
        }
        turn_angle = -turn_angle; // Uhhhh just keep wandering around the field and
                                  // avoiding stuff until we get lucky...?

        float target_dist_mm = map_getDistanceToNextObj() * 10.0;
        sprintf(uartbuf, "Distance to next (mm): %.2f", target_dist_mm);
        uart_sendLine(uartbuf);
        if (target_dist_mm > 1000.0)
        {
            target_dist_mm = 1000.0; // 1 meter increments, max
        }

        if(emergency_stop) return;

        explore(target_dist_mm);

        if(emergency_stop) return;

        scan_and_map();
    }

    uart_sendLine("Found largest object");
    sprintf(uartbuf, "Distance to largest: %.2f", map_getLargestObjDistance());
    uart_sendLine(uartbuf);

    // Go to the largest object, while avoiding other objects
    // and recording to the map
    while (map_getLargestObjDistance() > 10.0)
    {

        sprintf(uartbuf, "ang: %.2f, dist: %.2f, head: %.2f", map_getLargestObjAngle(), map_getLargestObjDistance(), map_getBotHeading());
        uart_sendLine(uartbuf);
        float turn_angle = map_getLargestObjAngle();
        mvmt_turn(cybot, turn_angle); // Turn towards the largest object
        map_updateBotPosition(0.0, turn_angle);

        if(emergency_stop) return;

        explore(map_getLargestObjDistance() * 10.0 / 2.0); // Move towards the largest object in half-distance steps

        if(emergency_stop) return;

        scan_and_map();
    }

    uart_sendLine("At largest object");
    sound_bomb();

//    // Go back to the start, while avoiding other objects
//    // and recording to map
//    while (map_getStartDistance() > 15.0)
//    {
//        mvmt_turn(cybot, map_getStartAngle()); // Turn towards the largest object
//
//        explore(map_getStartDistance() * 10.0 / 2.0); // Move towards the largest object in half-distance steps
//
//        scan_and_map();
//    }
//
//    uart_sendLine("Back home");
//    sound_home();
}

void nav_emergencyStop()
{
    emergency_stop = true;
}
