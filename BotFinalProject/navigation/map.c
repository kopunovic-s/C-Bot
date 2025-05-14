#include "map.h"

#include <math.h>
#include "../uart/uart.h"

/**
 * Notes:
 * - There aren't a lot of "constants" here to go off of.
 * You don't know where you start or your heading. All you know is where
 * you started and that it's a valid spot.
 *
 * You can complete the whole challenge only based on your starting point
 * and the things you find along the way. The graph/map WILL NOT look
 * square or correct, but from the bot's perspective everything will be
 * right.
 *
 *
 * ALL DISTANCES ARE RELATIVE TO CENTER OF THE BOT
 * 0 DEGREES IS TO THE RIGHT (like the unit circle)
 * POSITIVE Y IS FORWARD
 * POSITIVE X IS RIGHT
 */

#define IN_TOLERANCE(val, target, tol) ((val >= target - tol) && (val <= target + tol))
#define DISTANCE(x1, y1, x2, y2) (sqrtf(((x1) - (x2)) * ((x1) - (x2)) + ((y1) - (y2)) * ((y1) - (y2))))

#define CYBOT_DIAMETER_cm (34.9)   // iRobot datasheet pg 5
#define CYBOT_SERVO_OFFSET_cm (10) // Distance from center of bot to servo axle
#define CYBOT_SENSOR_OFFSET_cm (4) // Distance from servo axle to sensor lenses
#define LARGEST_OBJ_SIZE_cm (15)   // Actual width: 10cm

#define OBJECT_POS_UNKNOWN (0xFFFFFBAD)

typedef enum
{
    MAP_OBJECT_UNKNOWN = 0,
    MAP_OBJECT_START, // Coordinates of where we started
    MAP_OBJECT_BOT,
    MAP_OBJECT_TALL,
    MAP_OBJECT_SHORT,
    MAP_OBJECT_HOLE,
    MAP_OBJECT_BOUNDARY,
} map_object_type_t;

typedef struct
{
    map_object_type_t type;
    float position_x; // in cm, relative to start
    float position_y;
    float width_cm;
} map_object_t;

// Start point is always (0, 0). All other points are relative to this
#define SET_MAP_OBJECT_START(object)    \
    {                                   \
        object.type = MAP_OBJECT_START; \
        object.position_x = 0.0;        \
        object.position_y = 0.0;        \
        object.width_cm = 10.0;         \
    }

#define SET_MAP_OBJECT_BOT(object)           \
    {                                        \
        object.type = MAP_OBJECT_BOT;        \
        object.position_x = 0.0;             \
        object.position_y = 0.0;             \
        object.width_cm = CYBOT_DIAMETER_cm; \
    }

#define MAP_SIZE (100)
#define MAP_START_IDX (0)
#define MAP_BOT_IDX (1)

// Map is just a list of objects
static map_object_t map[MAP_SIZE];
static map_object_t *start = &map[MAP_START_IDX];
static map_object_t *bot = &map[MAP_BOT_IDX];
static float bot_heading_deg = 90.0; // Forwards is 90 degrees, this is hard to think about but it's fine
static map_object_t *largest = NULL;
static int count = 2; // Bot and start point are 0 and 1

/*
 *
 * STATIC FUNCTIONS
 *
 */

/**
 * @brief Convert ping sensor distance to distance from
 * center of cybot. This is PAINFUL, there are like 2
 * stacked circles going on here
 *
 * @param obj
 * @return float Distance in cm
 */
static float object_dist_from_center(object_t *obj)
{
    // You have polar coordinates for the object, convert them
    // to cartesian and take the distance from the object to the origin
    // For distance, all objects are always in front of you and the bot is facing straight up (90 degrees)
    float obj_center_rad = (obj->angle_end + obj->angle_start) * M_PI / (2.0 * 180.0);
    float obj_dist_to_servo = obj->dist_cm + CYBOT_SENSOR_OFFSET_cm; // distance from obj to center of servo axle

    // Ping only measures distance to the edge of the object, we want the center of it
    return DISTANCE(0, 0, obj_dist_to_servo * cosf(obj_center_rad), obj_dist_to_servo * sinf(obj_center_rad) + CYBOT_SERVO_OFFSET_cm) + obj->linear_width_cm / 2.0;
}

/**
 * @brief Convert servo angle to "polar" angle from center
 * of cybot. Right side is 0 degrees, like unit circle. This
 * is in *absolute* degrees, relative to our coordinate system
 * NOT the bot's heading.
 *
 * @param obj
 * @param dist_from_center
 * @return float Angle in degrees
 */
static float object_angle_from_center(object_t *obj, float dist_from_center)
{
    // You have a polar point, convert to cartesian and use arctan()
    // Make sure to account for the heading of the bot
    float bot_heading_offset_rad = (bot_heading_deg - 90) * M_PI / 180.0;
    float obj_center_rad = ((obj->angle_end + obj->angle_start) * M_PI / (2.0 * 180.0)) + bot_heading_offset_rad;
    float obj_dist_to_servo = obj->dist_cm + CYBOT_SENSOR_OFFSET_cm; // distance from obj to center of servo axle

    if (obj_center_rad == (M_PI / 2.0))
    {
        return 90.0;
    }
    else if (obj_center_rad == (3.0 * M_PI / 2.0)) {
        return 270.0;
    }

    return atan2(obj_dist_to_servo * sinf(obj_center_rad) + CYBOT_SERVO_OFFSET_cm, obj_dist_to_servo * cosf(obj_center_rad)) * 180.0 / M_PI;
}

/**
 * @brief Convert "polar" distance and angle (from center of
 * cybot) to cartesian offset from the cybot's position
 *
 * @param dist_cm
 * @param angle_deg
 * @return float
 */
static float get_x_offset(float dist_cm, float angle_deg)
{
    return dist_cm * cosf(angle_deg * M_PI / 180.0);
}

/**
 * @brief Convert "polar" distance and angle (from center of
 * cybot) to cartesian offset from the cybot's position
 *
 * @param dist_cm
 * @param angle_deg
 * @return float
 */
static float get_y_offset(float dist_cm, float angle_deg)
{
    return dist_cm * sinf(angle_deg * M_PI / 180.0);
}

/**
 * @brief Return true if obj already exists in the map given
 * the current location of [bot] and the state of the map
 *
 * @param obj Object to test
 */
static bool object_already_exists(object_t *obj)
{
    float dist_cm = object_dist_from_center(obj);
    float angle_deg = object_angle_from_center(obj, dist_cm);
    float position_x = bot->position_x + get_x_offset(dist_cm, angle_deg);
    float position_y = bot->position_y + get_y_offset(dist_cm, angle_deg);

    int i;
    for (i = 2; i < count; i++)
    {
        // Check if objects overlap
        float dist = DISTANCE(position_x, position_y, map[i].position_x, map[i].position_y);
        if (dist < (obj->linear_width_cm + 5.0)) // Add some tolerance
        {
            // If the object already exists, average the new measurement
            map[i].width_cm = (obj->linear_width_cm + map[i].width_cm) / 2.0;
            return true;
        }
    }
    return false;
}

// Returns the angle between the heading of the bot and the object, [-180, 180]
// Straight forward is 0 degrees. Used for turning
static float map_object_to_angle(map_object_t *obj)
{
    if (!obj)
    {
        return INFINITY;
    }

    static float dx, dy, angle; // This function is called quite a lot, leave these static
    dx = obj->position_x - bot->position_x;
    dy = obj->position_y - bot->position_y;

    // Make the bot the "origin", find the angle of the obj relative to
    // that (arctan()). Then, offset it by the heading of the bot.
    if (dy == 0.0)
    {
        return (dx > 0.0) ? 0.0 : 180.0;
    }
    angle = (atan2f(dy, dx) * 180.0 / M_PI) - bot_heading_deg;

    if (angle > 180.0)
    {
        angle -= 360.0;
    }
    else if (angle < -180.0)
    {
        angle += 360.0;
    }
    return angle;
}

static float map_object_to_distance(map_object_t *obj)
{
    if (!obj)
    {
        return INFINITY;
    }
    // Distance formula
    return DISTANCE(bot->position_x, bot->position_y, obj->position_x, obj->position_y);
}

/*
 *
 * GLOBAL FUNCTIONS
 *
 */

void map_init()
{
    static int initialized = 0;
    if (initialized)
    {
        return;
    }

    SET_MAP_OBJECT_START(map[MAP_START_IDX]);
    SET_MAP_OBJECT_BOT(map[MAP_BOT_IDX]);
}

void map_updateBotPosition(float distance_forwards_cm, float heading_deg)
{
    // Just polar to cartesian again. You have a distance and a heading from where you started
    bot_heading_deg = bot_heading_deg + heading_deg;
    if (bot_heading_deg >= 360.0)
    {
        bot_heading_deg -= 360.0;
    }
    if (bot_heading_deg < 0.0)
    {
        bot_heading_deg += 360.0;
    }
    if (distance_forwards_cm != 0.0)
    {
        bot->position_x += get_x_offset(distance_forwards_cm, bot_heading_deg);
        bot->position_y += get_y_offset(distance_forwards_cm, bot_heading_deg);
    }
}

void map_addShortObject(bool left_bump, bool right_bump)
{
    object_t short_obj;
    short_obj.dist_cm = 10;
    short_obj.linear_width_cm = 20;

    // All sizes and locations are approximate. Best we can do is guess
    if (left_bump && right_bump)
    {
        map[count].position_x = bot->position_x;
        map[count].position_y = bot->position_y + 25;
        short_obj.angle_start = 80;
        short_obj.angle_end = 110;
    }
    else if (left_bump)
    {
        map[count].position_x = bot->position_x - 25;
        map[count].position_y = bot->position_y + 10;
        short_obj.angle_start = 130;
        short_obj.angle_end = 150;
    }
    else if (right_bump)
    {
        map[count].position_x = bot->position_x + 25;
        map[count].position_y = bot->position_y + 10;
        short_obj.angle_start = 40;
        short_obj.angle_end = 60;
    }
    map[count].type = MAP_OBJECT_SHORT;
    map[count].width_cm = 20; // Overestimate here
    if (!object_already_exists(&short_obj))
    {
        count++; // Only keep the object if it doesn't already exist
    }
}

void map_addTallObjects(object_t *objects, int len_objects)
{
    int i;
    for (i = 0; i < len_objects; i++)
    {
        if (!object_already_exists(&objects[i]))
        {
            float dist_cm = object_dist_from_center(&objects[i]);
            float angle_deg = object_angle_from_center(&objects[i], dist_cm);
            map[count].position_x = bot->position_x + get_x_offset(dist_cm, angle_deg);
            map[count].position_y = bot->position_y + get_y_offset(dist_cm, angle_deg);
            map[count].type = MAP_OBJECT_TALL;
            map[count].width_cm = objects[i].linear_width_cm;
            count++;
        }
    }
}

void map_addBoundaryOrHole()
{
    return;
}

bool map_foundLargestObj()
{
    int i;
    for (i = 0; i < count; i++)
    {
        if (IN_TOLERANCE(map[i].width_cm, LARGEST_OBJ_SIZE_cm, 2.0))
        {
            largest = &map[i];
            return true;
        }
    }
    return false;
}

float map_getLargestObjAngle()
{
    return map_object_to_angle(largest);
}

float map_getLargestObjDistance()
{
    return map_object_to_distance(largest) - CYBOT_DIAMETER_cm / 2.0 - largest->width_cm / 2.0;
}

float map_getStartAngle()
{
    return map_object_to_angle(start);
}

float map_getStartDistance()
{
    return map_object_to_distance(start);
}

float map_getDistanceToNextObj()
{
    // You can project two lines out from the left/right sides of the bot
    // in the direction it's facing. Now test every object we've found to
    // see if any fall between them. The test: "Is the distance from the
    // object to either of these lines less than the radius of the object?"
    // Thanks wikipedia! https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points

    // Two pairs of points, for the left and right sides of the bot
    // Pick the second point an arbitrarily far distance away, it doesn't matter
    static float left_x1, left_x2, left_y1, left_y2, right_x1, right_x2, right_y1, right_y2;
    left_x1 = bot->position_x + get_x_offset(CYBOT_DIAMETER_cm / 2.0, bot_heading_deg + 90);
    left_x2 = left_x1 + get_x_offset(100.0, bot_heading_deg);
    left_y1 = bot->position_y + get_y_offset(CYBOT_DIAMETER_cm / 2.0, bot_heading_deg + 90);
    left_y2 = left_y1 + get_y_offset(100.0, bot_heading_deg);
    right_x1 = bot->position_x + get_x_offset(CYBOT_DIAMETER_cm / 2.0, bot_heading_deg - 90);
    right_x2 = left_x1 + get_x_offset(100.0, bot_heading_deg);
    right_y1 = bot->position_y + get_y_offset(CYBOT_DIAMETER_cm / 2.0, bot_heading_deg - 90);
    right_y2 = left_y1 + get_y_offset(100.0, bot_heading_deg);

    float left_denom = 100.0; // Distance between *1 and *2, set when setting the *2 coordinates
    float right_denom = 100.0;
    const float wiggle_room = 5.0; // I don't trust our measurements 100%

    float shortest_dist = 1000000.0;

    int i;
    for (i = MAP_BOT_IDX + 1; i < count; i++)
    {
        if (fabsf(map_object_to_angle(&map[i])) > 90.0)
        {
            continue; // Object is behind us, we don't care
        }

        // Distances are from center of obj to the two lines
        float left_dist = fabsf((left_y2 - left_y1) * map[i].position_x - (left_x2 - left_x1) * map[i].position_y + left_x2 * left_y1 - left_y2 * left_x1) / left_denom;
        float right_dist = fabsf((right_y2 - right_y1) * map[i].position_x - (right_x2 - right_x1) * map[i].position_y + right_x2 * right_y1 - right_y2 * right_x1) / right_denom;
        // Check against the edges of the object to see if we'll hit it
        if (left_dist < ((map[i].width_cm / 2) + wiggle_room) || right_dist < ((map[i].width_cm / 2) + wiggle_room))
        {
            // Radial distance from the center of the bot, slightly inaccurate
            float edge_to_edge_dist = map_object_to_distance(&map[i]) - (CYBOT_DIAMETER_cm / 2.0) - (map[i].width_cm / 2.0);

            if (edge_to_edge_dist < shortest_dist)
            {
                shortest_dist = edge_to_edge_dist;
            }
        }
    }
    return shortest_dist;
}

void map_sendMapToUart()
{
    static char buf[75];
    static const char *UNKNOWN = "UNKNOWN";
    static const char *START = "START";
    static const char *BOT = "BOT";
    static const char *TALL = "TALL";
    static const char *SHORT = "SHORT";
    static const char *HOLE = "HOLE";
    static const char *BOUNDARY = "BOUNDARY";
    static char *type;

    int i;
    for (i = 0; i < count; i++)
    {
        switch (map[i].type)
        {
        case MAP_OBJECT_UNKNOWN:
            type = (char *)UNKNOWN;
            break;
        case MAP_OBJECT_START:
            type = (char *)START;
            break;
        case MAP_OBJECT_BOT:
            type = (char *)BOT;
            break;
        case MAP_OBJECT_TALL:
            type = (char *)TALL;
            break;
        case MAP_OBJECT_SHORT:
            type = (char *)SHORT;
            break;
        case MAP_OBJECT_HOLE:
            type = (char *)HOLE;
            break;
        case MAP_OBJECT_BOUNDARY:
            type = (char *)BOUNDARY;
            break;
        }
        sprintf(buf, "MapObj: %s, x: %.2f, y: %.2f, width: %.2f", type, map[i].position_x, map[i].position_y, map[i].width_cm);
        uart_sendLine(buf);
    }
}

float map_getBotHeading() {
    return bot_heading_deg;
}
