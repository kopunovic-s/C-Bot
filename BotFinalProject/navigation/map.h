#ifndef MAP_H_
#define MAP_H_

#include "../scan/scan.h"
#include <stdbool.h>

/**
 * @brief Initialize the map
 *
 * Map is just an array of objects, including the start location
 * and the bot. Everything is referenced off of the start point (0, 0).
 *
 */
void map_init();

/**
 * @brief Update the bot's position on the map. Essentially takes in a
 * vector (with *direction* and *magnitude*, oh yeah) and updates where
 * the bot should be
 *
 * @param distance_forward_cm Distance traveled since the last call to
 * map_updateBotPosition(). Positive is forwards, negative is backwards, 0
 * is "didn't move"
 * @param heading_deg Change in heading since the last call to
 * map_updateBotPosition(). Positive is counter-clockwise/left, 0 is "unchanged
 * from last call" (i.e. you went forward)
 */
void map_updateBotPosition(float distance_forward_cm, float heading_deg);

/**
 * @brief Add a short object to the map. We basically guess where
 * it is based on the location of the bot and which side it hit.
 * Hitting both bump sensors means we hit head-on.
 *
 * @param left_bump Left bump sensor state
 * @param right_bump Right bump sensor state
 */
void map_addShortObject(bool left_bump, bool right_bump);

/**
 * @brief Add tall objects from a scan to the map.
 *
 * @param objects List of objects from a scan
 * @param len_objects Length of array
 */
void map_addTallObjects(object_t *objects, int len_objects);

/**
 * @brief Adds a boundary or a hole to the map
 *
 */
void map_addBoundaryOrHole();

/**
 * @brief Return true if we've found the largest object
 * (largest tall object has a known diameter, we'll know when we
 * see it). If we've found multiple, return the first one we found.
 *
 * @return True if we've found the tallest object, false otherwise
 */
bool map_foundLargestObj();

/**
 * @brief Given the bot's current position and heading, return the
 * angle between us and the largest object (0 degrees is to the right).
 * Used to nav to the largest object once we've found it.
 *
 * @return float Angle in degrees, [-180, 180]
 */
float map_getLargestObjAngle();

/**
 * @brief Given the bot's current position and heading, return the
 * distance between us and the largest object.
 *
 * @return float Distance in cm
 */
float map_getLargestObjDistance();

/**
 * @brief Given the bot's current position and heading, return the
 * angle between us and the start point (0 degrees is to the right).
 * Used to return home.
 *
 * @return float Angle in degrees, [-180, 180]
 */
float map_getStartAngle();

/**
 * @brief Given the bot's current position and heading, return the
 * distance between us and the start point. Used to nav back
 * to where we started.
 *
 * @return float Distance in cm
 */
float map_getStartDistance();

/**
 * @brief Given the bot's current position and heading, return
 * the distance to the next object it will run into (not including
 * the start zone). Used to make sure we don't crash into objects
 * as we explore around
 *
 * @return float Distance in cm
 */
float map_getDistanceToNextObj();

/**
 * @brief Send the map to the UI over UART
 *
 */
void map_sendMapToUart();

float map_getBotHeading();

#endif
