#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include <stdbool.h>
#include "../movement/open_interface.h"

void nav_init(oi_t *cybot_ptr);

/**
 * @brief Go cybot go!
 *
 * Find the largest thing in the test field,
 * play a sound, then return to the start point.
 * Return if there is an exception of some sort.
 *
 */
void nav_run();

/**
 * @brief Emergency-stop autonomous mode
 *
 */
void nav_emergencyStop();

#endif
