#ifndef PING_H_
#define PING_H_

/**
 * @brief Initialize timer capture for the PING sensor
 *
 */
void ping_init();

/**
 * @brief Read raw measurement pulse width from PING sensor,
 * in timer clock cycles
 *
 * @return unsigned int
 */
unsigned int ping_getRawDist();

/**
 * @brief Send a ping, measure the distance, and return
 *
 * @return float Distance in cm
 */
float ping_getDist();

#endif
