#ifndef BUTTON_H_
#define BUTTON_H_

/**
 * @brief Initialize GPIO for button input.
 *
 */
void button_init();

/**
 * @brief Returns the rightmost button being pushed, clears a button event.
 *
 * @return int Rightmost button being pressed. 4 is rightmost,
 * 1 is leftmost, 0 indicates no button pressed.
 */
int button_getButton();

/**
 * @brief Check if a button has changed since the last
 * call to button_eventHappened().
 *
 * @return int Nonzero if a button changed, zero otherwise.
 */
int button_eventHappened();

/**
 * @brief Busy wait for a button press.
 *
 */
void button_waitForPress();

/**
 * @brief Busy wait for a button release.
 *
 */
void button_waitForRelease();

#endif
