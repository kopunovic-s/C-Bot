#ifndef IR_H_
#define IR_H_

/**
 * @brief Initialize the ADC for reading the IR sensor
 *
 */
void ir_init();

/**
 * @brief Read from the IR sensor
 *
 * @return int IR sensor reading
 */
int ir_read();

#endif
