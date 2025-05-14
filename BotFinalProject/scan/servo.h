#ifndef SERVO_H_
#define SERVO_H_

/**
 * @brief Initialize PWM output for driving servo
 *
 * @param cal_0_ 0 degree calibration value from servo_cal()
 * @param cal_180_ 180 degree calibration value from servo_cal()
 */
void servo_init(unsigned int cal_0_, unsigned int cal_180_);

/**
 * @brief Move the servo to a particular angle,
 * busy wait while it moves
 *
 * @param angle Angle in degrees (0-180 inclusive)
 */
void servo_move(unsigned int angle);

/**
 * @brief Servo calibration program, will exit once
 * calibration is finished.
 *
 */
void servo_cal();

#endif
