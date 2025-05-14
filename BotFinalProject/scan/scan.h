#ifndef SCAN_H_
#define SCAN_H_

#include <stdbool.h>
#include "../movement/open_interface.h"

typedef struct
{
    int angle_start;
    int angle_end;
    float dist_cm;
    float linear_width_cm;
} object_t;

// Initialize scan (call once)
void scan_init(oi_t *cybot_ptr);

// Calibrate the servo and IR blaster, then wait forever
void scan_calibrate();

// Scan a 180 degree view of the front of the bot
void scan_scan180(bool log_to_uart);

// Detect objects from the previous scan, optionally log to uart
// Returns number of objects detected
int scan_detectObjects(bool log_to_uart);

// Return the smallest linear width object
object_t *scan_getSmallestObj(bool log_to_uart);

void scan_pointToSmallest();

// Get the pointer to the object buffer. This is overwritten after
// every call to scan_detectObjects()
object_t * scan_getObjectBuf();

#endif
