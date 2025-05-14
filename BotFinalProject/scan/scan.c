#include "scan.h"

#include <stdio.h>
#include <math.h>
#include "ir.h"
#include "ping.h"
#include "servo.h"
#include "../utils/button.h"
#include "../utils/lcd.h"
#include "../utils/Timer.h"
#include "../uart/uart.h"
#include "../movement/movement.h"

/// ** CALIBRATION VALUES **
// !!! CYBOT 2 !!!

#define SERVO_CAL_LEFT (284880)
#define SERVO_CAL_RIGHT (313680)

#define IR_RAW_10cm (2387)
#define IR_RAW_50cm (706)

#define IR_CYBOT_OFFSET_MM (35) // Distance from IR to front of cybot

/// ************************

static oi_t *cybot = {0};

#define LOG_LINE_BUF_LEN (75)
static char log_line_buf[LOG_LINE_BUF_LEN] = {0};

// Scan distance data
#define SCAN_STEP_DEGREES (2)
#define SCAN_REPEAT_CNT (2)
#define SCAN_BUF_LEN_90 (90 / SCAN_STEP_DEGREES)
#define SCAN_BUF_LEN_180 (180 / SCAN_STEP_DEGREES)
#define SCAN_BUF_LEN_360 (360 / SCAN_STEP_DEGREES)
#define SCAN_IR_LOWER_BOUND (IR_RAW_50cm - 200) // Ignore all readings below this value, they're unreliable
#define SCAN_IR_INVALID_VAL (-1.0)              // Magic distance for invalid reading
static float scan_ping_cm[SCAN_BUF_LEN_360] = {0.0};
static float scan_ir_cm[SCAN_BUF_LEN_360] = {0.0};
static int last_scan_cnt = SCAN_BUF_LEN_180; // How much data the last scan created

// Detected object data
#define OBJECTS_LEN (10) // Picked at random, adjust as needed
#define OBJECT_DISTANCE_TOL (5)
#define OBJECT_MIN_ANGLE (6 / SCAN_STEP_DEGREES)
static object_t objects[OBJECTS_LEN] = {0};
static int objects_detected = 0;

/*
 *
 * STATIC FUNCTIONS
 *
 */

// see IR datasheet last page
static float ir_invert(float raw)
{
    return 1.0 / (raw + 0.42);
}

static float ir_raw_to_dist(int raw)
{
    // !!! The graph in the datasheet is in VOLTS, all math here is in ADC ticks
    // !!! Everything is also inverted, the graph is Volts/cm and we want cm/Volt
    // two points: (scan_50cm.IR_raw_val, 50) and (scan_10cm.IR_raw_val, 10)
    float slope = (ir_invert(10) - ir_invert(50)) / (IR_RAW_10cm - IR_RAW_50cm); // cm/tick

    // Point-slope form, point is (scan_10cm.IR_raw_val, 10)
    float dist_inverse = (raw - IR_RAW_10cm) * slope + ir_invert(10);
    float dist = 1.0 / dist_inverse - 0.42;
    return dist;
}

static float object_get_linear_width(object_t *obj)
{
    // Law of cosines
    int angle = obj->angle_end - obj->angle_start;
    return sqrt(obj->dist_cm * obj->dist_cm + obj->dist_cm * obj->dist_cm - 2 * obj->dist_cm * obj->dist_cm * cos((angle * M_PI) / 180));
}

/*
 *
 *   GLOBAL FUNCTIONS
 *
 */

void scan_init(oi_t *cybot_ptr)
{
    static int initialized = 0;
    if (initialized)
    {
        return;
    }

    cybot = cybot_ptr;

    uart_init(); // For logging
    ir_init();
    ping_init();
    servo_init(SERVO_CAL_RIGHT, SERVO_CAL_LEFT);

    initialized = 1;
}

void scan_calibrate()
{
    button_init(); // No lcd init, it's not multiple-initialization safe
    timer_init();

    servo_cal();
    button_waitForRelease();
    button_waitForPress(); // Press any button to start IR cal
    button_waitForRelease();

    lcd_clear();
    lcd_home();
    lcd_printf("Set obj touching bot");
    button_waitForPress();
    button_waitForRelease();

    mvmt_move(cybot, -1 * (100 - IR_CYBOT_OFFSET_MM)); // Move so the IR is 10cm away from object
    timer_waitMillis(250);
    servo_move(90);
    int raw_10cm = ir_read();

    mvmt_move(cybot, -400); // Move so IR is 50cm away
    timer_waitMillis(250);
    servo_move(90);

    lcd_clear();
    lcd_home();
    lcd_printf(" 50cm: %d\n 10cm: %d", ir_read(), raw_10cm);
    while (1)
        ; // Loop forever, you have to change the macros at the top of the file and reprogram
}

void scan_scan180(bool log_to_uart)
{
    // Send header
    if (log_to_uart)
    {
        uart_sendLine("Degrees    Ping Distance (cm)     IR Distance (cm)");
    }

    last_scan_cnt = SCAN_BUF_LEN_180;
    int angle;
    for (angle = 0; angle <= 180; angle += SCAN_STEP_DEGREES)
    {
        // Take multiple scans at the same spot and avg them together
        int ir_raw_sum = 0;
        float ping_raw_sum = 0.0;
        int subscan;
        for (subscan = 0; subscan < SCAN_REPEAT_CNT; subscan++)
        {
            servo_move(angle);
            ping_raw_sum += ping_getDist();
            ir_raw_sum += ir_read();
        }

        scan_ping_cm[angle / SCAN_STEP_DEGREES] = ping_raw_sum / SCAN_REPEAT_CNT;
        scan_ir_cm[angle / SCAN_STEP_DEGREES] = ir_raw_to_dist(ir_raw_sum / SCAN_REPEAT_CNT);

        if (log_to_uart)
        {
            sprintf(log_line_buf, "%d      %.2f    %.2f", angle,
                    scan_ping_cm[angle / SCAN_STEP_DEGREES], scan_ir_cm[angle / SCAN_STEP_DEGREES]);
            uart_sendLine(log_line_buf);
        }
    }
}

int scan_detectObjects(bool log_to_uart)
{
    // Find changes in distance, record each as an object
    objects_detected = 0;
    float current_dist = scan_ir_cm[0];
    int found_object = 0;
    objects[0].angle_start = 0;
    bool ignore_scan_start = true; // Remove objects at the start of a scan, they're unreliable

    int i;
    for (i = 1; i < last_scan_cnt; i++)
    {
        // Look for flat spots, that means we're still looking at an object
        // Set bounds at 0.0 < dist < 100.0 so we end objects when they go out of IR range
        if (scan_ir_cm[i] < (current_dist + OBJECT_DISTANCE_TOL) && scan_ir_cm[i] > (current_dist - OBJECT_DISTANCE_TOL) && scan_ir_cm[i] > 0.0 && scan_ir_cm[i] < 100.0)
        {
            if(!ignore_scan_start) {
                found_object++;
                current_dist = scan_ir_cm[i];
            }
        }
        else // Object has ended or we're looking at noise
        {
            ignore_scan_start = false;
            if (found_object >= OBJECT_MIN_ANGLE)
            {
                objects[objects_detected].angle_end = i * SCAN_STEP_DEGREES;
                // Check the ping sensor reading in the middle of the obj for distance
                int middle_angle = (objects[objects_detected].angle_end + objects[objects_detected].angle_start) / 2;
                objects[objects_detected].dist_cm = scan_ping_cm[middle_angle / SCAN_STEP_DEGREES];
                objects[objects_detected].linear_width_cm = object_get_linear_width(&objects[objects_detected]);
                objects_detected++;

                if (objects_detected >= OBJECTS_LEN)
                {
                    uart_sendLine("Too many objects found");
                    break;
                }
            }
            found_object = 0;
            objects[objects_detected].angle_start = i * SCAN_STEP_DEGREES;
            current_dist = scan_ir_cm[i];
        }
    }

    if (log_to_uart)
    {
        uart_sendLine("Object#    Angle(deg)     Distance(cm)   Width (deg)");
        for (i = 0; i < objects_detected; i++)
        {
            sprintf(log_line_buf,
                    "%d          %d          %.2f          %d", i,
                    objects[i].angle_start, objects[i].dist_cm,
                    (objects[i].angle_end - objects[i].angle_start));
            uart_sendLine(log_line_buf);
        }
        sprintf(log_line_buf, "Objects found: %d", objects_detected);
        uart_sendLine(log_line_buf);
    }
    return objects_detected;
}

object_t *scan_getSmallestObj(bool log_to_uart)
{
    if (!objects_detected)
    {
        return NULL;
    }

    object_t *smallest = &objects[0];
    float smallest_width = objects[0].linear_width_cm;
    int i;
    for (i = 0; i < objects_detected; i++)
    {
        if (objects[i].linear_width_cm < smallest_width)
        {
            smallest_width = objects[i].linear_width_cm;
            smallest = &objects[i];
        }
    }
    if (log_to_uart)
    {
        sprintf(log_line_buf, "Smallest object found at %d to %d, width %.2f, dist %.2f",
                smallest->angle_start, smallest->angle_end, smallest_width, smallest->dist_cm);
        uart_sendLine(log_line_buf);
    }
    return smallest;
}

void scan_pointToSmallest()
{
    object_t *smallest = scan_getSmallestObj(false);
    servo_move((smallest->angle_end + smallest->angle_start) / 2);
}

object_t *scan_getObjectBuf()
{
    return objects;
}
