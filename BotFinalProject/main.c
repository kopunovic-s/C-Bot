#include <stdbool.h>

#include "scan/scan.h"
#include "navigation/navigation.h"
#include "uart/uart.h"
#include "utils/lcd.h"
#include "utils/Timer.h"
#include "movement/open_interface.h"
#include "movement/movement.h"

bool autonomous = false;

int main(void)
{
    timer_init();
    lcd_init(); // Must be initialized in main, not multiple-initialization safe
    uart_init();

    oi_t *cybot = oi_alloc(); // Initialize globally so it doesn't happen multiple times
    oi_init(cybot);
    oi_setWheels(0, 0);

    nav_init(cybot);

    scan_init(cybot);

    bool run = true;
    while (run)
    {
        while (!uart_dataPresent())
            ;

        char izard = uart_receive();
        switch (izard)
        {
        case 't':
            nav_run();
            break;
        case 'm':
            scan_scan180(true);
            scan_detectObjects(true);
            scan_getSmallestObj(true);
            break;
        case 'w':
            mvmt_move(cybot, 50);
            break;
        case 's':
            mvmt_move(cybot, -50);
            break;
        case 'a':
            mvmt_turn(cybot, 30);
            break;
        case 'd':
            mvmt_turn(cybot, -30);
            break;
        case 'q':
            run = false;
            break;
        default:
            break;
        }
    }
    oi_free(cybot);
}
