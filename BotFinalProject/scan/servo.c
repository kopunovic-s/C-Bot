#include "servo.h"

#include <stdbool.h>
#include "../utils/lcd.h"
#include "../utils/Timer.h"
#include "../utils/button.h"

// Servo is on PB5, TIM1 slice B

#define PERIOD_TICKS (320000)

// Calibration values in TIMER TICKS!
static unsigned int cal_0 = PERIOD_TICKS - 1000 * 16;
static unsigned int cal_180 = PERIOD_TICKS - 2000 * 16;

static void set_angle(unsigned int angle_deg)
{
    // 1ms + 1/180th ms per degree of servo angle
    // Adjust for calibration
    unsigned int match = cal_0 - (angle_deg * (cal_0 - cal_180)) / 180;

    // 24 bit val spread across match and prescaler
    TIMER1_TBMATCHR_R = (match & 0x00FFFF);
    TIMER1_TBPMR_R = (match & 0xFF0000) >> 16;
}

static void set_match(unsigned int match)
{
    TIMER1_TBMATCHR_R = (match & 0x00FFFF);
    TIMER1_TBPMR_R = (match & 0xFF0000) >> 16;
}

static unsigned int get_match()
{
    return ((TIMER1_TBPMR_R & 0xFF) << 16) | (TIMER1_TBMATCHR_R & 0xFFFF);
}

void servo_init(unsigned int cal_0_, unsigned int cal_180_)
{
    static int initialized = 0;
    if (initialized)
    {
        return;
    }

    cal_0 = cal_0_;
    cal_180 = cal_180_;

    SYSCTL_RCGCGPIO_R |= 0b10;
    SYSCTL_RCGCTIMER_R |= 0b10;
    timer_waitMillis(10);

    GPIO_PORTB_DEN_R |= 0b100000;
    GPIO_PORTB_AFSEL_R |= 0b100000;
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & ~0xF00000) | 0x700000;

    TIMER1_CTL_R &= ~0x100; // Stop timer B
    TIMER1_CFG_R = 0x4;
    TIMER1_TBMR_R = 0b1010; // PWM mode, no capture, periodic

    // Counter only counts DOWN
    // In non-inverted mode, the output starts HIGH
    // and falls low when TBR = BMR
    // Period = 20ms, pulse width = 1-2ms
    TIMER1_TBILR_R = (PERIOD_TICKS & 0x00FFFF); // 20ms in clock cycles
    TIMER1_TBPR_R = (PERIOD_TICKS & 0xFF0000) >> 16;
    set_angle(90);

    TIMER1_CTL_R |= 0x100; // Start

    initialized = 1;
    timer_waitMillis(375); // Wait for the servo to move to 90 degrees
}

void servo_move(unsigned int angle_deg)
{
    static unsigned int last_angle_deg = 90;
    if (angle_deg > 180)
    {
        angle_deg = 180;
    }
    set_angle(angle_deg);

    // Servo takes about 750ms to do the full 180 degrees. Scale off of this
    int delay_ms = ((int)angle_deg - (int)last_angle_deg) * 750 / 180;
    if(delay_ms < 0) {
        delay_ms = -delay_ms;
    }
    if(delay_ms < 50) {
        delay_ms = 50; // 50ms lower bound
    }
    timer_waitMillis(delay_ms);
    last_angle_deg = angle_deg;
}

// TODO rewrite this to be better, this was written per the lab
void servo_cal()
{
    int dir = -1; // -1 for CCW, 1 for CW
    unsigned int match = PERIOD_TICKS - 1500 * 16;
    set_match(match);

    while (1)
    {
        if (dir == 1)
        {
            lcd_printf("+1   +5     Dir  End\nDir: CW\nRaw: %d", get_match());
        }
        else
        {
            lcd_printf("+1   +5     Dir  End\nDir: CCW\nRaw: %d", get_match());
        }

        button_waitForPress();
        int button = button_getButton();
        button_waitForRelease();

        switch (button)
        {
        case 1:                          // +1
            match += dir * 1 * (5 * 16); // Add 5us
            break;
        case 2:                          // +5
            match += dir * 5 * (5 * 16); // Add 5 * 5us
            break;
        case 3: // Swap direction
            dir = -dir;
            break;
//        case 4: // Move to 0/180, this is what the lab wants
//            match = (dir == -1) ? cal_180 : cal_0;
//            break;
        case 4:
            return;
        }
        set_match(match);
    }
}
