#include "ping.h"

#include <inc/tm4c123gh6pm.h>
#include <stdbool.h>
#include "../utils/Timer.h"

// PING sensor pin: PB3, T3CCP1

#define PING_TIMEOUT_CYCLES (300000) // 18.75ms
#define CYCLES_PER_uS (16)           // 16MHz CPU clock
#define SPEED_OF_SOUND_cm_us (0.034) // 340 m/s -> cm/us

static unsigned int get_edge_timestamp()
{
    // 24 bit value, all accessible through TBR (this is really unclear)
    return (TIMER3_TBR_R & 0xFFFFFF);
}

static unsigned int get_timer_count()
{
    // 24 bit value, all accessible through TBV (this is really unclear)
    return (TIMER3_TBV_R & 0xFFFFFF);
}

// True if within range, false otherwise
static bool is_timer_within_range(unsigned int start, unsigned int range)
{
    // Compensate for rollover
    unsigned int timer = get_timer_count();
    if (start < timer)
    {
        start += (1u << 24);
    }

    return ((start - timer) < range);
}

void ping_init()
{
    static int initialized = 0;
    if (initialized)
    {
        return;
    }

    SYSCTL_RCGCGPIO_R |= 0b10;
    SYSCTL_RCGCTIMER_R |= 0b1000;
    timer_waitMillis(1);

    // Configure for digital output muxed to TIM3, leave AFSEL unset
    GPIO_PORTB_DIR_R |= 0b1000;
    GPIO_PORTB_DEN_R |= 0b1000;
    GPIO_PORTB_DATA_R &= ~0b1000; // Explicitly drive the line low
    GPIO_PORTB_AFSEL_R &= ~0b1000;
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & ~0xF000) | (7u << 12);

    // Configure timer for pulse width capture (edge time mode)
    // Ping sensor is on TIM3, CCP1 (timer B)
    TIMER3_CTL_R &= ~0x100; // Stop timer B
    TIMER3_CFG_R = 0x4;
    TIMER3_TBMR_R = 0b111;       // !!! Count down mode, Capture mode, edge time mode
    TIMER3_CTL_R |= (0x1 << 10); // Capture *both* edges
    // No prescaler available for edge time mode, instead we get a 24 bit counter
    TIMER3_TBILR_R = 0xFFFF; // Edge-time mode is 24-bit, prescaler holds the upper 8 bits
    TIMER3_TBPR_R = 0xFF;
    TIMER3_CTL_R |= 0x100; // Start timer B

    initialized = 1;
}

unsigned int ping_getRawDist()
{
    // Send pulse
    GPIO_PORTB_AFSEL_R &= ~0b1000;
    GPIO_PORTB_DIR_R |= 0b1000;
    GPIO_PORTB_DATA_R |= 0b1000;
    timer_waitMicros(5);
    GPIO_PORTB_DATA_R &= ~0b1000;
    GPIO_PORTB_DIR_R &= ~0b1000;
    GPIO_PORTB_AFSEL_R |= 0b1000; // Hand it back to the timer

    /*
     * THE PING SENSOR SUCKS WHAT THE HELL IS THIS
     * IT DOESN'T FOLLOW ITS OWN DATASHEET AHHHHHHHHHHH
     *
     * Important info: 48 timer ticks (4us) is 1mm, 480 timer ticks (40us) is 1cm
     * 1cm precision is about what we're targeting.
     *
     * This stupid sensor sometimes doesn't actually ping. It will just sit floating
     * (with the green light on) forever.
     *
     * Also, sometimes the stupid sensor doesn't go to GND before the rising
     * edge of its output pulse. It will just float then jump up to 5V. Funny bit here
     * is that floating *can* be a 1 or a 0, so this *may* not register as an edge to
     * the micro.
     *
     * So, we're only going to look for *falling* edges. We're going to assume
     * that the 750us holdoff from the end of *our* pulse is correct, and then
     * look for only falling edges after that. We have quite a lot of wiggle room
     * in the timing (see: 480us is 1cm) so a few us here or there shouldn't matter.
     */

    timer_waitMicros(750);
    volatile unsigned int pulse_start = get_timer_count();

    // Sigh... this has to be busy anyway (we have to
    // wait for the IRQ to fire), might as well poll it

    volatile static const int pulse_timeout = 296000; // 18ms in timer counts
    TIMER3_ICR_R |= (1u << 10);                       // Clear CBECINT
    while (!(TIMER3_RIS_R & (1u << 10)) && is_timer_within_range(pulse_start, pulse_timeout))
        ;
    if (!is_timer_within_range(pulse_start, pulse_timeout))
    {
        return 0;
    }
    volatile unsigned int pulse_end = get_edge_timestamp();

    // No overflow checking needed, we handle timeouts above
    if (pulse_start < pulse_end)
    {
        // handle when rising edge is very close to 0 and timer underflows
        // before falling edge
        pulse_start += (1u << 24);
    }

    return (pulse_start - pulse_end);
}

float ping_getDist()
{
    // (timer / 16) = us
    // us * 0.0343 = distance in cm of the round-trip ping (to the obj and back)
    // distance/2 = distance to obj
    return ((ping_getRawDist() >> 5) * SPEED_OF_SOUND_cm_us);
}
