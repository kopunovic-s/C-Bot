#include "ir.h"

#include <inc/tm4c123gh6pm.h>
#include "../utils/Timer.h"

void ir_init()
{
    static int initialized = 0;
    if (initialized)
    {
        return;
    }

    // IR blaster is on PB4 -> AIN10
    // Guessing ADC0 for now
    SYSCTL_RCGCGPIO_R |= 0b10;
    SYSCTL_RCGCADC_R |= 0b1;
    timer_waitMillis(1);

    GPIO_PORTB_AFSEL_R |= 0b10000;  // Enable alternate func
    GPIO_PORTB_DEN_R &= ~(0b10000); // Disable digital operation
    GPIO_PORTB_AMSEL_R |= 0b10000;  // Enable analog mode

    // Set up a sample sequence with 1 input in it, then read it
    // ** Specifically want to nuke the rest of SEQ 0 so we don't get
    // some other data coming through
    ADC0_SSMUX0_R = 10u;
    ADC0_SSCTL0_R = 0b0110; // Enable interrupt, end of sequence
    ADC0_SAC_R = 0x3;       // x8 averaging
    ADC0_ACTSS_R |= 0b1;

    initialized = 1;
}

int ir_read()
{
    ADC0_PSSI_R |= 0b1;         // Start SEQ 0
    while (!(ADC0_RIS_R & 0b1)) // Wait for conversion
        ;
    ADC0_ISC_R = 0b1;
    return (ADC0_SSFIFO0_R & 0xFFF); // Grab the result, lower 12 bits
}
