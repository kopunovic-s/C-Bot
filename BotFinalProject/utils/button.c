#include "button.h"

#include <stdbool.h> // Needed for interrupt.h
#include <stdint.h> // Needed for interrupt.h
#include <inc/tm4c123gh6pm.h>
#include "driverlib/interrupt.h"

// Button 1-4 are mapped to PORTE 0-3

static volatile int button_event;
static volatile int button_num;

static void gpioe_handler()
{
	// Clear interrupt status register
	GPIO_PORTE_ICR_R = 0x0F;
	button_event = 1;
	button_num = 0;
	// 4 (PE3) is on the right, 1 (PE0) is on the left
	// Could for this, but this is easier to read
	if (!(GPIO_PORTE_DATA_R & 0x08))
	{
		button_num = 4;
	}
	if (!(GPIO_PORTE_DATA_R & 0x04))
	{
		button_num = 3;
	}
	if (!(GPIO_PORTE_DATA_R & 0x02))
	{
		button_num = 2;
	}
	if (!(GPIO_PORTE_DATA_R & 0x01))
	{
		button_num = 1;
	}
}

void button_init()
{
	static int initialized = 0;

	if (initialized)
	{
		return;
	}

	// 1) Turn on PORTE system clock, do not modify other clock enables
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;

	// 2) Set the buttons as inputs, do not modify other PORTE wires
	GPIO_PORTE_DIR_R &= ~(0x0F);

	// 3) Enable digital functionality for button inputs,
	//    do not modify other PORTE enables
	GPIO_PORTE_DEN_R |= 0x0F;

	// 1) Mask the bits for pins 0-3 (masked/cleared by default, but whatever)
	GPIO_PORTE_IM_R &= ~(0x0F);

	// 2) Set pins 0-3 to use edge sensing
	GPIO_PORTE_IS_R &= ~(0x0F);

	// 3) Set pins 0-3 to use both edges. We want to update the LCD
	//    when a button is pressed, and when the button is released.
	GPIO_PORTE_IBE_R |= 0x0F;

	// 4) Clear the interrupts
	GPIO_PORTE_ICR_R = 0x0F;

	// 5) Unmask the bits for pins 0-3
	GPIO_PORTE_IM_R |= 0x0F;

	// 6) Enable GPIO port E interrupt (interrupt table on pg 104)
	NVIC_EN0_R |= (1u << 4);

	// Bind the interrupt to the handler.
	IntRegister(INT_GPIOE, gpioe_handler);

	initialized = 1;
}

int button_getButton()
{
	button_event = 0;
	return button_num;
}

int button_eventHappened()
{
	return button_event;
}

void button_waitForPress()
{
	while (!button_num)
		;
}

void button_waitForRelease()
{
	while (button_num)
		;
}
