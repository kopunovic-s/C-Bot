#include "uart.h"

#include <inc/tm4c123gh6pm.h>
#include "../utils/Timer.h"
#include "../navigation/navigation.h"

static volatile bool dataPresent = false;
static volatile char data = 0;

static void UART1_Handler()
{
    dataPresent = true;
    data = UART1_DR_R; // Clears interrupt flag
    if(data == 'y')
    {
        nav_emergencyStop();
        dataPresent = false;
    }
}

void uart_init(void)
{
    static int initialized = 0;
    if (initialized)
    {
        return;
    }

    // SO MANY MAGIC NUMBERS AHHHHHHHH
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
    timer_waitMillis(1); // Small delay before accessing device after turning on clock
    GPIO_PORTB_AFSEL_R |= 0x03;
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & ~0xFF) | (1u << 4) | (1u << 0); // UART is function 1
    GPIO_PORTB_DEN_R |= 0x03;
    GPIO_PORTB_DIR_R &= ~(0x02); // Force 0's in the desired locations
    GPIO_PORTB_DIR_R |= 0x01;    // Force 1's in the desired locations

    // PB0 = UART1Rx, PB1 = UART1Tx
    // 115200 baud, 8 data bits, 1 stop bit, no parity
    // Sysclk = 16MHz, using /16 divisor in UARTCTL
    // BRD = BRDInt + BRDFrac = 16000000/(16 + 115200) = 8.6805555555
    // BRDInt = 8
    // BRDFrac = 0.68055555555
    // UARTFBRD = floor(0.680555555 * 64 + 0.5) = 44
    SYSCTL_RCGCUART_R |= (1u << 1);
    timer_waitMillis(1);
    UART1_IBRD_R = 8u; // See above
    UART1_FBRD_R = 44;
    UART1_LCRH_R = 0b01100000; // disable FIFOs, 8 bits data, 1 stop bit, no parity
    UART1_CC_R = 0;            // Sysclk clock source

    UART1_ICR_R = UART1_ICR_R; // Clear all interrupts
    UART1_IFLS_R = 0;          // Send Rx interrupts when Rx has anything in it
    UART1_IM_R = 0b10000;      // Enable Rx interrupt
    NVIC_EN0_R |= (1u << 6);

    // Bind the interrupt to the handler.
    IntRegister(INT_UART1, UART1_Handler);

    UART1_CTL_R = 0b1100000001; // Enable send, receive, and the UART

    initialized = 1;
}

void uart_sendChar(char data)
{
    while (UART1_FR_R & 0b100000)
        ; // Wait until TX holding buffer is empty
    UART1_DR_R = data;
}

char uart_receive(void)
{
    dataPresent = false;
    return data;
}

bool uart_dataPresent()
{
    return dataPresent;
}

void uart_sendStr(const char *data)
{
    int i = 0;
    while (data[i] != '\0')
    {
        uart_sendChar(data[i++]);
    }
}

void uart_sendLine(const char *data)
{
    int i = 0;
    while (data[i] != '\0')
    {
        uart_sendChar(data[i++]);
    }
    uart_sendChar('\r');
    uart_sendChar('\n');
}
