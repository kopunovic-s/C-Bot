#ifndef UART_H_
#define UART_H_

#include <stdbool.h>

/**
 * @brief Initialize UART for sending to the Wifi module
 * (115200 baud, 8 data, 1 stop, no parity)
 *
 */
void uart_init(void);

/**
 * @brief Send a single char via UART
 *
 * @param data Char to send
 */
void uart_sendChar(char data);

/**
 * @brief Receive a char over uart if data is present.
 * If no data is present, will return the last char received.
 *
 * @return char
 */
char uart_receive(void);

/**
 * @brief Check if data is present in the UART receive buffer.
 *
 * @return True if data is present, false otherwise
 */
bool uart_dataPresent();

/**
 * @brief Send a null-terminated string over UART.
 *
 * @param data String to send
 */
void uart_sendStr(const char *data);

/**
 * @brief Send a null-terminated string over UART,
 * followed by \r\n.
 *
 * @param data String to send
 */
void uart_sendLine(const char *data);

#endif
