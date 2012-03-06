/*
 * Polling UART implementation based on ATMega32 datasheet
 * Author: Vanya A. Sergeev - <vsergeev@gmail.com>
 * Date: 02/15/05
 */

#include "uart.h"

void UART_init(uint16_t baudRate) {
	/* Set higher byte of the computed baudrate */
	UART_BAUDH = (uint8_t)(baudRate >> 8);
	/* Set lower byte of the computed baudrate */
	UART_BAUDL = (uint8_t)baudRate;
	/* Enable UART Send and Receive */
	UART_CTRL = ((1<<4)|(1<<3));
	/* Set the frame format to 8 data bits, 1 stop bit, no parity */
	UART_SETT = (1<<2)|(1<<1);
}

uint8_t UART_getc(void) {
	/* Wait for receive buffer to clear */
	while ( !(UART_STAT & (1<<7)) )
		;
	/* Read and return the data */
	return UART_DATA;
}

uint8_t UART_data_available(void) {
	if (UART_STAT & (1<<7))
		return 1;
	return 0;
}

uint8_t UART_getc_nonblock(void) {
	/* If no data is available, return 0 */
	if (!(UART_STAT & (1<<7)))
		return 0;
	/* Otherwise read and return the data */
	return UART_DATA;
}

void UART_putc(uint8_t data) {
	/* Wait for send buffer to clear */
	while ( !(UART_STAT & (1<<5)) )
		;
	/* Send data */
	UART_DATA = data;
}

void UART_puts(char *data) {
	/* Keep on sending data until we encounter a null byte */
	while (*data != '\0')
		UART_putc(*data++);
}

void UART_flush(void) {
	uint8_t temp;
	/* While data exists, read it from the UDR register */
	while (UART_STAT & (1<<7))
		temp = UART_DATA;
}

