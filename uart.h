/*
 * Polling UART implementation based on ATMega32 datasheet
 * Author: Vanya A. Sergeev - <vsergeev@gmail.rr.com>
 * Date: 02/15/05
 */

#include <avr/io.h>
#include <stdint.h>

#define UART_STAT   	UCSR0A
#define UART_CTRL	UCSR0B
#define UART_SETT	UCSR0C
#define UART_DATA     	UDR0
#define	UART_BAUDH	UBRR0H
#define	UART_BAUDL	UBRR0L

#define	UART_calcBaudRate(baudRate) ((uint8_t)((F_CPU/(16L * (unsigned long)baudRate)) - 1))

void UART_init(uint16_t baudRate);
uint8_t UART_getc(void);
uint8_t UART_getc_nonblock(void);
void UART_putc(uint8_t data);
void UART_puts(char *data);
void UART_flush(void);
uint8_t UART_data_available(void);
