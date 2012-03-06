/* Rev 1.0 Wireless Triac by Vanya A. Sergeev - <vsergeev@gmail.com> */

#include "uart.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

void process_command_data(void);

#define LED_1			(1<<1)
#define LED_2			(1<<2)
#define TRIAC_TRIGGER		(1<<6)
#define ADC_ZERO_CROSS		(1<<0)

/* The time between ADC samples (determined by the ADC clock rate setting in
 * ADC_init() in microseconds */
#define ADC_SAMPLE_TIME		83

volatile uint8_t triac_enable;
volatile int16_t triac_delay;
volatile int16_t triac_wait_us;

/* System millisecond timer built from the ADC interrupt handler */
volatile uint16_t msHigh = 0;
volatile uint16_t msLow = 0;
volatile uint16_t usCount = 0;

ISR(ADC_vect) {
	uint16_t adc_data;

	/* Sample the zero cross detect */
	adc_data = ADCL;
	adc_data |= (ADCH << 8);


	/* Count elapsed [83] microseconds */
	usCount++;

	/* Once 1000*83 microseconds have passed, 83 milliseconds have
	 * passed. */
	if (usCount == 1000) {
		/* Check for overflow from 1024 milliseconds, if so, increment
		 * msHigh */
		if ((msLow + ADC_SAMPLE_TIME) < msLow)
			msHigh++;
		/* Increment our low milliseconds */
		msLow += ADC_SAMPLE_TIME;
		usCount = 0;
	}

	if (triac_enable) {
		/* If we hit a zero crossing, reset the triac wait time */
		if (adc_data > 100) {
			triac_wait_us = triac_delay;
		}

		/* Subtract out from out triac wait time */
		if (triac_wait_us > 0)
			triac_wait_us -= ADC_SAMPLE_TIME;

		/* If the wait time has elapsed, trigger the triac */
		if (triac_wait_us <= 0) {
			PORTD &= ~TRIAC_TRIGGER;
			_delay_us(3);
			PORTD |= TRIAC_TRIGGER;
		}
	}

	process_command_data();
}

void ADC_init(void) {
	/* Disable global interrupts temporarily */
	cli();

	/* Make our ADC channels inputs */
	DDRC &= ~(ADC_ZERO_CROSS);

	/* Configure ADC Multiplexer
	 * [7:6] REFS1:0 = 01 for AVcc voltage reference
	 * [3:0] MUX3:0 = 0000 for ADC0 */
	ADMUX = (1<<6);

	/* Configure Digital Input Disable Register 0
	 * [5:0] = ADC5D - ADC0D = 11111 to disable digital input buffer
	 *         for ADC0-ADC5 to reduce power consumption. */
	DIDR0 = 0x1F;

	/* Configure ADC Control and Status Register B
	 * [2:0] ADTS2:0 = 100 for Timer/Counter0 Overflow
	 * [2:0] ADTS2:0 = 000 for Free Running mode */
	ADCSRB = 0;

	/* Configure ADC Control and Status Register A
	 * [7] ADEN = 1 to enable ADC
	 * [6] ADSC = 1 to start the conversion
	 * [5] ADATE = 1 to enable trigger of ADC
	 * [3] ADIE = 1 to enable ADC interrupt
	 * [2:0] ADPS2:0 = 111 for 128 prescaling = 156.25kHz */
	ADCSRA = ((1<<7) | (1<<6) | (1<<5) | (1<<3) | (1<<2) | (1<<1) | (1<<0));

	/* Enable global interrupts */
	sei();
}

void delay_us(uint16_t duration) {
	for (; duration > 0; duration--)
		_delay_us(1);
}


void delay_ms(uint16_t duration) {
	for (; duration > 0; duration--)
		_delay_ms(1);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#define MAX_POWER		0
#define MIN_POWER		7100

#define HEARTBEAT_DIM_TIME	625
#define HEARTBEAT_HOLD_TIME	250
#define HEARTBEAT_SINGLE_TIME	(2*HEARTBEAT_DIM_TIME+HEARTBEAT_HOLD_TIME)

#define DIM_MAX_POWER		500
#define DIM_MIN_POWER		6800

void heartbeat(uint8_t bpm);
void dim_up(uint16_t ms, int8_t reset);
void dim_down(uint16_t ms, int8_t reset);
void full_on(void);
void full_off(void);
uint8_t ascii2hex(uint8_t c);

void heartbeat(uint8_t bpm) {
	uint16_t i;
	uint16_t dim_up_time, dim_down_time, hold_time, deadtime;
	uint16_t cycle_time;

	cycle_time = (60000/bpm);

	deadtime = 0;
	dim_up_time = cycle_time/2;
	dim_down_time = cycle_time/2;
	hold_time = 0;

	triac_delay = DIM_MIN_POWER;
	for (i = 0; i < 10; i++) {
		dim_up(dim_up_time, 0);
		delay_ms(hold_time);
		dim_down(dim_down_time, 0);
		delay_ms(deadtime);
	}
}

#define STATE_WAIT		0
#define STATE_FIRST_HEX		1
#define STATE_SECOND_HEX	2
#define STATE_END		3

uint8_t data_hex_1, data_hex_2;
uint8_t state = STATE_WAIT;
uint8_t bpm = 16;

uint8_t ascii2hex(uint8_t c) {
	if (c >= 'A')
		c -= 7;
	c -= '0';
	return c;
}

void process_command_data(void) {
	uint8_t c;

	if (!UART_data_available())
		return;

	c = UART_getc();

	switch (state) {
		case STATE_WAIT:
			if (c == 'S') {
				state = STATE_FIRST_HEX;
				PORTB &= ~LED_2;
			}
			break;
		case STATE_FIRST_HEX:
			if (c < '0' || c > 'F' || (c > '9' && c < 'A')) {
				state = STATE_WAIT;
				break;
			}
			data_hex_1 = c;
			state = STATE_SECOND_HEX;
			break;
		case STATE_SECOND_HEX:
			if (c < '0' || c > 'F' || (c > '9' && c < 'A')) {
				state = STATE_WAIT;
				break;
			}
			data_hex_2 = c;
			state = STATE_END;
			break;
		case STATE_END:
			PORTB |= LED_2;
			if (c != 'X') {
				state = STATE_WAIT;
				break;
			}
			bpm = ascii2hex(data_hex_1) << 4;
			bpm += ascii2hex(data_hex_2);
			state = STATE_WAIT;
			break;
	}
}

void heartbeat_loop(void) {
	uint16_t dim_up_time, dim_down_time, hold_time, deadtime;
	uint16_t cycle_time;

	while (1) {
		if (bpm == 0) {
			full_off();
		} else if (bpm == 255) {
			full_on();
		} else {
			cycle_time = (60000/bpm);

			deadtime = 0;
			dim_up_time = cycle_time/2;
			dim_down_time = cycle_time/2;
			hold_time = 0;

			triac_delay = DIM_MIN_POWER;
			dim_up(dim_up_time, 0);
			delay_ms(hold_time);
			dim_down(dim_down_time, 0);
			delay_ms(deadtime);
		}
	}
}

void full_on(void) {
	triac_delay = MAX_POWER;
}

void full_off(void) {
	triac_delay = MIN_POWER;
}

void dim_up(uint16_t ms, int8_t reset) {
	uint16_t steps;

	if (reset)
		steps = (DIM_MIN_POWER - DIM_MAX_POWER)/ms;
	else
		steps = (triac_delay - DIM_MAX_POWER)/ms;
	if (steps == 0)
		steps = 1;

	if (reset)
		triac_delay = DIM_MIN_POWER;
	for (; ms > 0; ms--) {
		triac_delay -= steps;
		_delay_ms(1);
	}
	//triac_delay = DIM_MAX_POWER;
}

void dim_down(uint16_t ms, int8_t reset) {
	uint16_t steps;

	if (reset)
		steps = (DIM_MIN_POWER - DIM_MAX_POWER)/ms;
	else
		steps = (DIM_MIN_POWER - triac_delay)/ms;
	if (steps == 0)
		steps = 1;

	if (reset)
		triac_delay = DIM_MAX_POWER;
	for (; ms > 0; ms--) {
		triac_delay += steps;
		_delay_ms(1);
	}
	//triac_delay = DIM_MIN_POWER;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main(void) {
#ifdef TEST
	uint8_t c;
#endif
	/* Some start up time for other parts to settle */
	delay_ms(200);

	/* Initialize UART */
	UART_init(UART_calcBaudRate(9600));

	/* Enable the LEDs as outputs */
	DDRB |= (LED_1 | LED_2);
	/* Light up LED1, turn off LED2, for now */
	PORTB &= ~LED_1;
	PORTB |= LED_2;

	/* Enable the triac pin as an output */
	DDRD |= TRIAC_TRIGGER;
	/* Disable the triac for now */
	PORTD |= TRIAC_TRIGGER;
	triac_enable = 0;

	/* Initialize the ADC */
	ADC_init();

#ifndef TEST
	triac_delay = MIN_POWER;
	triac_enable = 1;
	heartbeat_loop();
#else
	while (1) {
		c = UART_getc();
		if (c == 'q') {
			triac_enable = 1;
			full_on();
		} else if (c == 'd') {
			dim_up(3000, 1);
		} else if (c == 'c') {
			dim_down(3000, 1);
		} else if (c == 't') {
			triac_delay += 20;
		} else if (c == 'h') {
			triac_delay -= 20;
		} else if (c == 'j') {
			heartbeat(20);
		} else if (c == 'k') {
			heartbeat(30);
		} else if (c == 'l') {
			heartbeat(50);
		} else if (c == ';') {
			heartbeat(100);
		} else if (c == 'p') {
			full_off();
			triac_enable = 0;
			PORTD |= TRIAC_TRIGGER;
		}
	}
#endif
	return 0;
}

