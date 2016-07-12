/*
 * midi2cv.c
 *
 * Created: 12/07/2016 11:07:49 PM
 *  Author: Samuel
 */ 

#include <inttypes.h>

#define F_CPU 20000000

#define BAUD_RATE 31250

#include "midi2cv.h"
#include <avr/io.h>

volatile uint8_t receivedByte;

int main(void)
{
	uartInit();
	
	
    while(1)
    {
         
    }
}

/* USART Data Receive Interrupt */
ISR(USART_RX_vect) {
	receivedByte = UDR;
}

/* USART Initialisation */
void uartInit(void) {
	UCSRB |= _BV(RXEN) | (RXCIE); // enable receiver and interrupt
	UCSRC |= _BV(UCSZ1) | _BV(UCSZ0); // 8 bit character size
	uint32_t baud = F_CPU / (16*BAUD_RATE) - 1;
	UBRRH = baud & 0xFF00;
	UBRRL = baud;
}