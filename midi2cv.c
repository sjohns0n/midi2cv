/*
* midi2cv.c
*
* Created: 12/07/2016 11:07:49 PM
*  Author: Samuel
*/

#include <inttypes.h>

#define F_CPU 20000000

#define GATE_PORT PORTD
#define GATE_PIN PIND1

#define BAUD_RATE 31250
#define TRIGGER_TIME 10

#include "midi2cv.h"
#include <avr/io.h>

volatile uint8_t receivedByte;

enum status {
	NONE,
	NOTEOFF,
	NOTEON,
	PRESSURE,
	CONTROL,
	PROGRAM,
	CHPRESSURE,
	PITCH,
	STATUSMAX
};

enum boolean {
	TRUE,
	FALSE
};

int main(void) {
	uint8_t b;
	uint8_t channel;
	uint8_t adcOutput = 0;
	
	uint8_t currentNote = 0;
	uint8_t currentVelocity = 0;
	
	enum boolean firstDataByte = TRUE;
	enum status currentStatus;
	
	uartInit();
	
	while(1) {
		b = receivedByte;
		// decode received byte
		// Status or Data byte
		if(b & 0x80) { // Status
			channel = b & 0x0F;
			
			switch(b & 0xF0) {
				case 0x80: // Note Off
				currentStatus = NOTEOFF;
				break;
				case 0x90: // Note On
				currentStatus = NOTEON;
				break;
				case 0xA0: // Polyphonic Key Pressure
				currentStatus = PRESSURE;
				break;
				case 0xB0: // Control Change
				currentStatus = CONTROL;
				break;
				case 0xC0: // Program Change
				currentStatus = PROGRAM;
				break;
				case 0xD0: // Channel Pressure / Aftertouch
				currentStatus = CHPRESSURE;
				break;
				case 0xE0: // Pitch Bend Change
				currentStatus = PITCH;
				break;
				
				default: // invalid Status byte
				currentStatus = NONE;
				break;
			}
			
		} else
		if((b & 0x80) == 0) { // Data
			// do something with the data, determined by what the current status is
			switch(currentStatus) {
				case NOTEOFF:
				if(firstDataByte == TRUE) {
					currentNote = b;
					firstDataByte = FALSE;
				} else {
					firstDataByte = TRUE;
				}
				break;
				
				case NOTEON:
				if(firstDataByte == TRUE) {
					currentNote = b;
					firstDataByte = FALSE;
				} else {
					if(b == 0x00) {
						gateOff();
					} else {
						currentVelocity = b;
						triggerOn(TRIGGER_TIME);
						gateOn();
					}
					firstDataByte = TRUE;
				}
				break;
				
				default:
				// do nothing
				break;
			}
			
		} else { // invalid byte
			
		}
	}
}

void gateOn(void) {
	GATE_PORT |= _BV(GATE_PIN);
}

void gateOff(void) {
	GATE_PORT &= ~_BV(GATE_PIN);
}

void triggerOn(uint16_t timeOn) {
	// turn trigger pin on for timeOn milliseconds
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