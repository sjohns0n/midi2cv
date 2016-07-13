/*
* midi2cv.c
*
* Created: 12/07/2016 11:07:49 PM
*  Author: Samuel
*/

#include <inttypes.h>

#define F_CPU 16000000

#define GATE_PORT PORTD
#define GATE_PIN PIND1

#define ADC_ADDR 0b00011110

#define BAUD_RATE 31250
#define TRIGGER_TIME 10

#include "midi2cv.h"
#include "i2cmaster.h"
#include <util/delay.h>
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
	
	uint16_t testAdc = 0;
	
	uint8_t currentNote = 0;
	uint8_t currentVelocity = 0;
	
	enum boolean firstDataByte = TRUE;
	enum status currentStatus;
	
	uartInit();
	i2c_init();
	
	while(1) {
		i2c_start_wait(ADC_ADDR + I2C_WRITE);
		i2c_write((uint8_t)((testAdc & 0x0F00) >> 8));
		i2c_write(testAdc & 0xFF);
		//i2c_stop();
		
		testAdc += 100;
		
		_delay_ms(250);
		continue;
		
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
	receivedByte = UDR0;
}

/* USART Initialisation */
void uartInit(void) {
	UCSR0B |= _BV(RXEN0) | (RXCIE0); // enable receiver and interrupt
	UCSR0C |= _BV(UCSZ01) | _BV(UCSZ00); // 8 bit character size
	uint32_t baud = F_CPU / (16*BAUD_RATE) - 1;
	UBRR0H = baud & 0xFF00;
	UBRR0L = baud;
}