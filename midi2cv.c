/*
* midi2cv.c
*
* Created: 12/07/2016 11:07:49 PM
*  Author: Samuel
*/

#include <inttypes.h>

#define F_CPU 16000000U

#define RECEIVE_BUFFER_SIZE 16

#define GATE_PORT PORTD
#define GATE_DDR DDRD
#define GATE_PIN PIND2

#define TRIGGER_PORT PORTD
#define TRIGGER_DDR DDRD
#define TRIGGER_PIN PIND2

#define DAC_ADDR 0b00011110

#define BAUD_RATE 31250
#define TRIGGER_TIME 10

#include "midi2cv.h"
#include "i2cmaster.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint8_t receivedByte;
volatile uint8_t receivedBuffer[RECEIVE_BUFFER_SIZE];
volatile uint8_t writePointer = 0;
volatile uint8_t readPointer = 0;
volatile uint8_t bufferItemCount = 0;

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

void ledBlink(uint8_t pin) {
	if(pin == 1) {
		PORTD |= _BV(PIND2);
		_delay_ms(250);
		PORTD &= ~_BV(PIND2);
		_delay_ms(250);
		} else {
		PORTD |= _BV(PIND3);
		_delay_ms(250);
		PORTD &= ~_BV(PIND3);
		_delay_ms(250);
	}
}

int main(void) {
	uint8_t b;
	//uint8_t channel;
	
	uint8_t currentNote = 0;
	uint8_t currentVelocity = 0;
	
	enum boolean firstDataByte = TRUE;
	enum status currentStatus = NONE;
	
	DDRD |= _BV(PIND2);
	PORTD &= ~_BV(PIND2);
	
	uartInit();
	sei();
	ledBlink(1);
	ledBlink(2);
	
	gateInit();
	triggerInit();
	triggerTimerInit();
	i2c_init();

	while(1) {
		while(bufferItemCount == 0) {
			// wait for another byte to be received
		}
		
		b = receivedBuffer[readPointer];
		readPointer++;
		if(readPointer > (RECEIVE_BUFFER_SIZE - 1))		 {
			readPointer = 0;
		}
		
		if(bufferItemCount <= 0) {
			bufferItemCount = 0;	
		} else {
			bufferItemCount--;
		}
		
		// decode received byte
		// Status or Data byte
		if(b & 0b10000000) { // Status
			ledBlink(1);
			//channel = b & 0x0F;
			
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
			ledBlink(2);
			// do something with the data, determined by what the current status is
			switch(currentStatus) {
				case NOTEOFF:
				if(firstDataByte == TRUE) {
					//gateOff();
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
					currentVelocity = b;
					if(currentVelocity == 0x00) { // Note Off
						//gateOff();
					} else {
						//triggerOn();
						//gateOn();
					}
					firstDataByte = TRUE;
				}
				outputNote(currentNote);
				break;
				
				default:
				// do nothing
				break;
			}
			
		} else { // invalid byte
			
		}
	}
}

void gateInit(void) {
	GATE_DDR |= _BV(GATE_PIN);
	gateOff();
}

void triggerInit(void) {
	TRIGGER_DDR |= _BV(TRIGGER_PIN);
}

void gateOn(void) {
	GATE_PORT |= _BV(GATE_PIN);
}

void gateOff(void) {
	GATE_PORT &= ~_BV(GATE_PIN);
}

void triggerTimerInit(void) {
	uint8_t timerValue = (uint8_t)(TRIGGER_TIME * F_CPU / 1024);
	OCR0A = timerValue;
	
	TCCR0B &= ~(_BV(CS02) | _BV(CS01) | _BV(CS00)); // disable timer
	TIMSK0 |= _BV(OCIE0A); // enable interrupt
}

void triggerOn() {
	// start timer
	TCCR0B |= _BV(CS02) | _BV(CS00); // 1024 prescaler
	TRIGGER_PORT |= _BV(TRIGGER_PIN); // turn on trigger
}

/* 'Trigger On' timer interrupt */
ISR(TIMER0_COMPA_vect) {
	TCCR0B &= ~(_BV(CS02) | _BV(CS01) | _BV(CS00)); // disable timer
	TCNT0 = 0; // reset counter
	TRIGGER_PORT &= ~_BV(TRIGGER_PIN); // turn of trigger
}

/* USART Data Receive Interrupt */
ISR(USART_RX_vect) {
	receivedByte = UDR0;
	receivedBuffer[writePointer] = receivedByte;
	writePointer++;
	if(writePointer > (RECEIVE_BUFFER_SIZE - 1)) {
		writePointer = 0;
	}
	
	if(bufferItemCount >= RECEIVE_BUFFER_SIZE) {
		bufferItemCount = RECEIVE_BUFFER_SIZE - 1;
	} else {
		bufferItemCount++;
	}
}

/* USART Initialisation */
void uartInit(void) {
	UCSR0B |= _BV(RXEN0) | _BV(RXCIE0); // enable receiver and interrupt
	UCSR0C |= _BV(UCSZ01) | _BV(UCSZ00); // 8 bit character size
	//uint32_t baud = F_CPU / (16U*BAUD_RATE) - 1;
	uint8_t baud = 31;
	UBRR0H = (uint8_t)(baud & 0xFF00);
	UBRR0L = (uint8_t)baud;
	return;
}

/* Send the current note value to the ADC */
void outputNote(uint16_t note) {
	note = note * 32; // DAC has 4096 steps for 128 notes
	i2c_start_wait(DAC_ADDR + I2C_WRITE);
	i2c_write((uint8_t)((note & 0x0F00) >> 8));
	i2c_write(note & 0xFF);
	i2c_stop();
	return;
}