/*
* midi2cv.c
*
* Created: 12/07/2016 11:07:49 PM
*  Author: Samuel
*/

#include <inttypes.h>

#define F_CPU 16000000U

#define RECEIVE_BUFFER_SIZE 16

#define GATE_PORT PORTC
#define GATE_DDR DDRC
#define GATE_PIN PINC3

#define TRIGGER_PORT PORTC
#define TRIGGER_DDR DDRC
#define TRIGGER_PIN PINC2

#define DAC_ADDR 0b00011110

#define PITCH_BEND_MID 0x4000

#define BAUD_RATE 31250
#define TRIGGER_TIME 5

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
	uint16_t delay = 0;
	if(pin == 1) {
		PORTD |= _BV(PIND2);
		_delay_ms(delay);
		PORTD &= ~_BV(PIND2);
		_delay_ms(delay);
		} else {
		PORTD |= _BV(PIND3);
		_delay_ms(delay);
		PORTD &= ~_BV(PIND3);
		_delay_ms(delay);
	}
}

int main(void) {
	uint8_t currentInputByte;
	//uint8_t channel;
	
	uint8_t currentNote = 0;
	uint8_t currentVelocity = 0;
	uint16_t pitchBendData = 0x0040;
	int8_t pitchBendAmount = 0; // +- 16 steps
	
	enum boolean firstDataByte = TRUE;
	enum status currentStatus = NONE;
	
	DDRD |= _BV(PIND2);
	PORTD &= ~_BV(PIND2);
	
	uartInit();
	gateInit();
	triggerInit();
	triggerTimerInit();
	i2c_init();
	dacZero();
	
	ledBlink(1);
	ledBlink(2);
	gateOn();
	_delay_ms(500);
	gateOff();
	triggerOn();

	sei();
	
	while(1) {
		while(bufferItemCount == 0) {
			// wait for another byte to be received
		}
		
		currentInputByte = receivedBuffer[readPointer];
		readPointer++;
		if(readPointer > (RECEIVE_BUFFER_SIZE - 1))		 {
			readPointer = 0;
		}
		
		if(bufferItemCount > 0) {
			bufferItemCount--;
		}
		
		// decode received byte
		// Status or Data byte
		if(currentInputByte & 0b10000000) { // Status
			ledBlink(1);
			//channel = b & 0x0F;
			
			switch(currentInputByte & 0xF0) {
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
				//currentStatus = NONE;
				break;
			}
			
		} else if((currentInputByte & 0x80) == 0) { // Data
			ledBlink(2);
			// do something with the data, determined by what the current status is
			switch(currentStatus) {
				case NOTEOFF:
				if(firstDataByte == TRUE) {
					gateOff();
					currentNote = currentInputByte;
					firstDataByte = FALSE;
				} else {
					firstDataByte = TRUE;
				}
				break;
				
				case NOTEON:
				if(firstDataByte == TRUE) {
					currentNote = currentInputByte;
					firstDataByte = FALSE;
				} else {
					currentVelocity = currentInputByte;
					if(currentVelocity == 0x00) { // Note Off
						gateOff();
					} else {
						triggerOn();
						gateOn();
					}
					firstDataByte = TRUE;
				}
				outputNote(currentNote, 0);
				break;
				
				case PITCH:
				if(firstDataByte == TRUE) {
					pitchBendData = currentInputByte;
					firstDataByte = FALSE;
				} else {
					pitchBendData |= (currentInputByte << 8);
					
					if(pitchBendData == PITCH_BEND_MID) { // no bend
						pitchBendAmount = 0;
					} else {
						pitchBendAmount = (pitchBendData - PITCH_BEND_MID) / 512;
					}
					firstDataByte = TRUE;
				}
				outputNote(currentNote, pitchBendAmount);
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

void triggerOn(void) {
	// start timer
	TCCR0B |= _BV(CS02) | _BV(CS00); // 1024 prescaler
	TRIGGER_PORT |= _BV(TRIGGER_PIN); // turn on trigger
}

void triggerOff(void) {
	TRIGGER_PORT &= ~_BV(TRIGGER_PIN); // turn off trigger
	TCCR0B &= ~(_BV(CS02) | _BV(CS01) | _BV(CS00)); // disable timer
}

/* 'Trigger On' timer interrupt */
ISR(TIMER0_COMPA_vect) {
	TCCR0B &= ~(_BV(CS02) | _BV(CS01) | _BV(CS00)); // disable timer
	TCNT0 = 0; // reset counter
	TRIGGER_PORT &= ~_BV(TRIGGER_PIN); // turn off trigger
}

/* USART Data Receive Interrupt */
ISR(USART_RX_vect) {
	receivedByte = UDR0;
	receivedBuffer[writePointer] = receivedByte;
	writePointer++;
	if(writePointer > (RECEIVE_BUFFER_SIZE - 1)) {
		writePointer = 0;
	}
	
	bufferItemCount++;
	if(bufferItemCount >= RECEIVE_BUFFER_SIZE) {
		bufferItemCount = RECEIVE_BUFFER_SIZE - 1;
	}
}

/* USART Initialisation */
void uartInit(void) {
	UCSR0B |= _BV(RXEN0) | _BV(RXCIE0); // enable receiver and interrupt
	UCSR0C |= _BV(UCSZ01) | _BV(UCSZ00); // 8 bit character size
	//uint32_t baud = F_CPU / (16U*BAUD_RATE) - 1;
	uint8_t baud = 31;
	UBRR0H = (uint8_t)(baud & 0xFF00);
	UBRR0L = (uint8_t)(baud);
	return;
}

/* Send the current note value to the ADC */
void outputNote(uint8_t note, int8_t pitchBend) {
	uint16_t dacData = (note * 32) + pitchBend; // DAC has 4096 steps for 128 notes
	i2c_start_wait(DAC_ADDR + I2C_WRITE);
	i2c_write((uint8_t)((dacData & 0x0F00) >> 8));
	i2c_write((uint8_t)(dacData & 0xFF));
	i2c_stop();
	return;
}

void dacZero(void) {
	i2c_start_wait(DAC_ADDR + I2C_WRITE);
	i2c_write(0x00);
	i2c_write(0x00);
	i2c_stop();
}
