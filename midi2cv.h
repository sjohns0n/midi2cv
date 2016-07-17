/*
 * midi2cv.h
 *
 * Created: 12/07/2016 11:18:09 PM
 *  Author: Samuel
 */ 


#ifndef MIDI2CV_H_
#define MIDI2CV_H_

void uartInit(void);
void gateInit(void);
void gateOn(void);
void gateOff(void);
void triggerInit(void);
void triggerOn(void);
void triggerOff(void);
void triggerTimerInit(void);

void outputNote(uint8_t note, int8_t pitchBend);
void dacZero(void);

#endif /* MIDI2CV_H_ */