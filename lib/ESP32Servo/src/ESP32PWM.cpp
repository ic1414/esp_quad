/*
 * ESP32PWM.cpp
 *
 *  Created on: Sep 22, 2018
 *      Author: hephaestus
 */

#include <ESP32PWM.h>
#include "esp32-hal-ledc.h"

// initialize the class variable ServoCount
int ESP32PWM::PWMCount = -1;              // the total number of attached servos
bool  ESP32PWM::explicateAllocationMode=false;
ESP32PWM * ESP32PWM::ChannelUsed[NUM_PWM]; // used to track whether a channel is in service
long ESP32PWM::timerFreqSet[4] = { -1, -1, -1, -1 };
int ESP32PWM::timerCount[4] = { 0, 0, 0, 0 };
// The ChannelUsed array elements are 0 if never used, 1 if in use, and -1 if used and disposed
// (i.e., available for reuse)
/**
 * allocateTimer
 * @param a timer number 0-3 indicating which timer to allocate in this library
 * Switch to explicate allocation mode
 *
 */
void ESP32PWM::allocateTimer(int timerNumber){
	if(timerNumber<0 || timerNumber>3)
		return;
	if(ESP32PWM::explicateAllocationMode==false){
		ESP32PWM::explicateAllocationMode=true;
		for(int i=0;i<4;i++)
			ESP32PWM::timerCount[i]=4;// deallocate all timers to start mode
	}
	ESP32PWM::timerCount[timerNumber]=0;
}

ESP32PWM::ESP32PWM() {
	resolutionBits = 8;
	pwmChannel = -1;
	pin = -1;
	myFreq = -1;
	if (PWMCount == -1) {
		for (int i = 0; i < NUM_PWM; i++)
			ChannelUsed[i] = NULL; // load invalid data into the storage array of pin mapping
		PWMCount = PWM_BASE_INDEX; // 0th channel does not work with the PWM system
	}
}

ESP32PWM::~ESP32PWM() {
	if (attached()) {
		ledcDetachPin(pin);
	}
	deallocate();
}

double ESP32PWM::_ledcSetupTimerFreq(uint8_t chan, double freq,
		uint8_t bit_num) {
	return ledcSetup(chan, freq, bit_num);

}

int ESP32PWM::timerAndIndexToChannel(int timerNum, int index) {
	int localIndex = 0;
	for (int j = 0; j < NUM_PWM; j++) {
		if (((j / 2) % 4) == timerNum) {
			if (localIndex == index) {
				return j;
			}
			localIndex++;
		}
	}
	return -1;
}
int ESP32PWM::allocatenext(double freq) {
	long freqlocal = (long) freq;
	if (pwmChannel < 0) {
		for (int i = 0; i < 4; i++) {
			bool freqAllocated = ((timerFreqSet[i] == freqlocal)
					|| (timerFreqSet[i] == -1));
			if (freqAllocated && timerCount[i] < 4) {
				if (timerFreqSet[i] == -1) {
					//Serial.println("Starting timer "+String(i)+" at freq "+String(freq));
					timerFreqSet[i] = freqlocal;
				}
				//Serial.println("Free channel timer "+String(i)+" at freq "+String(freq)+" remaining "+String(4-timerCount[i]));

				timerNum = i;
				for (int index=0; index<4; ++index)
				{
					int myTimerNumber = timerAndIndexToChannel(timerNum,index);
					if ((myTimerNumber >= 0)  && (!ChannelUsed[myTimerNumber]))
					{
						pwmChannel = myTimerNumber;
// 						Serial.println(
// 							"PWM on ledc channel #" + String(pwmChannel)
// 									+ " using 'timer " + String(timerNum)
// 									+ "' to freq " + String(freq) + "Hz");
						ChannelUsed[pwmChannel] = this;
						timerCount[timerNum]++;
						PWMCount++;
						myFreq = freq;
						return pwmChannel;
					}
				}
			} else {
//				if(timerFreqSet[i]>0)
//					Serial.println("Timer freq mismatch target="+String(freq)+" on timer "+String(i)+" was "+String(timerFreqSet[i]));
//				else
//					Serial.println("Timer out of channels target="+String(freq)+" on timer "+String(i)+" was "+String(timerCount[i]));
			}
		}
	} else {
		return pwmChannel;
	}
	Serial.println(
			"ERROR All PWM timers allocated! Can't accomodate " + String(freq)
					+ "Hz\r\nHalting...");
	while (1)
		;
}
void ESP32PWM::deallocate() {
	if (pwmChannel < 0)
		return;
// 	Serial.println("PWM deallocating LEDc #" + String(pwmChannel));
	timerCount[getTimer()]--;
	if (timerCount[getTimer()] == 0) {
		timerFreqSet[getTimer()] = -1; // last pwn closed out
	}
	timerNum = -1;
	attachedState = false;
	ChannelUsed[pwmChannel] = NULL;
	pwmChannel = -1;
	PWMCount--;

}

int ESP32PWM::getChannel() {
	if (pwmChannel < 0) {
		Serial.println("FAIL! must setup() before using get channel!");
	}
	return pwmChannel;
}

double ESP32PWM::setup(double freq, uint8_t resolution_bits) {
	checkFrequencyForSideEffects(freq);

	resolutionBits = resolution_bits;
	if (attached()) {
		ledcDetachPin(pin);
		double val = ledcSetup(getChannel(), freq, resolution_bits);
		attachPin(pin);
		return val;
	}
	return ledcSetup(getChannel(), freq, resolution_bits);
}
float ESP32PWM::getDutyScaled() {
	return mapf((float) myDuty, 0, (float) ((1 << resolutionBits) - 1), 0.0,
			1.0);
}
void ESP32PWM::writeScaled(float duty) {
	write(mapf(duty, 0.0, 1.0, 0, (float) ((1 << resolutionBits) - 1)));
}
void ESP32PWM::write(uint32_t duty) {
	myDuty = duty;
	ledcWrite(getChannel(), duty);
}
void ESP32PWM::adjustFrequencyLocal(double freq, float dutyScaled) {
	timerFreqSet[getTimer()] = (long) freq;
	myFreq = freq;
	if (attached()) {
		ledcDetachPin(pin);
		// Remove the PWM during frequency adjust
		_ledcSetupTimerFreq(getChannel(), freq, resolutionBits);
		writeScaled(dutyScaled);
		ledcAttachPin(pin, getChannel()); // re-attach the pin after frequency adjust
	} else {
		_ledcSetupTimerFreq(getChannel(), freq, resolutionBits);
		writeScaled(dutyScaled);
	}
}
void ESP32PWM::adjustFrequency(double freq, float dutyScaled) {
	if(dutyScaled<0)
		dutyScaled=getDutyScaled();
	writeScaled(dutyScaled);
	for (int i = 0; i < timerCount[getTimer()]; i++) {
		int pwm = timerAndIndexToChannel(getTimer(), i);
		if (ChannelUsed[pwm] != NULL) {
			if (ChannelUsed[pwm]->myFreq != freq) {
				ChannelUsed[pwm]->adjustFrequencyLocal(freq,
						ChannelUsed[pwm]->getDutyScaled());
			}
		}
	}
}
double ESP32PWM::writeTone(double freq) {
	for (int i = 0; i < timerCount[getTimer()]; i++) {
		int pwm = timerAndIndexToChannel(getTimer(), i);
		if (ChannelUsed[pwm] != NULL) {
			if (ChannelUsed[pwm]->myFreq != freq) {
				ChannelUsed[pwm]->adjustFrequencyLocal(freq,
						ChannelUsed[pwm]->getDutyScaled());
			}
			write(1 << (resolutionBits-1)); // writeScaled(0.5);
		}
	}

	return 0;
}
double ESP32PWM::writeNote(note_t note, uint8_t octave) {
	const uint16_t noteFrequencyBase[12] = {
			//   C        C#       D        Eb       E        F       F#        G       G#        A       Bb        B
			4186, 4435, 4699, 4978, 5274, 5588, 5920, 6272, 6645, 7040, 7459,
			7902 };

	if (octave > 8 || note >= NOTE_MAX) {
		return 0;
	}
	double noteFreq = (double) noteFrequencyBase[note]
			/ (double) (1 << (8 - octave));
	return writeTone(noteFreq);
}
uint32_t ESP32PWM::read() {
	return ledcRead(getChannel());
}
double ESP32PWM::readFreq() {
	return myFreq;
}
void ESP32PWM::attach(int p) {
	pin = p;
	attachedState = true;
}
void ESP32PWM::attachPin(uint8_t pin) {

	if (hasPwm(pin)) {
		attach(pin);
		ledcAttachPin(pin, getChannel());
	} else {
		Serial.println(
				"ERROR PWM channel unavailible on pin requested! " + String(pin)
						+ "\r\nPWM availible on: 2,4,5,12-19,21-23,25-27,32-33");
		return;
	}
	//Serial.print(" on pin "+String(pin));
}
void ESP32PWM::attachPin(uint8_t pin, double freq, uint8_t resolution_bits) {

	if (hasPwm(pin))
		setup(freq, resolution_bits);
	attachPin(pin);
}
void ESP32PWM::detachPin(int pin) {
	ledcDetachPin(pin);
	deallocate();
}
/* Side effects of frequency changes happen because of shared timers
 *
 * LEDC Chan to Group/Channel/Timer Mapping
 ** ledc: 0  => Group: 0, Channel: 0, Timer: 0
 ** ledc: 1  => Group: 0, Channel: 1, Timer: 0
 ** ledc: 2  => Group: 0, Channel: 2, Timer: 1
 ** ledc: 3  => Group: 0, Channel: 3, Timer: 1
 ** ledc: 4  => Group: 0, Channel: 4, Timer: 2
 ** ledc: 5  => Group: 0, Channel: 5, Timer: 2
 ** ledc: 6  => Group: 0, Channel: 6, Timer: 3
 ** ledc: 7  => Group: 0, Channel: 7, Timer: 3
 ** ledc: 8  => Group: 1, Channel: 0, Timer: 0
 ** ledc: 9  => Group: 1, Channel: 1, Timer: 0
 ** ledc: 10 => Group: 1, Channel: 2, Timer: 1
 ** ledc: 11 => Group: 1, Channel: 3, Timer: 1
 ** ledc: 12 => Group: 1, Channel: 4, Timer: 2
 ** ledc: 13 => Group: 1, Channel: 5, Timer: 2
 ** ledc: 14 => Group: 1, Channel: 6, Timer: 3
 ** ledc: 15 => Group: 1, Channel: 7, Timer: 3
 */

bool ESP32PWM::checkFrequencyForSideEffects(double freq) {

	allocatenext(freq);
	for (int i = 0; i < timerCount[getTimer()]; i++) {
		int pwm = timerAndIndexToChannel(getTimer(), i);

		if (pwm == pwmChannel)
			continue;
		if (ChannelUsed[pwm] != NULL)
			if (ChannelUsed[pwm]->getTimer() == getTimer()) {
				double diff = abs(ChannelUsed[pwm]->myFreq - freq);
				if (abs(diff) > 0.1) {
					Serial.println(
							"\tWARNING PWM channel " + String(pwmChannel)
									+ " shares a timer with channel "
									+ String(pwm) + "\n"
											"\tchanging the frequency to "
									+ String(freq)
									+ " Hz will ALSO change channel "
									+ String(pwm)
									+ " \n\tfrom its previous frequency of "
									+ String(ChannelUsed[pwm]->myFreq) + " Hz\n"
											" ");
					ChannelUsed[pwm]->myFreq = freq;
				}
			}
	}
	return true;
}

ESP32PWM* pwmFactory(int pin) {
	for (int i = 0; i < NUM_PWM; i++)
		if (ESP32PWM::ChannelUsed[i] != NULL) {
			if (ESP32PWM::ChannelUsed[i]->getPin() == pin)
				return ESP32PWM::ChannelUsed[i];
		}
	return NULL;
}
