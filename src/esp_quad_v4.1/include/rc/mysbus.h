#ifndef _mysbus_h_
#define _mysbus_h_

/*
<SBUS.h>
-author
Brian R Taylor
brian.taylor@bolderflight.com
-forke
TheDIYGuy999
https://github.com/TheDIYGuy999/SBUS
*/

#ifdef sbus

#include "config.h"
#include <SBUS.h>

SBUS x8r(Serial2);

class MYSBUS{
    private:
        bool failSafe;
        bool lostFrame;
        int pwmChannel[16];
        uint16_t sbusChannel[16];
    public:
        bool read();
        void begin();
        bool getFailSafe();
        void resetChannel();
        int getPwmChannel(int);
        uint16_t getSbusChannel(int);
};


void MYSBUS::begin(){
    x8r.begin(16, 17, true, 100000);
}

bool MYSBUS::read(){
    bool statu = x8r.read(&sbusChannel[0], &failSafe, &lostFrame);
    for(int i=0; i<16; i++) pwmChannel[i] = map(sbusChannel[i], sbus_minVal, sbus_maxVal, motor_minPwm, motor_maxPwm);
    return statu;
}

uint16_t MYSBUS::getSbusChannel(int channel){
    return sbusChannel[channel];
}

int MYSBUS::getPwmChannel(int channel){
    return pwmChannel[channel];
}

void MYSBUS::resetChannel(){
    for(int i=0; i<16; i++) sbusChannel[i] = sbus_midVal;
    sbusChannel[sbus_channelT] = sbus_minVal;
    for(int i=0; i<16; i++) pwmChannel[i] = motor_midPwm;
    sbusChannel[sbus_channelT] = motor_minPwm;
}

bool MYSBUS::getFailSafe(){
    return failSafe;
}



#endif




#endif