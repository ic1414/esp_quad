#ifndef _vtg_h_
#define _vtg_h_

#include "config.h"


class VTG{
    private:
        float arrayV[4];

    public:
        void begin();
        void update();
        float getVoltage(V1234);
};


void VTG::begin(){
    pinMode(v1sPin, INPUT);
    pinMode(v2sPin, INPUT);
    pinMode(v3sPin, INPUT);
    pinMode(v4sPin, INPUT);
}


// get voltages
void VTG::update() {
    float vRead[4];
    vRead[V1] = analogRead(v1sPin);
    vRead[V2] = analogRead(v2sPin);
    vRead[V3] = analogRead(v3sPin);
    vRead[V4] = analogRead(v4sPin);
    vRead[V1] = vRead[V1] / 4095.0f * 3.3f;
    vRead[V2] = vRead[V2] / 4095.0f * 3.3f;
    vRead[V3] = vRead[V3] / 4095.0f * 3.3f;
    vRead[V4] = vRead[V4] / 4095.0f * 3.3f;
    arrayV[V1] = vRead[V1] * (1.0f + v1s_hr / v1s_lr);
    arrayV[V2] = vRead[V2] * (1.0f + v2s_hr / v2s_lr);
    arrayV[V3] = vRead[V3] * (1.0f + v3s_hr / v3s_lr);
    arrayV[V4] = vRead[V4] * (1.0f + v4s_hr / v4s_lr);
}


float VTG::getVoltage(V1234 which){ 
    return arrayV[which];
}



VTG voltage;


#endif