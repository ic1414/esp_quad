#ifndef _vtg_h_
#define _vtg_h_

#include "Arduino.h"


//voltage
#define v3sPin 4
#define v2sPin 15
#define v1sPin 14
#define v_pre_weight  0.99f
#define v_cur_weight  0.01f


class VTG{
    private:
        float v1ss;
        float v2ss;
        float v3ss;

    public:
        float v1s = 0;
        float v2s = 0;
        float v3s = 0;
        void begin();
        void update();

};


void VTG::begin(){
    pinMode(v1sPin, INPUT);
    pinMode(v2sPin, INPUT);
    pinMode(v3sPin, INPUT);
}


// get voltages
void VTG::update() {
    v1ss = analogRead(v1sPin);
    v2ss = analogRead(v2sPin);
    v3ss = analogRead(v3sPin);
    v1ss = v1ss / 4095.0f * 3.3f;
    v2ss = v2ss / 4095.0f * 3.3f;
    v3ss = v3ss / 4095.0f * 3.3f;

    v1ss = v1ss * 9.40f / 4.7f + 0.2f;
    v2ss = v2ss * 14.7f / 4.7f + 0.4f; //systematic error ~0.70v
    v3ss = v3ss * 9.00f / 2.2f + 0.6f; // systematic error ~0.85v
  
    v1s = v1s * v_pre_weight + v1ss * v_cur_weight;
    v2s = v2s * v_pre_weight + v1ss * v_cur_weight;
    v3s = v3s * v_pre_weight + v1ss * v_cur_weight;

}



#endif