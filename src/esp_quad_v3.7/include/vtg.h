#ifndef _vtg_h_
#define _vtg_h_

#include "Arduino.h"


//voltage
#define v4sPin 4
#define v3sPin 4
#define v2sPin 15
#define v1sPin 14
// 123
#define v1s_lr 4.70f
#define v1s_hr 9.40f
#define v2s_lr 4.70f
#define v2s_hr 14.7f
#define v3s_lr 2.20f
#define v3s_hr 9.00f
#define v4s_lr 0.0f
#define v4s_hr 0.0f


#define v_pre_weight  0.99f
#define v_cur_weight  0.01f


class VTG{
    private:
        int hi = 1;
    public:
        float v1s = 0;
        float v2s = 0;
        float v3s = 0;
        float v4s = 0;
        void begin();
        void update();

};


void VTG::begin(){
    pinMode(v1sPin, INPUT);
    pinMode(v2sPin, INPUT);
    pinMode(v3sPin, INPUT);
    pinMode(v4sPin, INPUT);
}


// get voltages
void VTG::update() {
    float v1ss = analogRead(v1sPin);
    float v2ss = analogRead(v2sPin);
    float v3ss = analogRead(v3sPin);
    float v4ss = analogRead(v4sPin);
    v1ss = v1ss / 4095.0f * 3.3f;
    v2ss = v2ss / 4095.0f * 3.3f;
    v3ss = v3ss / 4095.0f * 3.3f;
    v4ss = v4ss / 4095.0f * 3.3f;

    v1ss = v1ss + v1ss * v1s_hr / v1s_lr;
    v2ss = v2ss + v2ss * v2s_hr / v2s_lr; 
    v3ss = v3ss + v3ss * v3s_hr / v3s_lr;
    v4ss = v4ss + v4ss * v4s_hr / v4s_lr;
  
    v1s = v1ss;
    v2s = v2ss;
    v3s = v3ss;
    v4s = v4ss;
    /*
    v1s = v1s * v_pre_weight + v1ss * v_cur_weight;
    v2s = v2s * v_pre_weight + v2ss * v_cur_weight;
    v3s = v3s * v_pre_weight + v3ss * v_cur_weight;
    v4s = v4s * v_pre_weight + v4ss * v_cur_weight;
    */
}



#endif
