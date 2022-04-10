#ifndef _nrf_h_
#define _nrf_h_

#include "Arduino.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


#define csn 25
#define ce 26
RF24 radio(26, 25); // CE, CSN


struct nrf {
    byte pwm[4];
    float pid[3];
    float pidz[3];
    byte en; 
};
//send
struct drone_data {
    float voltage[3];
    byte motorOut[4];
    float cur_angle[3];
};


class NRF{
    private:
        const byte defautT = 0;   
        const byte defautX = 127;
        const byte defautY = 127;
        const byte defautZ = 127;
        const byte addresses[2][6] = {"00001", "00002"};     
        
    public:
        struct nrf radioo;
        struct drone_data ack;
        void begin();
        void update();
        void reset_pwm();
        void set_ack_pwm(int, int, int, int);
        void set_ack_vgt(float, float, float);
        void set_ack_agl(float, float, float);

        bool available();
        void read();
        bool writeAckPayload();
};


bool NRF::available(){
    return radio.available();
}

void NRF::read(){
    radio.read(&radioo, sizeof(radioo));
}

bool NRF::writeAckPayload(){
    return radio.writeAckPayload(1, &ack, sizeof(ack));
}


void NRF::begin(){
    //initiate radio
    radio.begin();
    radio.enableDynamicPayloads();
    radio.enableAckPayload();
    radio.openReadingPipe(1, addresses[1]); // address global
    radio.openWritingPipe(addresses[0]); // 00001
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening();
    radio.writeAckPayload(1, &ack, sizeof(ack));   
}


void NRF::reset_pwm(){
    radioo.pwm[0] = defautX;
    radioo.pwm[1] = defautY;
    radioo.pwm[2] = defautZ;
    radioo.pwm[3] = defautT;
    radioo.en = 0;
}

void NRF::set_ack_pwm(int p1, int p2, int p3, int p4){
    // update ack 
    p1 = constrain(p1, 1000, 2000);
    p2 = constrain(p2, 1000, 2000);
    p3 = constrain(p3, 1000, 2000);
    p4 = constrain(p4, 1000, 2000);
    ack.motorOut[0] = map(p1, 1000, 2000, 0, 100);
    ack.motorOut[1] = map(p2, 1000, 2000, 0, 100);
    ack.motorOut[2] = map(p3, 1000, 2000, 0, 100);
    ack.motorOut[3] = map(p4, 1000, 2000, 0, 100);
}


void NRF::set_ack_vgt(float v1s, float v2s, float v3s){
    ack.voltage[0] = v1s;
    ack.voltage[1] = v2s;
    ack.voltage[2] = v3s;
}


void NRF::set_ack_agl(float x, float y, float z){
    ack.cur_angle[0] = x;
    ack.cur_angle[1] = y;
    ack.cur_angle[2] = z;
}


#endif