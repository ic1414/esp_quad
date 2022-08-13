#ifndef _nrf_h_
#define _nrf_h_

#include "config.h"

/* <nRF24L01.h> + <RF24.h>
https://github.com/nRF24/RF24 */

#ifdef nrff

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "config.h"


RF24 radio(nrf_cePin, nrf_csnPin); // CE, CSN

//接收
struct nrf {
    bool en; //
    bool extra;
    byte joyStick[4]; // x, y, z, t  
    uint8_t synchronizeByte = 0;
};
//回传给遥控
struct drone_data {
    float voltage; // 当前电压 1s, 2s, 3s
    byte motorOut[4]; // a, b, c, d
    int cur_gyro[3];  // 当前角速度 x, y, z
    float cur_angle[3]; // 当前角度 x, y, z
};

class NRF{
    private:
        const byte defautT = 0;   
        const byte defautX = 127;
        const byte defautY = 127;
        const byte defautZ = 127;
        const byte addresses[2][6] = {nrf_sendAddress, nrf_recieveAddress}; 
        struct nrf radioo;
        struct drone_data ack;    
        
    public:
        void begin();
        void reset_pwm();
        void setAck_pwm(int, int, int, int);
        void setAck_vgt(float);
        void setAck_gyr(int, int, int);
        void setAck_agl(float, float, float);
        void read();
        bool available();
        bool writeAckPayload();
        int getRadioo_joyStick(XYZT);
        bool getRadioo_en();
        bool getRadioo_extra();
        uint8_t getSyncByte();
};

void NRF::begin(){
    //initiate radio
    radio.begin();
    radio.enableDynamicPayloads();
    radio.enableAckPayload();
    radio.openReadingPipe(1, addresses[1]); 
    radio.openWritingPipe(addresses[0]);
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening();
    radio.writeAckPayload(1, &ack, sizeof(ack));   
}

bool NRF::available(){
    return radio.available();
}

void NRF::read(){
    radio.read(&radioo, sizeof(radioo));    
}

bool NRF::writeAckPayload(){
    return radio.writeAckPayload(1, &ack, sizeof(ack));
}

void NRF::reset_pwm(){
    radioo.joyStick[X] = defautX;
    radioo.joyStick[Y] = defautY;
    radioo.joyStick[Z] = defautZ;
    radioo.joyStick[T] = defautT;
    radioo.en = false;
    radioo.extra = false;
}

void NRF::setAck_pwm(int p1, int p2, int p3, int p4){
    // update ack 
    ack.motorOut[mA] = map(p1, motor_minPwm, motor_maxPwm, 0, 100);
    ack.motorOut[mB] = map(p2, motor_minPwm, motor_maxPwm, 0, 100);
    ack.motorOut[mC] = map(p3, motor_minPwm, motor_maxPwm, 0, 100);
    ack.motorOut[mD] = map(p4, motor_minPwm, motor_maxPwm, 0, 100);
}

void NRF::setAck_agl(float x, float y, float z){
    ack.cur_angle[X] = x;
    ack.cur_angle[Y] = y;
    ack.cur_angle[Z] = z;
}

void NRF::setAck_gyr(int x, int y, int z){
    ack.cur_gyro[X] = x;
    ack.cur_gyro[Y] = y;
    ack.cur_gyro[Z] = z;
}

void NRF::setAck_vgt(float v){
    ack.voltage = v;
}

int NRF::getRadioo_joyStick(XYZT which){
    return radioo.joyStick[which];
}

bool NRF::getRadioo_en(){
    return radioo.en;
}

bool NRF::getRadioo_extra(){
    return radioo.extra;
}

uint8_t NRF::getSyncByte(){
    return radioo.synchronizeByte;
}


#endif










#endif