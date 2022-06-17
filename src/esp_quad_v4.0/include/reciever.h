#ifndef _reciever_h_
#define _reciever_h_

#include "config.h"


#ifndef sbus
    #include "nrf.h"

    class RECIEVER{
        private:
            NRF r;
            bool enable = false;
            bool extra = false;
            int joyStick[4] = {recie_midJoyAnalog, recie_midJoyAnalog, recie_midJoyAnalog, recie_minJoyAnalog};
            unsigned long lastRecieveTime;
        public:
            void update();
            bool read();
            void begin();
            bool isTimeOut();
            bool getEnable();
            bool getExtra();
            void resetJoyStick();
            int getJoyStick(XYZT);
            void updatePayLoad(float, float, float, float, float, float, float, float, float, int, int, int, int);
    };

    void RECIEVER::begin(){
        r.begin();
    }

    void RECIEVER::update(){
        read(); // read
        if(isTimeOut()) resetJoyStick();  // 检查有没有超时
        // 更新回传数据
        r.setAck_vgt(voltage.getVoltage(V3));
        r.setAck_agl(imu.getAngle(X), imu.getAngle(Y), imu.getAngle(Z));
        r.setAck_gyr(imu.getGyro(X) , imu.getGyro(Y), imu.getGyro(Z));
        r.setAck_pwm(motorA.getStandardisedPwm(), motorB.getStandardisedPwm(), motorC.getStandardisedPwm(), motorD.getStandardisedPwm());
    }

    bool RECIEVER::read(){
        if(r.available()){
            r.read();
            r.writeAckPayload();
            enable = r.getRadioo_en();
            extra = r.getRadioo_extra();
            joyStick[X] = r.getRadioo_joyStick(X);
            joyStick[Y] = r.getRadioo_joyStick(Y);
            joyStick[Z] = r.getRadioo_joyStick(Z);
            joyStick[T] = r.getRadioo_joyStick(T);
            lastRecieveTime = millis();
            return true;
        }else return false;
    }

    bool RECIEVER::isTimeOut(){
        if(millis() - lastRecieveTime > recie_timeOut) return true;
        else return false;
    }

    bool RECIEVER::getEnable(){
        static bool lowered = false;
        if(enable == true and joyStick[T] < recie_minJoyAnalog+10) lowered = true;
        if(enable == false) lowered = false;
        if(lowered and enable) return true;
        else return false;
    }

    bool RECIEVER::getExtra(){
        return extra;
    }

    void RECIEVER::updatePayLoad(float v1, float v2, float v3 , float x, float y, float z, 
                                 float gx, float gy, float gz, int p1, int p2, int p3, int p4){
        r.setAck_agl(x, y, z);
        r.setAck_vgt(v3);
        r.setAck_pwm(p1, p2, p3, p4);
        r.setAck_gyr(gx, gy, gz);
    }

    void RECIEVER::resetJoyStick(){
        enable = false;
        extra = false;
        nrf.reset_pwm(); 
        for(int i=0; i<3; i++) joyStick[i] = recie_midJoyAnalog;
        joyStick[3] = recie_minJoyAnalog;
    }

    int RECIEVER::getJoyStick(XYZT which){
        return joyStick[which];
    }

#else
    #include "mysbus.h"

    class RECIEVER{
        private:
            MYSBUS r;
            bool enable;
            bool extra;
            int joyStick[4] = {recie_midJoyAnalog, recie_midJoyAnalog, recie_midJoyAnalog, recie_minJoyAnalog};
            unsigned long lastRecieveTime;
        public:
            void update();
            bool read();
            void begin();
            bool isTimeOut();
            bool getEnable();
            bool getExtra();
            void resetJoyStick();
            int getJoyStick(XYZT);
    };

    void RECIEVER::begin(){ 
        r.begin();
    }

    void RECIEVER::update(){
        read(); // read
        if(isTimeOut()) resetJoyStick();  // 检查有没有超时

    }

    bool RECIEVER::read(){
        if(r.read()){
            enable = (r.getPwmChannel(sbus_channelEnable) - motor_minPwm);
            extra = (r.getPwmChannel(sbus_channelExtra) - motor_minPwm);
            joyStick[0] = map(r.getSbusChannel(sbus_channelX), sbus_minVal, sbus_maxVal, recie_minJoyAnalog, recie_maxJoyAnalog);
            joyStick[1] = map(r.getSbusChannel(sbus_channelY), sbus_minVal, sbus_maxVal, recie_minJoyAnalog, recie_maxJoyAnalog);
            joyStick[2] = map(r.getSbusChannel(sbus_channelZ), sbus_minVal, sbus_maxVal, recie_minJoyAnalog, recie_maxJoyAnalog);
            joyStick[3] = map(r.getSbusChannel(sbus_channelT), sbus_minVal, sbus_maxVal, recie_minJoyAnalog, recie_maxJoyAnalog);
            lastRecieveTime = millis();
            return true;
        }else return false;
    }

    bool RECIEVER::isTimeOut(){
        if(millis() - lastRecieveTime > recie_timeOut or r.getFailSafe()) return true;
        else return false;
    }

    bool RECIEVER::getEnable(){
        static bool lowered = false;
        if(enable == true and joyStick[T] < recie_minJoyAnalog+10) lowered = true;
        if(enable == false) lowered = false;
        if(lowered and enable) return true;
        else return false;
    }

    bool RECIEVER::getExtra(){
        return extra;
    }

    void RECIEVER::resetJoyStick(){
        enable = false;
        extra = false;
        r.resetChannel();
        for(int i=0; i<3; i++) joyStick[i] = recie_midJoyAnalog;
        joyStick[3] = recie_minJoyAnalog;
    }

    int RECIEVER::getJoyStick(XYZT which){
        return joyStick[which];
    }

#endif


RECIEVER reciever;

#endif