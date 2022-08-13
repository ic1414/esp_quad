#ifndef _reciever_h_
#define _reciever_h_

#include "config.h"


class RECIEVER_BASE{
    protected:
        bool extra;
        bool enable;
        unsigned long lastRecieveTime;
        int joyStick[4] = {recie_midJoyAnalog, recie_midJoyAnalog, recie_midJoyAnalog, recie_minJoyAnalog,};
    public:
        bool read(){return -1;};
        void begin();
        void update();
        bool isTimeOut(){return -1;};
        bool getExtra(){return extra;};
        void resetJoyStick();
        void setLostPerSecond();
        int getJoyStick(XYZT which){return joyStick[which];};
        bool getEnable(){      
            static bool lowered = false;
            if(enable == true and joyStick[T] < recie_minJoyAnalog+10) lowered = true;
            if(enable == false) lowered = false;
            if(lowered and enable) return true;
            else return false;
        }
};


#ifdef nrff
    #include "rc/nrf.h"

    class RECIEVER:public RECIEVER_BASE{
        private:
            NRF r;
            void updatePayLoad(float, float, float, float, float, 
                               float, float, int, int, int, int);
        public:
            bool read();
            void begin();
            void update();
            bool isTimeOut();
            void resetJoyStick();
    };

    void RECIEVER::begin(){
        r.begin();
    }

    void RECIEVER::update(){
        read(); // read
        if(isTimeOut()) resetJoyStick();  // 检查有没有超时
        updatePayLoad(voltage.getVoltage(V3), imu.getAngle(X), imu.getAngle(Y), imu.getAngle(Z), 
                                imu.getGyro(X) , imu.getGyro(Y), imu.getGyro(Z), 
                                motorA.getStandardisedPwm(), motorB.getStandardisedPwm(), 
                                motorC.getStandardisedPwm(), motorD.getStandardisedPwm());
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


    void RECIEVER::resetJoyStick(){
        r.reset_pwm(); 
        extra = false;
        enable = false;
        joyStick[3] = recie_minJoyAnalog;
        for(int i=0; i<3; i++) joyStick[i] = recie_midJoyAnalog;
    }


    void RECIEVER::updatePayLoad(float v , float x, float y, float z, float gx, float gy, float gz, int p1, int p2, int p3, int p4){
        r.setAck_agl(x, y, z);
        r.setAck_vgt(v);
        r.setAck_pwm(p1, p2, p3, p4);
        r.setAck_gyr(gx, gy, gz);
    }

#endif




#ifdef sbus
    #include "rc/mysbus.h"

    class RECIEVER:public RECIEVER_BASE{
        private:
            MYSBUS r;
        public:
            bool read();
            void begin();
            void update();
            bool isTimeOut();
            void resetJoyStick();
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

    void RECIEVER::resetJoyStick(){
        r.resetChannel();
        extra = false;
        enable = false;
        joyStick[3] = recie_minJoyAnalog;
        for(int i=0; i<3; i++) joyStick[i] = recie_midJoyAnalog;
    }

#endif



RECIEVER reciever;

#endif