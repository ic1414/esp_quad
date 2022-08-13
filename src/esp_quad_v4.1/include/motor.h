#ifndef _motor_h_
#define _motor_h_


/*
<DShotRMT.h>
-author
Wastl Kraus
https://github.com/derdoktor667/DShotRMT

<ESP32Servo.h>
-author
John K. Bennett
https://github.com/madhephaestus/ESP32Servo
*/

#include "config.h"


class MOTOR_BASE{
    protected:
        int standardisedPwm = motor_minPwm;
        int astandardisedPwm = motor_minPwm;
    public:
        int getStandardisedPwm(){return standardisedPwm;};
        int getAStandardisedPwm(){return astandardisedPwm;};
        void setStandardisedPwm(int pwm){standardisedPwm = constrain(pwm, motor_minPwm, motor_maxPwm);};
        void setStandardisedPwmRun(int pwm){standardisedPwm = constrain(pwm, motor_minPwmRunning, motor_maxPwm);};
        void setAStandardisedPwm(int pwm){astandardisedPwm = pwm;};
        void setStandardisedPwm_write();
};


// 电机初始化
#ifdef brushed
    #define resolution 8
    #define frequency 5000

    class MOTOR:public MOTOR_BASE{
        private:
            int channel = -1;
            int standardisedAPwm = 0; //analog
            int getStandardisedAPwm();
            void setStandardisedAPwm();
        public:
            MOTOR(int, int);
            void setStandardisedPwm_write(int, bool);
    };

    MOTOR::MOTOR(int channe, int pin){
        channel = channe;
        ledcSetup(channel, frequency, resolution);
        ledcAttachPin(pin, channel);
    }

    void MOTOR::setStandardisedPwm_write(int pwm, bool running){
        if(running) setStandardisedPwmRun(pwm);
        else setStandardisedPwm(pwm);
        setStandardisedAPwm();
        ledcWrite(channel, standardisedAPwm);
    }

    int MOTOR::getStandardisedAPwm(){
        return standardisedAPwm;
    }

    void MOTOR::setStandardisedAPwm(){
        standardisedAPwm = (standardisedPwm - motor_minPwm) / 4; 
    }
#endif


#ifdef dshot
    #include <DShotRMT.h>

    class MOTOR:public MOTOR_BASE{
        private:
            DShotRMT m;
        public:
            MOTOR(int, int);
            void setStandardisedPwm_write(int, bool);
    };

    MOTOR::MOTOR(int channel, int pin){
        // 我把库稍微改了下
        m.setPin(pin);
        m.setChannel(channel);
        m.begin(DSHOT150, false);
    }

    void MOTOR::setStandardisedPwm_write(int pwm, bool running){
        if(running) setStandardisedPwmRun(pwm); // 
        else setStandardisedPwm(pwm);
        m.send_dshot_value((standardisedPwm - motor_minPwm) * 2 + 48); // (1000到2000-1000)*2+48 = 48到2048
    }

#endif


#ifdef pwmm
    #include <ESP32Servo.h>
    
    class MOTOR:public MOTOR_BASE{
        private:
            Servo m;
        public:
            MOTOR(int, int);
            void setStandardisedPwm_write(int, bool);
    };

    MOTOR::MOTOR(int timer, int pin){
        ESP32PWM::allocateTimer(timer);
        m.setPeriodHertz(400);
        m.attach(pin, motor_minPwm, motor_maxPwm);
    }

    void MOTOR::setStandardisedPwm_write(int pwm, bool running){
        if(running) setStandardisedPwmRun(pwm);
        else setStandardisedPwm(pwm);
        m.writeMicroseconds(standardisedPwm);
    }

#endif



MOTOR motorA(0, motorA_pin);
MOTOR motorB(1, motorB_pin);
MOTOR motorC(2, motorC_pin);
MOTOR motorD(3, motorD_pin);


#endif
