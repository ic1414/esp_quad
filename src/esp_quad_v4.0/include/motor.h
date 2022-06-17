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


// 电机初始化
#ifdef brushed
    #define resolution 8
    #define frequency 5000

    class MOTOR{
        private:
            int channel = -1;
            int standardisedAPwm = 0; //analog
            int standardisedPwm = motor_minPwm;
            int astandardisedPwm = motor_minPwm;

        public:
            MOTOR(int, int);
            int getStandardisedPwm();
            int getAStandardisedPwm();
            void setStandardisedPwm(int);
            void setAStandardisedPwm(int);
            void setStandardisedPwm_write(int, bool);
    };

    MOTOR::MOTOR(int channe, int pin){
        channel = channe;
        ledcSetup(channel, frequency, resolution);
        ledcAttachPin(pin, channel);
    }

    void MOTOR::setStandardisedPwm(int pwm){
        setAStandardisedPwm(pwm);
        standardisedPwm = constrain(pwm, motor_minPwm, motor_maxPwm);
        standardisedAPwm = (standardisedPwm - motor_minPwm) / 4; 
    }

    void MOTOR::setStandardisedPwm_write(int pwm, bool running){
        setAStandardisedPwm(pwm);
        if(running) standardisedPwm = constrain(pwm, motor_minPwmRunning, motor_maxPwm);
        else standardisedPwm = constrain(pwm, motor_minPwm, motor_maxPwm);
        standardisedAPwm = (standardisedPwm - motor_minPwm) / 4;
        ledcWrite(channel, standardisedAPwm);
    }

    int MOTOR::getStandardisedPwm(){
        return standardisedPwm;
    }

    void MOTOR::setAStandardisedPwm(int pwm){
        astandardisedPwm = pwm;
    }

    int MOTOR::getAStandardisedPwm(){
        return astandardisedPwm;
    }


#else
    #ifdef dshot
        #include <DShotRMT.h>

        class MOTOR{
            private:
                DShotRMT m;
                int standardisedPwm = motor_minPwm;
                int astandardisedPwm = motor_minPwm;
            public:
                MOTOR(int, int);
                int getStandardisedPwm();
                int getAStandardisedPwm();
                void setStandardisedPwm(int);
                void setAStandardisedPwm(int);
                void setStandardisedPwm_write(int, bool);
        };

        MOTOR::MOTOR(int channel, int pin){
            // 我把库稍微改了下
            m.setPin(pin);
            m.setChannel(channel);
            m.begin(DSHOT150, false);
        }

        void MOTOR::setStandardisedPwm(int pwm){
            standardisedPwm = constrain(pwm, motor_minPwm, motor_maxPwm);
        }

        void MOTOR::setStandardisedPwm_write(int pwm, bool running){
            if(running) standardisedPwm = constrain(pwm, motor_minPwmRunning, motor_maxPwm); // 
            else standardisedPwm = constrain(pwm, motor_minPwm, motor_maxPwm);
            m.send_dshot_value((standardisedPwm - motor_minPwm) * 2 + 48); // (1000到2000-1000)*2+48 = 48到2048
        }

        int MOTOR::getStandardisedPwm(){
            return standardisedPwm;
        }

        void MOTOR::setAStandardisedPwm(int pwm){
            astandardisedPwm = pwm;
        }

        int MOTOR::getAStandardisedPwm(){
            return astandardisedPwm;
        }

    #else
        #include <ESP32Servo.h>
        
        class MOTOR{
            private:
                Servo m;
                int standardisedPwm = motor_minPwm;
                int astandardisedPwm = motor_minPwm;
            public:
                MOTOR(int, int);
                int getStandardisedPwm();
                int getAStandardisedPwm();
                void setStandardisedPwm(int);
                void setAStandardisedPwm(int);
                void setStandardisedPwm_write(int, bool);
        };

        MOTOR::MOTOR(int timer, int pin){
            ESP32PWM::allocateTimer(timer);
            m.setPeriodHertz(400);
            m.attach(pin, motor_minPwm, motor_maxPwm);
        }

        void MOTOR::setStandardisedPwm(int pwm){
            setAStandardisedPwm(pwm);
            standardisedPwm = constrain(pwm, motor_minPwm, motor_maxPwm);
        }

        void MOTOR::setStandardisedPwm_write(int pwm, bool running){
            setAStandardisedPwm(pwm);
            if(running) standardisedPwm = constrain(pwm, motor_minPwmRunning, motor_maxPwm);
            else standardisedPwm = constrain(pwm, motor_minPwm, motor_maxPwm);
            m.writeMicroseconds(standardisedPwm);
        }

        int MOTOR::getStandardisedPwm(){
            return standardisedPwm;
        }

        void MOTOR::setAStandardisedPwm(int pwm){
            astandardisedPwm = pwm;
        }

        int MOTOR::getAStandardisedPwm(){
            return astandardisedPwm;
        }
    #endif

#endif



MOTOR motorA(0, motorA_pin);
MOTOR motorB(1, motorB_pin);
MOTOR motorC(2, motorC_pin);
MOTOR motorD(3, motorD_pin);


#endif
