#ifndef _motor_h_
#define _motor_h_

#include "Arduino.h"
#include <ESP32Servo.h>

Servo   motorA;
Servo   motorB;
Servo   motorC;
Servo   motorD;
#define motorA_pin 32
#define motorB_pin 33
#define motorC_pin 27
#define motorD_pin 13


void motor_begin(){
    //initiate motors
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    motorA.setPeriodHertz(400);
    motorB.setPeriodHertz(400);
    motorC.setPeriodHertz(400);
    motorD.setPeriodHertz(400);
    motorA.attach(motorA_pin, 1000, 2000);
    motorB.attach(motorB_pin, 1000, 2000);
    motorC.attach(motorC_pin, 1000, 2000);
    motorD.attach(motorD_pin, 1000, 2000);
}








#endif
