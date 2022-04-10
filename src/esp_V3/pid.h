#ifndef _pid_h_
#define _pid_h_

#include "Arduino.h"


class PID {
  private:
    // output limit  
    float kp;
    float ki;
    float kd;
    int limitP;
    int limitI;
    int limitD;
    float integral;
    float pre_error;

  public:
    // constant
    // functions
    PID(float p, float i, float d, int lp, int li, int ld);
    int _calculate(float set_point, float measure, float dt);
    float get_integral();
    void integral_reset();
    void set_pid(float, float, float);
};


PID::PID(float p, float i, float d, int lp, int li, int ld){
  kp = p;
  ki = i;
  kd = d;
  limitP = lp;
  limitI = li;
  limitD = ld;
}

void PID::integral_reset(){
  integral = 0;
}

void PID::set_pid(float p, float i, float d){
  kp = p;
  ki = i;
  kd = d;
}

float PID::get_integral(){
  return integral;
}

int PID::_calculate(float set_point, float measure, float dt){

  // error
  float cur_error = set_point - measure;

  // propotion
  float propotion_out = cur_error * kp;
  propotion_out = constrain(propotion_out, -limitP, limitP);

  // intergral
  integral = integral + (cur_error / 1000 * dt);
  integral = constrain(integral, (float)-limitI/ki, (float)limitI/ki);
  float integral_out = integral * ki;

  // derivative
  // ...
  float derivative_out = (cur_error - pre_error) * kd / dt;
  derivative_out = constrain(derivative_out, -limitD, limitD);
  pre_error = cur_error;

  // out
  float out = (propotion_out + integral_out + derivative_out);
  return out;
}



#endif