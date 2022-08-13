#ifndef _pid_h_
#define _pid_h_

#include "config.h"


#define dtermFilter

class PID {
  private:
    // output limit  
    int limitP;
    int limitI;
    int limitD;
    int limitO;
    float arrayPID[3];
    float integral = 0;
    float pre_error = 0;
    float pre_derivative_out = 0;
  public:
    float getPid(PIDD);
    float getIntegral();
    void setIntegral(float);
    void setPid(float, float, float);
    int calculate(float, float, float);
    int calculate(float, float, float, SimpleKalmanFilter*);
    int calculate(float, float, float, SimpleKalmanFilter*, bool);
    PID(float, float, float, int, int, int);
};


PID::PID(float p, float i, float d, int lp, int li, int ld){
  arrayPID[P] = p;
  arrayPID[I] = i;
  arrayPID[D] = d;
  limitP = lp;
  limitI = li;
  limitD = ld;
}

void PID::setIntegral(float t){
  integral = t;
}

float PID::getIntegral(){
  return integral;
}

void PID::setPid(float p, float i, float d){
  arrayPID[P] = p;
  arrayPID[I] = i;
  arrayPID[D] = d;
}

float PID::getPid(PIDD which){
  return arrayPID[which];
}



int PID::calculate(float set_point, float measure, float dt){
  // error
  float cur_error = set_point - measure;

  // propotion
  float propotion_out = cur_error * arrayPID[P];
  propotion_out = constrain(propotion_out, -limitP, limitP);

  // intergral
  integral = integral + (cur_error / 1000 * dt);
  integral = constrain(integral, (float)-limitI/arrayPID[I], (float)limitI/arrayPID[I]);
  float integral_out = integral * arrayPID[I];

  float derivative_out = (cur_error - pre_error) * arrayPID[D] / dt;
  derivative_out = constrain(derivative_out, -limitD, limitD) * 0.90 + pre_derivative_out * 0.10;
  pre_derivative_out = derivative_out;
  pre_error = cur_error;

  // out
  float out = (propotion_out + integral_out + derivative_out);
  return out;
}



int PID::calculate(float set_point, float measure, float dt, SimpleKalmanFilter *filterD){
  // error
  float cur_error = set_point - measure;

  // propotion
  float propotion_out = cur_error * arrayPID[P];
  propotion_out = constrain(propotion_out, -limitP, limitP);

  // intergral
  integral = integral + (cur_error / 1000 * dt);
  integral = constrain(integral, (float)-limitI/arrayPID[I], (float)limitI/arrayPID[I]);
  float integral_out = integral * arrayPID[I];

  // derivative
  float derivative_out = (cur_error - pre_error) * arrayPID[D] / dt;
  derivative_out = filterD->updateEstimate(constrain(derivative_out, -limitD, limitD));

  pre_derivative_out = derivative_out;
  pre_error = cur_error;

  // out
  float out = (propotion_out + integral_out + derivative_out);
  return out;
}


int PID::calculate(float set_point, float measure, float dt, SimpleKalmanFilter *filterD, bool integral_enable){
  // error
  float cur_error = set_point - measure;

  // propotion
  float propotion_out = cur_error * arrayPID[P];
  propotion_out = constrain(propotion_out, -limitP, limitP);

  // intergral
  float integral_out = integral * arrayPID[I];
  if(integral_enable){
    integral = integral + (cur_error / 1000 * dt);
    integral = constrain(integral, (float)-limitI/arrayPID[I], (float)limitI/arrayPID[I]);
    integral_out = integral * arrayPID[I];
  }

  // derivative
  float derivative_out = (cur_error - pre_error) * arrayPID[D] / dt;
  derivative_out = filterD->updateEstimate(constrain(derivative_out, -limitD, limitD));

  pre_derivative_out = derivative_out;
  pre_error = cur_error;

  // out
  float out = (propotion_out + integral_out + derivative_out);
  return out;
}


// 前三个参数是k(pid)。后三个是每个控制器的pwm输出限制。
// 如果开蓝牙调参的话，可以不设置k(pid)，因为会被EEPROM里的k(pid)覆盖掉。 
PID pidX(1.6f, 0.0f, 350.0f, 250, 100, 200);
PID pidY(1.6f, 0.0f, 350.0f, 250, 100, 200);
PID pidZ(4.5f, 0.0f, 0.000f, 250, 100, 200); 


#endif