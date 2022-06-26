#ifndef _mpu_h_
#define _mpu_h_

/*
<MPU6050_6Axis_MotionApps20.h> + <I2Cdev.h>
MPU-6050 6-axis accelerometer/gyroscope Arduino Library adapted for Arduino Library Manager by Electronic Cats
https://github.com/ElectronicCats/mpu6050
*/

#include "config.h"
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>


// 我感觉单单用dmp也可以(没试过)

MPU6050 mpu;
Quaternion q;
VectorFloat gravity;


class MPU{
  private:
    float yrp[3];
    uint16_t packetSize;
    uint16_t fifoCount;
    uint8_t fifoBuffer[64];

    float angle[3]; // x, y, z
    int sample_count = 0;
    
    // x, y, z
    float cur_Angle[3] = {0, 0, 0}; 
    float cur_Gyro[3] = {0, 0, 0};
    float pre_Gyro[3] = {0, 0, 0};
    float GyroChange[3] = {0, 0, 0};
    float AccChange[3] = {0, 0, 0};
    float GyroCorrectVal[3] = {0, 0, 0};
    float cur_Acc[3] = {0, 0, 0};
    float pre_Acc[3] = {0, 0, 0};
    float cur_AccAngle[3] = {0, 0, 0};  

    
  public:
    void begin();
    void update();
    float getAcc(XYZT);
    float getGyro(XYZT);
    float getAngle(XYZT);
    float getAccAngle(XYZT);
    void correctZshift();
    int getSampleCount(boolean);
};


void MPU::begin(){
  Wire.begin();
  Wire.setClock(1000000);
  mpu.initialize();
  mpu.dmpInitialize();
  // auto calibrate
  Serial.println("");
  Serial.println("");
  Serial.println("calibrate mpu");

  #ifdef mpu_manu_gyro_offset
  mpu.setXGyroOffset(mpu_gyro_offsetX);
  mpu.setYGyroOffset(mpu_gyro_offsetY);
  mpu.setZGyroOffset(mpu_gyro_offsetZ);
  #else
  delay(3000);
  mpu.CalibrateGyro(5);
  mpu.setXGyroOffset(mpu.getXGyroOffset());
  mpu.setYGyroOffset(mpu.getYGyroOffset());
  mpu.setZGyroOffset(mpu.getZGyroOffset());
  #endif

  #ifdef mpu_manu_acc_offset
  mpu.setXAccelOffset(mpu_acc_offsetX);
  mpu.setYAccelOffset(mpu_acc_offsetY);
  mpu.setZAccelOffset(mpu_acc_offsetZ);  
  #else
  mpu.CalibrateAccel(5);
  mpu.setXAccelOffset(mpu.getXAccelOffset());
  mpu.setYAccelOffset(mpu.getYAccelOffset());
  mpu.setZAccelOffset(mpu.getZAccelOffset());
  #endif

  mpu.PrintActiveOffsets();
  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();
  fifoCount = mpu.getFIFOCount();
}


void MPU::update(){
  static unsigned long timer_angle = micros();
  static unsigned long timer_sample1 = millis(); 
  static unsigned long timer_sample2 = millis(); 
  static unsigned long timer_sample3 = millis(); 

  if(millis() - timer_sample1 >= 5){
    // get gyro raw data
    sample_count += 1;
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t) 0x68, (uint8_t) 6);
    float raw_gyro[3]; // x, y, z
    int16_t rawData_gyro[3]; // x, y, z
    for (int i = 0; i < 3; i++) {
      rawData_gyro[i]  = Wire.read() << 8;
      rawData_gyro[i] |= Wire.read();
    }

    raw_gyro[X] = (float)rawData_gyro[X] / 16.4f;
    raw_gyro[Y] = (float)rawData_gyro[Y] / 16.4f;
    raw_gyro[Z] = (float)rawData_gyro[Z] / 16.4f;
    raw_gyro[X] = -raw_gyro[X];
    raw_gyro[Y] =  raw_gyro[Y];
    raw_gyro[Z] = -raw_gyro[Z];
    for(int i=0; i<3; i++) raw_gyro[i] = raw_gyro[i];

    #ifdef mpu_invertX
      raw_gyro[X] = -raw_gyro[X];
    #endif
    #ifdef mpu_invertY
      raw_gyro[Y] = -raw_gyro[Y];
    #endif
    #ifdef mpu_invertZ
      raw_gyro[Z] = -raw_gyro[Z];
    #endif
    #ifdef mpu_swapXY
      float swap_temp = raw_gyro[X];
      raw_gyro[X] = raw_gyro[Y];
      raw_gyro[Y] = swap_temp;
    #endif

    correctZshift();

    #ifdef gyroFilter
      cur_Gyro[X] = filterGX.updateEstimate(raw_gyro[X]) - GyroCorrectVal[X];
      cur_Gyro[Y] = filterGY.updateEstimate(raw_gyro[Y]) - GyroCorrectVal[Y];
      cur_Gyro[Z] = filterGZ.updateEstimate(raw_gyro[Z]) - GyroCorrectVal[Z];
    #else
      for(int i=0; i<3; i++) cur_Gyro[i] = raw_gyro[i];
    #endif

    for(int i=0; i<3; i++) GyroChange[i] = cur_Gyro[i] - pre_Gyro[i];
    for(int i=0; i<3; i++) pre_Gyro[i] = cur_Gyro[i];

    // calculate angle
    float dta = (float)(micros() - timer_angle) / 1000000.0f;
    angle[X] +=  cur_Gyro[X] * dta;
    angle[Y] +=  cur_Gyro[Y] * dta;
    angle[Z] +=  cur_Gyro[Z] * dta;
    angle[X] += angle[Y] * sin(cur_Gyro[Z] * dta * 0.017453292f);
    angle[Y] -= angle[X] * sin(cur_Gyro[Z] * dta * 0.017453292f);
    timer_angle = micros();

    if(angle[Z] > 180.0f) angle[Z] = -180 + (angle[Z] - 180);
    if(angle[Z] < -180.0f) angle[Z] = 180 + (angle[Z] + 180);

    #ifdef angleFilter
      cur_Angle[X] = filterX.updateEstimate(angle[X]);
      cur_Angle[Y] = filterY.updateEstimate(angle[Y]);
      cur_Angle[Z] = angle[Z];
    #else
      cur_Angle[X] = angle[X];
      cur_Angle[Y] = angle[Y];
      cur_Angle[Z] = angle[Z];
    #endif

    timer_sample1 = millis();
  }


  // dmp
  if(millis() - timer_sample2 > 10){
    fifoCount = mpu.getFIFOCount();
    if ((!fifoCount) || (fifoCount % packetSize)) mpu.resetFIFO();
    else{
      // get dmp
      while (fifoCount >= packetSize) {
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
      }

      // fetch
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(yrp, &q, &gravity);

      float angle_dmp[3];
      angle_dmp[X] = (-yrp[Z] * 180.0f / PI);
      angle_dmp[Y] = (-yrp[Y] * 180.0f / PI);
      angle_dmp[Z] = ( yrp[X] * 180.0f / PI);

      #ifdef mpu_invertX
        angle_dmp[X] = -angle_dmp[X];
      #endif
      #ifdef mpu_invertY
        angle_dmp[Y] = -angle_dmp[Y];
      #endif
      #ifdef mpu_invertZ
        angle_dmp[Z] = -angle_dmp[Z];
      #endif
      #ifdef mpu_swapXY
        float swap_temp2 = angle_dmp[X];
        angle_dmp[X] = angle_dmp[Y];
        angle_dmp[Y] = swap_temp2;
      #endif

      angle[X] = angle[X] * (1.0f-dmpAngleWeight) + angle_dmp[X] * dmpAngleWeight;
      angle[Y] = angle[Y] * (1.0f-dmpAngleWeight) + angle_dmp[Y] * dmpAngleWeight;
      // 没使用z dmp，因为输出波形有锯齿。
      // angle[Z] = angle[Z] * (1.0f-dmpZAngleWeight) + angle_dmp[Z] * dmpZAngleWeight;
      
      #ifdef angleFilter
        cur_Angle[X] = filterX.updateEstimate(angle[X]);
        cur_Angle[Y] = filterY.updateEstimate(angle[Y]);
        cur_Angle[Z] = angle[Z];
      #else
        cur_Angle[X] = angle[X];
        cur_Angle[Y] = angle[Y];
        cur_Angle[Z] = angle[Z];
      #endif

      timer_sample2 = millis();
    }
  }


  if(millis() - timer_sample3 >= 1){
    // get raw acc
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t) 0x68, (uint8_t) 6);
    float raw_acc[3];
    int16_t rawData_acc[3];
    for (int i = 0; i < 3; i++) {
      rawData_acc[i]  = Wire.read() << 8;
      rawData_acc[i] |= Wire.read();
    }
    raw_acc[X] = ((float)rawData_acc[X]) / 16384.0f;
    raw_acc[Y] = ((float)rawData_acc[Y]) / 16384.0f;
    raw_acc[Z] = ((float)rawData_acc[Z]) / 16384.0f;
    cur_Acc[X] = raw_acc[X];
    cur_Acc[Y] = raw_acc[Y];
    cur_Acc[Z] = raw_acc[Z];

    for(int i=0; i<3; i++) AccChange[i] = cur_Acc[i] - pre_Acc[i];
    for(int i=0; i<3; i++) pre_Acc[i] = cur_Acc[i];
    
    float temp_cur_AccAngle[2];
    float total_Acc = sqrt((cur_Acc[X] * cur_Acc[X]) + (cur_Acc[Y] * cur_Acc[Y]) + (cur_Acc[Z] * cur_Acc[Z]));
    if(cur_Acc[Y] < total_Acc)temp_cur_AccAngle[X] = -(asin(cur_Acc[Y]/total_Acc)* 57.296);
    if(cur_Acc[X] < total_Acc)temp_cur_AccAngle[Y] = -(asin(cur_Acc[X]/total_Acc)* 57.296);

    #ifdef mpu_invertX
      temp_cur_AccAngle[X] = -temp_cur_AccAngle[X];
    #endif
    #ifdef mpu_invertY
      temp_cur_AccAngle[Y] = -temp_cur_AccAngle[Y];
    #endif
    #ifdef mpu_swapXY
      float temp;
      temp = temp_cur_AccAngle[X];
      temp_cur_AccAngle[X] = temp_cur_AccAngle[Y];
      temp_cur_AccAngle[Y] = temp;
    #endif

    cur_AccAngle[X] = temp_cur_AccAngle[X];
    cur_AccAngle[Y] = temp_cur_AccAngle[Y];
    

    angle[X] = angle[X] * (1.0f-accAngleWeight) + cur_AccAngle[X] * accAngleWeight;
    angle[Y] = angle[Y] * (1.0f-accAngleWeight) + cur_AccAngle[Y] * accAngleWeight;
    #ifdef angleFilter
      cur_Angle[X] = filterX.updateEstimate(angle[X]);
      cur_Angle[Y] = filterY.updateEstimate(angle[Y]);
    #else
      cur_Angle[X] = angle[X];
      cur_Angle[Y] = angle[Y];
    #endif
    timer_sample3 = millis();
  }

}


float MPU::getAngle(XYZT which){
  return cur_Angle[which];
}


float MPU::getGyro(XYZT which){
  return cur_Gyro[which];
}


float MPU::getAcc(XYZT which){
  return cur_Acc[which];
}

float MPU::getAccAngle(XYZT which){
  return cur_AccAngle[which];
}

int MPU::getSampleCount(boolean reset){
  int temp = sample_count;
  if(reset) sample_count = 0;
  return temp;
}

void MPU::correctZshift(){
  bool AccChanged = false;
  bool GyroChanged = false;
  if(abs(AccChange[X]) > 0.04f or abs(AccChange[Y]) > 0.04f or abs(AccChange[Z]) > 0.04f) AccChanged = true;
  if(abs(GyroChange[X]) > 0.1f or abs(GyroChange[Y]) > 0.1f or abs(GyroChange[Z]) > 0.1f) GyroChanged = true;
  if(not(AccChange) and not(GyroChanged) and cur_Gyro[Z] != 0.0f) GyroCorrectVal[Z] += 0.0f - cur_Gyro[Z];
}

MPU imu;

#endif
