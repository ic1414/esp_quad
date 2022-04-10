#ifndef _mpu_h_
#define _mpu_h_

#include "Arduino.h"
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SimpleKalmanFilter.h>

//#define manu_gyro_offset
#define gyro_offsetX  134.0f
#define gyro_offsetY  1.0f
#define gyro_offsetZ  134.0f

//#define manu_acc_offset
#define acc_offsetX  81.0f
#define acc_offsetY  41.0f
#define acc_offsetZ -20.0f

MPU6050 mpu;
Quaternion q;
VectorFloat gravity;

#define angleFilter
SimpleKalmanFilter filterX(5.0f, 5.0f, 0.1f);
SimpleKalmanFilter filterY(5.0f, 5.0f, 0.1f);
SimpleKalmanFilter filterZ(5.0f, 5.0f, 0.1f);

SimpleKalmanFilter filterGX(15.0f, 15.0f, 0.06f);
SimpleKalmanFilter filterGY(15.0f, 15.0f, 0.06f);
SimpleKalmanFilter filterGZ(15.0f, 15.0f, 0.06f);

class MPU{
    private:
      float yrp[3];
      uint16_t packetSize;
      uint16_t fifoCount;
      uint8_t fifoBuffer[64];
      float angle[3];
      int loop_count = 0;

    public:
      int sample_count = 0;
      float cur_Angle[3];
      float cur_Gyro[3] = {0, 0, 0};
      void begin();
      void update();
};


void MPU::begin(){
  //wire begin
  Wire.begin();
  Wire.setClock(1000000);
  // mpu begin
  mpu.initialize();
  //mpu.setFullScaleGyroRange(1);
  mpu.setFullScaleAccelRange(2);
  mpu.dmpInitialize();
  // auto calibrate
  Serial.println("calibrate mpu");
  delay(3000);
  mpu.CalibrateAccel(5);
  mpu.CalibrateGyro(5);

  #ifdef manu_gyro_offset
  mpu.setXGyroOffset(gyro_offsetX);
  mpu.setYGyroOffset(gyro_offsetY);
  mpu.setZGyroOffset(gyro_offsetZ);
  #else
  mpu.setXGyroOffset(mpu.getXGyroOffset());
  mpu.setYGyroOffset(mpu.getYGyroOffset());
  mpu.setZGyroOffset(mpu.getZGyroOffset());
  #endif

  #ifdef acc_gyro_offset
  mpu.setXAccelOffset(acc_offsetX);
  mpu.setYAccelOffset(acc_offsetY);
  mpu.setZAccelOffset(acc_offsetZ);  
  #else
  mpu.setXAccelOffset(mpu.getXAccelOffset());
  mpu.setYAccelOffset(mpu.getYAccelOffset());
  mpu.setZAccelOffset(mpu.getZAccelOffset());
  #endif

  // print offsets
  mpu.PrintActiveOffsets();

  // continue
  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();
  fifoCount = mpu.getFIFOCount();
    
}


void MPU::update(){
  static unsigned long timer_gyro = micros();  

  while (fifoCount < packetSize) {
    // update counts  
    sample_count += 1;

    float raw_gyro_avg[3] = {0, 0, 0};
    for(int i = 0; i < 3; i++){
      loop_count += 1;  
      // get gyro raw data
      Wire.beginTransmission(0x68);
      Wire.write(0x43);
      Wire.endTransmission(false);
      Wire.requestFrom((uint16_t) 0x68, (uint8_t) 6);

      float raw_gyro[3];
      int16_t rawData_gyro[3];
      for (int i = 0; i < 3; i++) {
        rawData_gyro[i]  = Wire.read() << 8;
        rawData_gyro[i] |= Wire.read();
      }

      raw_gyro[0] = (float)rawData_gyro[0] / 16.4f;
      raw_gyro[1] = (float)rawData_gyro[1] / 16.4f;
      raw_gyro[2] = (float)rawData_gyro[2] / 16.4f;
      raw_gyro[0] = -raw_gyro[0];
      raw_gyro[1] =  raw_gyro[1];
      raw_gyro[2] = -raw_gyro[2];

      raw_gyro_avg[0] += filterGX.updateEstimate(raw_gyro[0] / 3);
      raw_gyro_avg[1] += filterGY.updateEstimate(raw_gyro[1] / 3);
      raw_gyro_avg[2] += filterGZ.updateEstimate(raw_gyro[2] / 3);

      delayMicroseconds(4);
    }

    cur_Gyro[0] = raw_gyro_avg[0];
    cur_Gyro[1] = raw_gyro_avg[1];
    cur_Gyro[2] = raw_gyro_avg[2];

    // calculate angle
    float dt = ((float)(micros() - timer_gyro)) / 1000000.0f;
    if(dt < 0) dt = dt * -1.0f;
    angle[0] +=  cur_Gyro[0] * dt;
    angle[1] +=  cur_Gyro[1] * dt;
    angle[2] +=  cur_Gyro[2] * dt;
    angle[0] -= angle[0] * sin(cur_Gyro[2] * dt * 0.01745f);
    angle[1] += angle[1] * sin(cur_Gyro[2] * dt * 0.01745f);

    // update timer
    timer_gyro = micros();

    // fun
    #ifdef angleFilter
    cur_Angle[0] = filterX.updateEstimate(angle[0]);
    cur_Angle[1] = filterY.updateEstimate(angle[1]);
    cur_Angle[2] = filterZ.updateEstimate(angle[2]);
    #else
    cur_Angle[0] = angle[0];
    cur_Angle[1] = angle[1];
    cur_Angle[2] = angle[2];
    #endif
    delayMicroseconds(3);
    // get fifo less frequently to increase sample rate
    if (loop_count == 24) fifoCount = mpu.getFIFOCount();
    if (loop_count == 24) loop_count = 0; // reset
  }

  // get fifo angle
  if (fifoCount == 840 or fifoCount % packetSize != 0) fifoCount = 0;
  if (fifoCount == 840 or fifoCount % packetSize != 0) mpu.resetFIFO();
  else {
    // update counts
    loop_count = 0;    
    sample_count += 1;

    // get dmp
    while (fifoCount >= packetSize) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
    }

    // calculate
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(yrp, &q, &gravity);
    angle[0] = angle[0] * 0.40f + (-yrp[2] * 180.0f / PI) * 0.60f;
    angle[1] = angle[1] * 0.40f + (-yrp[1] * 180.0f / PI) * 0.60f;
    angle[2] = angle[1] * 0.10f + ( yrp[0] * 180.0f / PI) * 0.90f;

    // update timer
    timer_gyro = micros();

    // fun
    #ifdef angleFilter
    cur_Angle[0] = filterX.updateEstimate(angle[0]);
    cur_Angle[1] = filterY.updateEstimate(angle[1]);
    cur_Angle[2] = filterZ.updateEstimate(angle[2]);
    #else
    cur_Angle[0] = angle[0];
    cur_Angle[1] = angle[1];
    cur_Angle[2] = angle[2];
    #endif

    delayMicroseconds(5);
  }

}



#endif
