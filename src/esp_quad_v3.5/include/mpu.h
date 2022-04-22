#ifndef _mpu_h_
#define _mpu_h_

#include "Arduino.h"
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SimpleKalmanFilter.h>


// 我感觉单单用dmp也可以(没试过)
// 至于我为什么要加上一点自己算的单纯心理作用，代码越长越厉害, 还有esp32那么多算力可不能浪费了


//#define manu_gyro_offset
#define gyro_offsetX  134.0f
#define gyro_offsetY  1.0f
#define gyro_offsetZ  134.0f

//#define manu_acc_offset
#define acc_offsetX  81.0f
#define acc_offsetY  41.0f
#define acc_offsetZ -10.0f

MPU6050 mpu;
Quaternion q;
VectorFloat gravity;

#define angleFilter
SimpleKalmanFilter filterX(5.0f, 5.0f, 0.1f);
SimpleKalmanFilter filterY(5.0f, 5.0f, 0.1f);
SimpleKalmanFilter filterZ(5.0f, 5.0f, 0.1f);

#define gyroFilter
SimpleKalmanFilter filterGX(10.0f, 10.0f, 0.05f);
SimpleKalmanFilter filterGY(10.0f, 10.0f, 0.05f);
SimpleKalmanFilter filterGZ(10.0f, 10.0f, 0.05f);

#define accFilter
SimpleKalmanFilter filterAX(0.1f, 0.1f, 0.001f);
SimpleKalmanFilter filterAY(0.1f, 0.1f, 0.001f);
SimpleKalmanFilter filterAZ(0.1f, 0.1f, 0.001f);



class MPU{
    private:
      float yrp[3];
      uint16_t packetSize;
      uint16_t fifoCount;
      uint8_t fifoBuffer[64];
      float angle[3]; // x, y, z

    public:
      int sample_count = 0;
      float pre_Angle[3];
      float cur_Angle[3]; // x, y, z
      float pre_Gyro[3];
      float cur_Gyro[3] = {0, 0, 0}; // x, y, z
      float pre_Acc[3];
      float cur_Acc[3];
      float cur_Acc_sum[3] = {0, 0, 0};
      void begin();
      void update();
};


void MPU::begin(){
  Wire.begin();
  Wire.setClock(1000000); //我不知道为什么1mhz也可以用， datasheet说最高400khz
  mpu.initialize();
  //mpu.setFullScaleAccelRange(2);
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

  mpu.PrintActiveOffsets();
  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();
  fifoCount = mpu.getFIFOCount();
}


void MPU::update(){
  static unsigned long timer_gyro = micros();  
  static unsigned long timer_sample1 = micros(); 
  static unsigned long timer_sample2 = millis(); 

  if(micros() - timer_sample1 >= 300){
    sample_count += 1;
    // get gyro raw data
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

    raw_gyro[0] = (float)rawData_gyro[0] / 16.4f;
    raw_gyro[1] = (float)rawData_gyro[1] / 16.4f;
    raw_gyro[2] = (float)rawData_gyro[2] / 16.4f;
    raw_gyro[0] = -raw_gyro[0];
    raw_gyro[1] =  raw_gyro[1];
    raw_gyro[2] = -raw_gyro[2];
    // 平滑输出
    raw_gyro[0] = raw_gyro[0]*0.90f + pre_Gyro[0] * 0.10f;
    raw_gyro[1] = raw_gyro[1]*0.90f + pre_Gyro[1] * 0.10f;
    raw_gyro[2] = raw_gyro[2]*0.90f + pre_Gyro[2] * 0.10f;

    #ifdef gyroFilter
      cur_Gyro[0] = filterGX.updateEstimate(raw_gyro[0]);
      cur_Gyro[1] = filterGY.updateEstimate(raw_gyro[1]);
      cur_Gyro[2] = filterGZ.updateEstimate(raw_gyro[2]);
    #else
      cur_Gyro[0] = raw_gyro[0];
      cur_Gyro[1] = raw_gyro[1];
      cur_Gyro[2] = raw_gyro[2];
    #endif 

    // calculate angle
    float dt = ((float)(micros() - timer_gyro)) / 1000000.0f;
    if(dt < 0) dt = dt * -1.0f;
    angle[0] +=  cur_Gyro[0] * dt;
    angle[1] +=  cur_Gyro[1] * dt;
    angle[2] +=  cur_Gyro[2] * dt;

    angle[0] += angle[1] * sin(cur_Gyro[2] * dt * 0.017453292f);
    angle[1] -= angle[0] * sin(cur_Gyro[2] * dt * 0.017453292f);

    #ifdef angleFilter
      cur_Angle[0] = filterX.updateEstimate(angle[0]);
      cur_Angle[1] = filterY.updateEstimate(angle[1]);
       // 加不加滤波改不了z角度环异常
      float t;
      t = filterZ.updateEstimate(angle[2]);
      t = filterZ.updateEstimate(angle[2]);
      t = filterZ.updateEstimate(angle[2]);
      t = filterZ.updateEstimate(angle[2]);
      cur_Angle[2] = t;
    #else
      cur_Angle[0] = angle[0];
      cur_Angle[1] = angle[1];
      cur_Angle[2] = angle[2];
    #endif
    // 由于dmp输出比较稳，所以手动算角速度
    pre_Gyro[0] = (cur_Angle[0] - pre_Angle[0])/dt;
    pre_Gyro[1] = (cur_Angle[1] - pre_Angle[1])/dt;
    pre_Gyro[2] = (cur_Angle[2] - pre_Angle[2])/dt;
    for(int i =  0; i < 3; i++) pre_Angle[i] = cur_Angle[i];
    timer_gyro = micros(); // update timer

    // 可以忽略
    static int acc_count = 0;
    static float avg_acc[3] = {0, 0, 0};
    acc_count += 1;
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
    raw_acc[0] = ((float)rawData_acc[0]) / 16384.0f;
    raw_acc[1] = ((float)rawData_acc[1]) / 16384.0f;
    raw_acc[2] = ((float)rawData_acc[2]) / 16384.0f;

    // 1 = x^2 + y^2 + z^2
    float temp[3];
    temp[0] = -1 * sin(cur_Angle[1] * 0.0174532f);
    temp[1] = -1 * sin(cur_Angle[0] * 0.0174532f);
    temp[2] = sqrt(1 - temp[0] * temp[0] - temp[1] * temp[1]);

    if(acc_count <= 10) avg_acc[0] += raw_acc[0] - temp[0];
    if(acc_count <= 10) avg_acc[1] += raw_acc[1] - temp[1];
    if(acc_count <= 10) avg_acc[2] += raw_acc[2] - temp[2];
    if(acc_count == 10){
      cur_Acc[0] = avg_acc[0] / 10;
      cur_Acc[1] = avg_acc[1] / 10;
      cur_Acc[2] = avg_acc[2] / 10;
      cur_Acc_sum[0] += cur_Acc[0];
      cur_Acc_sum[1] += cur_Acc[1];
      cur_Acc_sum[2] += cur_Acc[2];
      avg_acc[0] = 0;
      avg_acc[1] = 0;
      avg_acc[2] = 0;
      acc_count = 0;      
    }

    timer_sample1 = micros();
  }


  if(millis() - timer_sample2 >= 10){

    fifoCount = mpu.getFIFOCount();
    if ((!fifoCount) || (fifoCount % packetSize)) mpu.resetFIFO();
    else{
      // get dmp
      while (fifoCount >= packetSize) {
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
      }

      // calculate
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(yrp, &q, &gravity);
      angle[0] = angle[0] * 0.25f + (-yrp[2] * 180.0f / PI) * 0.75f;
      angle[1] = angle[1] * 0.25f + (-yrp[1] * 180.0f / PI) * 0.75f;
      angle[2] = angle[2] * 0.05f + ( yrp[0] * 180.0f / PI) * 0.95f;

      // update timer
      float dt = ((float)(micros() - timer_gyro)) / 1000000.0f;
      if(dt < 0) dt = dt * -1.0f;

      #ifdef angleFilter
        cur_Angle[0] = filterX.updateEstimate(angle[0]);
        cur_Angle[1] = filterY.updateEstimate(angle[1]);
        // 加不加滤波改不了z角度环异常
        float t;
        t = filterZ.updateEstimate(angle[2]);
        t = filterZ.updateEstimate(angle[2]);
        t = filterZ.updateEstimate(angle[2]);
        t = filterZ.updateEstimate(angle[2]);
        cur_Angle[2] = t;
      #else
        cur_Angle[0] = angle[0];
        cur_Angle[1] = angle[1];
        cur_Angle[2] = angle[2];
      #endif
      // 由于dmp输出比较稳，所以手动算角速度
      pre_Gyro[0] = (cur_Angle[0] - pre_Angle[0])/dt;
      pre_Gyro[1] = (cur_Angle[1] - pre_Angle[1])/dt;
      pre_Gyro[2] = (cur_Angle[2] - pre_Angle[2])/dt;
      for(int i =  0; i < 3; i++) pre_Angle[i] = cur_Angle[i];
      timer_gyro = micros();
    }

  timer_sample2 = millis();
  }

}



#endif
