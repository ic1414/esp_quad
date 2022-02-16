/*
    B10ac A9 c
    \\   //
     \\ //
     // \\
    //   \\
   D5c    C6 ac
*/

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SimpleKalmanFilter.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <ESP32Servo.h>


// function
void task_nrf(void*);
void task_vtg(void*);
void task_main(void*);
void getAnglesV2();
float *getVoltages();
void motor_prop_balancer();
float pid(float, volatile float*, volatile float, float*,
          float, volatile float, volatile float, volatile float);


// task 
TaskHandle_t TaskHandle_mpu;
TaskHandle_t TaskHandle_nrf;
TaskHandle_t TaskHandle_vtg;
TaskHandle_t TaskHandle_ack;
TaskHandle_t TaskHandle_main;


// mpu
MPU6050 mpu;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float yrp[3];
volatile float gyroacc_offs[6];

float maxAngle = 65;
volatile float cAngles[3];
volatile float raw_cAngles[3];
volatile float setPointZ = 0;
volatile unsigned long mpu_update_flag_core0;
volatile unsigned long mpu_update_flag_core1;
// kalman filter for fun
SimpleKalmanFilter filterX(0.5f, 0.5f, 0.05f);
SimpleKalmanFilter filterY(0.5f, 0.5f, 0.05f);
SimpleKalmanFilter filterZ(0.5f, 0.5f, 0.05f);
//method 2
volatile double angleX;
volatile double angleY;
volatile double angleZ;
volatile double angleX_kal;
volatile double angleY_kal;
volatile double angleZ_kal;
volatile double angleAccX;
volatile double angleAccY;



//voltage
#define v3sPin 4
#define v2sPin 15
#define v1sPin 14
#define lowV 3.5*3
#define v_pre_weight  0.99
#define v_cur_weight  0.01
volatile float v1s;
volatile float v2s;
volatile float v3s;
volatile float vtg_set_cmp = 12.6;
volatile float motor_cmp_v = 0;
volatile float motor_cmp_pwm;


// radio
#define csn 25
#define ce 26
#define radioTimeOut 600
volatile unsigned long radio_update_flag_core0;
volatile unsigned long radio_update_flag_core1;
const byte addresses[][6] = {"00001", "00002"};
volatile byte defautX = 127;
volatile byte defautY = 127;
volatile byte defautZ = 127;
volatile byte defautT = 0;
RF24 radio(26, 25); // CE, CSN


// special
struct nrf {
  volatile byte pwm[4];
  volatile float rpid[3];
  volatile float rpidz[3];
  volatile byte en;
};
//send
struct drone_data {
  volatile float voltages[3];
  volatile byte motorOut[4];
  volatile float cAngles[3];
};
struct nrf radioo;
struct drone_data ack;


//pid
volatile float pError[3] = {0, 0, 0};
float integral[3] = {0, 0, 0};
volatile int maxPWMout = 300;// pwm in microseconds
volatile float max_propo = 300;
volatile float max_inter = 50;
volatile float max_deriv = 200;
volatile float Kp  = 1.8;
volatile float Ki  = 2.0; // /1000
volatile float Kd  = 70.0;
volatile float Kpz  = 4.5;
volatile float Kiz  = 3.0;
volatile float Kdz  = 20.0;
volatile float dt = 5;    // millisecond


//motors
ESP32PWM pwm;
#define motorA_pin 32
#define motorB_pin 33
#define motorC_pin 27
#define motorD_pin 13
Servo   motorA;
Servo   motorB;
Servo   motorC;
Servo   motorD;
volatile int motorPwm[4];


void setup() {
  Serial.begin(250000);

  //wire begin
  Wire.begin();
  Wire.setClock(2000000);
  // mpu begin
  mpu.initialize();
  mpu.setFullScaleGyroRange(1);
  mpu.setFullScaleAccelRange(2);
  // mpu.setRate(0);
  mpu.setDLPFMode(3);
  mpu.dmpInitialize();
  // auto calibrate
  Serial.println("calibrate mpu");
  delay(2000);
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);

  gyroacc_offs[0] = mpu.getXGyroOffset();
  gyroacc_offs[1] = mpu.getYGyroOffset();
  gyroacc_offs[2] = mpu.getZGyroOffset();
  gyroacc_offs[3] = mpu.getXAccelOffset();
  gyroacc_offs[4] = mpu.getYAccelOffset();
  gyroacc_offs[5] = mpu.getZAccelOffset();
/*
  gyroacc_offs[0] = 79;
  gyroacc_offs[1] = 46;
  gyroacc_offs[2] = -21;
  gyroacc_offs[3] = -1070;
  gyroacc_offs[4] = 2351;
  gyroacc_offs[5] = 298;
  */
  mpu.setXGyroOffset(gyroacc_offs[0]);
  mpu.setYGyroOffset(gyroacc_offs[1]);
  mpu.setZGyroOffset(gyroacc_offs[2]);
  mpu.setXAccelOffset(gyroacc_offs[3]);
  mpu.setYAccelOffset(gyroacc_offs[4]);
  mpu.setZAccelOffset(gyroacc_offs[5]);
  mpu.PrintActiveOffsets();
  // continue
  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();
  fifoCount = mpu.getFIFOCount();

  // task inni
  xTaskCreatePinnedToCore(task_nrf, "Task_nrf", 1000,
                          NULL, 8, &TaskHandle_nrf, 0);
  xTaskCreatePinnedToCore(task_vtg, "Task_vtg", 1000,
                          NULL, 5, &TaskHandle_vtg, 0);
  xTaskCreatePinnedToCore(task_ack, "Task_ack", 1000,
                          NULL, 5, &TaskHandle_ack, 0);
  xTaskCreatePinnedToCore(task_main, "Task_main", 1000,
                          NULL, 10, &TaskHandle_ack, 0);

}


void loop() {
  static int count = 0;
  static unsigned long timer = millis();

  while (fifoCount < packetSize) {
    getAnglesV2();
    count += 1;
    mpu_update_flag_core1 += 1;
    fifoCount = mpu.getFIFOCount();
    if (millis() - timer > 10)
    {
      Serial.print(cAngles[0]);
      Serial.print("   ");
      Serial.print(cAngles[1]);
      Serial.print("   ");
      Serial.print(cAngles[2]);
      Serial.print("   ");
      Serial.println(count);
      timer = millis();
      count = 0;
    }

  }
  // get fifo value and fuse with c filter val
  if ((fifoCount == 1024) or (fifoCount % packetSize != 0)) {
    mpu.resetFIFO();
  } else {
    while (fifoCount >= packetSize) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
    }
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(yrp, &q, &gravity);
    angleX = angleX * 0.2 + ( yrp[2] * 180 / PI) * 0.8;
    angleY = angleY * 0.2 + (-yrp[1] * 180 / PI) * 0.8;
    angleZ = yrp[0] * 180 / PI;
    count += 1;
  }
}


// higher rate
void getAnglesV2() {
  static int count_gyro = 0;
  static unsigned long timer_gyro = micros();

  // gyro
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, (uint8_t) 6);
  double raw_gyro[3];
  int16_t rawData_gyro[3];
  for (int i = 0; i < 3; i++) {
    rawData_gyro[i]  = Wire.read() << 8;
    rawData_gyro[i] |= Wire.read();
  }
  raw_gyro[0] = (((double)rawData_gyro[0]) - gyroacc_offs[0]) / 65.5;
  raw_gyro[1] = (((double)rawData_gyro[1]) - gyroacc_offs[1]) / 65.5;
  raw_gyro[2] = (((double)rawData_gyro[2]) - gyroacc_offs[2]) / 65.5;

  count_gyro += 1;
  if (count_gyro != 8) {
    // calculate angle
    double dt = ((double)(micros() - timer_gyro)) / 1000000;
    angleX +=  raw_gyro[0] * dt;
    angleY +=  raw_gyro[1] * dt;
    // angleZ +=  raw_gyro[2] * dt;
    angleX -= angleY * sin(raw_gyro[2] * dt * 0.01745);
    angleY += angleX * sin(raw_gyro[2] * dt * 0.01745);
    timer_gyro = micros();
  }

  // acc
  if (count_gyro == 8) {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, (uint8_t) 6);
    double raw_acc[3];
    int16_t rawData_acc[3];
    for (int i = 0; i < 3; i++) {
      rawData_acc[i]  = Wire.read() << 8;
      rawData_acc[i] |= Wire.read();
    }
    raw_acc[0] = (((double)rawData_acc[0]) - gyroacc_offs[3]) / 4096.0;
    raw_acc[1] = (((double)rawData_acc[1]) - gyroacc_offs[4]) / 4096.0;
    raw_acc[2] = (((double)rawData_acc[2]) - gyroacc_offs[5]) / 4096.0;


    // calculate angle
    double dt = ((double)(micros() - timer_gyro)) / 1000000;
    angleX +=  raw_gyro[0] * dt;
    angleY +=  raw_gyro[1] * dt;
    // angleZ +=  raw_gyro[2] * dt;
    angleX -= angleY * sin(raw_gyro[2] * dt * 0.01745);
    angleY += angleX * sin(raw_gyro[2] * dt * 0.01745);

    double acc_total_vector = sqrt((raw_acc[0] * raw_acc[0]) +
                                   (raw_acc[1] * raw_acc[1]) +
                                   (raw_acc[2] * raw_acc[2]));
    if (abs(raw_acc[1]) < acc_total_vector) {
      angleAccX = asin((double)raw_acc[1] / acc_total_vector) * 57.296;
    }
    if (abs(raw_acc[0]) < acc_total_vector) {
      angleAccY = asin((double)raw_acc[0] / acc_total_vector) * -57.296;
    }

    angleX = angleX * 0.99 + angleAccX * 0.01;
    angleY = angleY * 0.99 + angleAccY * 0.01;

    count_gyro = 0;
    timer_gyro = micros();
  }
  // kalman for fun
  cAngles[0] = filterX.updateEstimate(-angleX);
  cAngles[1] = filterY.updateEstimate( angleY);
  cAngles[2] = filterZ.updateEstimate( angleZ);
}


// radio
void task_nrf( void * pvParameters ) {
  //initiate radio
  radio.begin();
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.openReadingPipe(1, addresses[1]); // address global
  radio.openWritingPipe(addresses[0]); // 00001
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  radio.writeAckPayload(1, &ack, sizeof(ack));
  unsigned long radio_timer = millis();
  // loop forever
  for (;;) {
    // recieve radio
    if (radio.available()) {
      radio.read(&radioo, sizeof(radioo));
      if (not(radioo.rpid[0] +
              radioo.rpid[1] + radioo.rpid[2] == 0)) {
        Kp = radioo.rpid[0];
        Ki = radioo.rpid[1];
        Kd = radioo.rpid[2];
        Kpz = radioo.rpidz[0];
        Kiz = radioo.rpidz[1];
        Kdz = radioo.rpidz[2];
      }
      radio.writeAckPayload(1, &ack, sizeof(ack));
      radio_timer = millis();
      radio_update_flag_core0 += 1;
    } else {
      // radio time out
      if (millis() - radio_timer > radioTimeOut) {
        radioo.pwm[0] = defautX;
        radioo.pwm[1] = defautY;
        radioo.pwm[2] = defautZ;
        radioo.pwm[3] = defautT;
        radioo.en = 0;
      }
    }
    vTaskDelay(5);
  }
}
// update ack data
void task_ack( void * pvParameters ) {
  for (;;) {
    motorPwm[0] = constrain(motorPwm[0], 1000, 2000);
    motorPwm[1] = constrain(motorPwm[1], 1000, 2000);
    motorPwm[2] = constrain(motorPwm[2], 1000, 2000);
    motorPwm[3] = constrain(motorPwm[3], 1000, 2000);
    ack.voltages[0] = v1s;
    ack.voltages[1] = v2s;
    ack.voltages[2] = v3s;
    ack.cAngles[0] = cAngles[0];
    ack.cAngles[1] = cAngles[1];
    ack.cAngles[2] = cAngles[2];
    ack.motorOut[0] = map(motorPwm[0], 1000, 2000, 0, 100);
    ack.motorOut[1] = map(motorPwm[1], 1000, 2000, 0, 100);
    ack.motorOut[2] = map(motorPwm[2], 1000, 2000, 0, 100);
    ack.motorOut[3] = map(motorPwm[3], 1000, 2000, 0, 100);
    vTaskDelay(10);
  }
}


// vtg
void task_vtg( void * pvParameters ) {
  pinMode(v1sPin, INPUT);
  pinMode(v2sPin, INPUT);
  pinMode(v3sPin, INPUT);
  //check voltage
  for (;;) {
    float *voltages = getVoltages();
    v1s = v1s * v_pre_weight + (*(voltages + 0)) * v_cur_weight;
    v2s = v2s * v_pre_weight + (*(voltages + 1)) * v_cur_weight;
    v3s = v3s * v_pre_weight + (*(voltages + 2)) * v_cur_weight;
    vTaskDelay(10);
  }
}
// get voltages
float * getVoltages() {
  static float v[3] = {0, 0, 0};
  v[0] = analogRead(v1sPin);
  v[1] = analogRead(v2sPin);
  v[2] = analogRead(v3sPin);
  for (int i = 0; i < 3; i++) {
    v[i] = v[i] / 4095 * 3.3;
  }
  v[0] = v[0] * (9.4 + 0.0) / 4.7 + 0.2;
  v[1] = v[1] * (14.7 + 0.0) / 4.7 + 0.4; //linear error ~0.70v
  v[2] = v[2] * (9.0 + 0.0) / 2.2 + 0.6; // linear error ~0.85v
  return v;
}


// pid
float pid(float set_point, volatile float * pError, volatile float measure,
          float * integral, float dt, volatile float Kp, volatile float Ki, volatile float Kd) {
  // error
  float cError = set_point - measure;

  // propotion
  float propotion = cError * Kp;
  propotion = constrain(propotion, -max_propo, max_propo);

  // intergral
  *integral = *integral + cError / 1000;
  *integral = constrain(*integral, -max_inter, max_inter);
  float inter_out = *integral * Ki;

  // derivative
  float derivative = (cError - *pError) * Kd;
  derivative = constrain(derivative, -max_deriv, max_deriv);
  *pError = cError;

  // out
  float out = (propotion + inter_out + derivative);
  return out;
}

// untitled
void motor_prop_balancer() {
  if (Serial.available() > 0) {
    char str  = Serial.read();
    float pre_angle[2];
    float pre_change[2];
    float cur_change[2];
    float change_change[2];
    float max_change[2];
    if (str >= 'a' and str <= 'd')while (true) {
        int tpwm = 1000 + radioo.pwm[3] * 3.9;
        constrain(tpwm, 1000, 2000);
        switch (str) {
          case 'a':
            motorA.writeMicroseconds(tpwm);
            break;
          case 'b':
            motorB.writeMicroseconds(tpwm);
            break;
          case 'c':
            motorC.writeMicroseconds(tpwm);
            break;
          case 'd':
            motorD.writeMicroseconds(tpwm);
            break;
        }
        cur_change[0] = cAngles[0] - pre_angle[0];
        cur_change[1] = cAngles[1] - pre_angle[1];
        if (cur_change[0] > pre_change[0])max_change[0] = cur_change[0];
        if (cur_change[1] > pre_change[1])max_change[1] = cur_change[1];
        pre_angle[0] = cAngles[0];
        pre_angle[1] = cAngles[1];
        pre_change[0] = cur_change[0];
        pre_change[1] = cur_change[1];
        //Serial.print(" X:");
        //Serial.print(cAngles[0], 3);
        //Serial.print(" Y:");
        //Serial.print(cAngles[1], 3);
        Serial.print("   cur_changeX: ");
        Serial.print(cur_change[0], 3);
        //Serial.print("   cur_changeY: ");
        //Serial.print(cur_change[1], 3);
        //Serial.print("max_changeX:");
        //Serial.print(max_change[0], 3);
        //Serial.print("max_changeY:");
        //Serial.print(max_change[1], 3);
        Serial.println("   ");
        vTaskDelay(5);
      }
  }
}
// main
void task_main( void * pvParameters ) {
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

  // start when mpu started
  while (mpu_update_flag_core1 == mpu_update_flag_core0) vTaskDelay(4);
  // main loop
  for (;;) {
    unsigned long gtimer = millis();
    unsigned long gtimer2 = micros();
    motor_prop_balancer();

    //convert joystick to set angle
    float setAngles[3];
    for (int i = 0; i < 2; i++)setAngles[i] =
        map(radioo.pwm[i], 0, 255, -30, 30);
    setAngles[2] = map(radioo.pwm[2], 0, 255, -60, 60);;

    // if z moved, or move z manualy
    boolean zMove = false;
    boolean zRadio = false;
    if (cAngles[2] - setPointZ > 1.0
        or cAngles[2] - setPointZ < -1.0)zMove = true;
    if (radioo.pwm[2] > 135 or radioo.pwm[2] < 119)zRadio = true;
    

    // pid
    float tPwm = (float)radioo.pwm[3] * 3.9;
    float pid_out_pwm[4] = {0, 0, 0, tPwm};
    pid_out_pwm[0] = pid(setAngles[0], &pError[0], cAngles[0],
                         &integral[0], dt, Kp, Ki, Kd);
    pid_out_pwm[1] = pid(setAngles[1], &pError[1], cAngles[1],
                         &integral[1], dt, Kp, Ki, Kd);
    // if need pid z when not moving manualy
    // updata z if move manualy
    if (zRadio == false and zMove == true) {
      setAngles[2] = setPointZ;
      pid_out_pwm[2] = pid(setAngles[2], &pError[2], cAngles[2],
                           &integral[2], dt, Kpz, Kiz, Kdz);
    }
    if (zRadio == true) {
      setPointZ = cAngles[2];
      setAngles[2] = cAngles[2] + setAngles[2];
      pid_out_pwm[2] = pid(setAngles[2], &pError[2], cAngles[2],
                           &integral[2], dt, Kpz, Kiz, Kdz);
    }

    // calculate pwm out
    int motorPwm_zip[4];
    motorPwm_zip[0] = 0 - pid_out_pwm[0] - pid_out_pwm[1] - pid_out_pwm[2];
    motorPwm_zip[1] = 0 - pid_out_pwm[0] + pid_out_pwm[1] + pid_out_pwm[2];
    motorPwm_zip[2] = 0 + pid_out_pwm[0] - pid_out_pwm[1] + pid_out_pwm[2];
    motorPwm_zip[3] = 0 + pid_out_pwm[0] + pid_out_pwm[1] - pid_out_pwm[2];
    for (int i = 0; i < 4; i++)motorPwm_zip[i] = constrain(motorPwm_zip[i], -maxPWMout, maxPWMout);
    // motorPwm
    // 1000 is the min of pwm, out_pwm[3] is t
    for (int i = 0; i < 4; i++)motorPwm[i] = 1000 + pid_out_pwm[3] + motorPwm_zip[i];

    // stop when t is 0
    // stop when angle over 70.
    boolean motorStop = false;
    boolean overAngle = false;
    if (radioo.pwm[3] == defautT or radioo.en == 0) motorStop = true;
    if (abs(cAngles[0]) > abs(maxAngle) or
        abs(cAngles[1]) > abs(maxAngle)) overAngle = true;

    // vtg compensation
    if (not(overAngle) and not(motorStop)) {
      // compensate voltage drop
      if (v3s > 10.0)motor_cmp_v = vtg_set_cmp - v3s;
      else motor_cmp_v = 0;
      motor_cmp_pwm += motor_cmp_v / (vtg_set_cmp * 50);
      motor_cmp_pwm = constrain(motor_cmp_pwm, 0, 100);
      //for (int i = 0; i < 4; i++) motorPwm[i] += motor_cmp_pwm;
    }

    // motor write
    if (not(overAngle) and not(motorStop)) {
      motorPwm[0] = constrain(motorPwm[0], 1000, 2000);
      motorPwm[1] = constrain(motorPwm[1], 1000, 2000);
      motorPwm[2] = constrain(motorPwm[2], 1000, 2000);
      motorPwm[3] = constrain(motorPwm[3], 1000, 2000);
      motorA.writeMicroseconds(motorPwm[0]);
      motorB.writeMicroseconds(motorPwm[1]);
      motorC.writeMicroseconds(motorPwm[2]);
      motorD.writeMicroseconds(motorPwm[3]);
    } else {
      // stop
      integral[0] = 0;
      integral[1] = 0;
      integral[2] = 0;
      motorA.writeMicroseconds(1000);
      motorB.writeMicroseconds(1000);
      motorC.writeMicroseconds(1000);
      motorD.writeMicroseconds(1000);
      //for (int i = 0; i < 4; i++)motorPwm[i] = 1000;
    }
    long delay_time = 1000 - (micros() - gtimer2);
    delayMicroseconds(delay_time);
    vTaskDelay(4);
    //Serial.println(micros() - gtimer2);
  }
}


/*
    Serial.print(cAngles[0]);
    Serial.print("   ");
    Serial.print(cAngles[1]);
    Serial.print("   ");
    Serial.println(cAngles[2]);
*/
