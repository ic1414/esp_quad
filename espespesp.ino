/*
    B10ac A9 c
    \\   //
     \\ //
     // \\
    //   \\
   D5c    C6 ac
*/

#include <Wire.h>
#include <MPU6050_light.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <ESP32Servo.h>
#include <SimpleKalmanFilter.h>

// function
void task_mpu(void*);
void task_nrf(void*);
void task_vtg(void*);
float *getAngles();
float *getVoltages();
float pid(float, volatile float*, volatile float, float*,
          float, volatile float, volatile float, volatile float, char);
void motor_prop_balancer();


// task
TaskHandle_t TaskHandle_mpu;
TaskHandle_t TaskHandle_nrf;
TaskHandle_t TaskHandle_vtg;
TaskHandle_t TaskHandle_ack;


// kalman filter
SimpleKalmanFilter filterX(0.25, 0.25, 0.04);
SimpleKalmanFilter filterY(0.25, 0.25, 0.04);
SimpleKalmanFilter filterZ(0.25, 0.25, 0.04);


// mpu
MPU6050 mpu(Wire);
float maxAngle = 70;
volatile float cAngles[3];
volatile float pAngles[3];
volatile float setPointZ;
volatile unsigned long mpu_update_flag_core0;
volatile unsigned long mpu_update_flag_core1;


//voltage
#define v3sPin 4
#define v2sPin 15
#define v1sPin 14
#define lowV 3.5*3
volatile float v1s;
volatile float v2s;
volatile float v3s;
volatile float vtg_set_cmp = 12.6;
volatile float motor_cmp_v = 0;
volatile float motor_cmp_pwm;
volatile float v_pre_weight = 0.95;
volatile float v_cur_weight = 0.05;


// radio
#define csn 25
#define ce 26
#define radioTimeOut 1000
volatile unsigned long radio_timer;
volatile unsigned long radio_update_flag_core0;
volatile unsigned long radio_update_flag_core1;
const byte addresses[][6] = {"00001", "00002"};
volatile byte defautX = 127;
volatile byte defautY = 127;
volatile byte defautZ = 127;
volatile byte defautT = 0;
RF24 radio(26, 25); // CE, CSN
// recieve
struct nrf {
  volatile byte pwm[4];
  volatile float rpid[3];
  volatile float rpidz[3];
  volatile byte i_clear;
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
volatile float set_joyStick_angle[3];
volatile float out_joyStick_angle[3];
volatile float joyStick_angle_lim = 10.0;
volatile float pError[3] = {0, 0, 0};
float integral[3] = {0, 0, 0};
volatile int maxPWMout = 100;// pwm in microseconds
volatile float max_propo = 30;
volatile float max_inter = 30;
volatile float max_deriv = 40;
volatile float Kp  = 3.5;
volatile float Ki  = 0.0; // /1000
volatile float Kd  = 180;
volatile float Kpz  = 5.0;
volatile float Kiz  = 0.0;
volatile float Kdz  = 0.0;
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
const int minUs = 1000;
const int maxUs = 2000;


void setup() {
  Serial.begin(250000);

  int pwm_fre = 400;
  //initiate motors
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  motorA.setPeriodHertz(pwm_fre);
  motorB.setPeriodHertz(pwm_fre);
  motorC.setPeriodHertz(pwm_fre);
  motorD.setPeriodHertz(pwm_fre);
  motorA.attach(motorA_pin, minUs, maxUs);
  motorB.attach(motorB_pin, minUs, maxUs);
  motorC.attach(motorC_pin, minUs, maxUs);
  motorD.attach(motorD_pin, minUs, maxUs);


  xTaskCreatePinnedToCore(task_mpu, "Task_mpu", 1000,
                          NULL, 10, &TaskHandle_mpu, 0);
  xTaskCreatePinnedToCore(task_nrf, "Task_nrf", 1000,
                          NULL, 10, &TaskHandle_nrf, 0);
  xTaskCreatePinnedToCore(task_vtg, "Task_vtg", 1000,
                          NULL, 6, &TaskHandle_vtg, 0);
  xTaskCreatePinnedToCore(task_ack, "Task_ack", 1000,
                          NULL, 6, &TaskHandle_ack, 0);
}

void loop() {
  unsigned long gtimer = millis();
  unsigned long gtimer2 = micros();
  motor_prop_balancer();


  //convert joystick to set angle
  float setAngles[3];
  for (int i = 0; i < 3; i++) {
    float absolute = joyStick_angle_lim * 2;
    float lower = -joyStick_angle_lim;
    float x = radioo.pwm[i];
    // smooth angle change
    set_joyStick_angle[i] = (x * (absolute / 255.0)) + lower;
    out_joyStick_angle[i] = out_joyStick_angle[i] * 0.9 + set_joyStick_angle[i] * 0.1;
    setAngles[i] = out_joyStick_angle[i];
    //setAngles[i] = map(radioo.pwm[i], 0, 255, -15, 15);
  }

  // if z moved, or move z manualy
  boolean zMove = false;
  boolean zRadio = false;
  if (cAngles[2] - setPointZ > 0.5
      or cAngles[2] - setPointZ < -0.5)zMove = true;
  if (radioo.pwm[2] != 127)zRadio = true;

  // clear integration
  if (radioo.i_clear == 1) integral[0] = 0; integral[1] = 0; integral[2] = 0;

  // pid
  float tPwm = (float)radioo.pwm[3] * 3.9;
  float pid_out_pwm[4] = {0, 0, 0, tPwm};
  pid_out_pwm[0] = pid(setAngles[0], &pAngles[0], cAngles[0],
                       &integral[0], dt, Kp, Ki, Kd, 'x');
  pid_out_pwm[1] = pid(setAngles[1], &pAngles[1], cAngles[1],
                       &integral[1], dt, Kp, Ki, Kd, 'y');
  // if need calibrate z when not moving manualy
  // updata z if move manualy
  if (zRadio == false and zMove == true) {
    setAngles[2] = setPointZ;
    pid_out_pwm[2] = pid(setAngles[2], &pAngles[2], cAngles[2],
                         &integral[2], dt, Kpz, Kiz, Kdz, 'z');
  }
  if (zRadio == true) {
    setPointZ = cAngles[2];
    setAngles[2] = cAngles[2] + setAngles[2];
    pid_out_pwm[2] = pid(setAngles[2], &pAngles[2], cAngles[2],
                         &integral[2], dt, Kpz, Kiz, Kdz, 'z');
  }

  // calculate pwm out
  // motorPwm is global
  motorPwm[0] = 1000 + pid_out_pwm[3] - pid_out_pwm[0] - pid_out_pwm[1] - pid_out_pwm[2];
  motorPwm[1] = 1000 + pid_out_pwm[3] - pid_out_pwm[0] + pid_out_pwm[1] + pid_out_pwm[2];
  motorPwm[2] = 1000 + pid_out_pwm[3] + pid_out_pwm[0] - pid_out_pwm[1] + pid_out_pwm[2];
  motorPwm[3] = 1000 + pid_out_pwm[3] + pid_out_pwm[0] + pid_out_pwm[1] - pid_out_pwm[2];


  // if need stop motors
  // stop when t is 0
  // stop when angle over 80.
  boolean motorStop = false;
  boolean overAngle = false;
  if (radioo.pwm[3] == defautT) motorStop = true;
  if (cAngles[0] > maxAngle or cAngles[0] < -maxAngle or
      cAngles[1] > maxAngle or cAngles[1] < -maxAngle) overAngle = true;


  // vtg compensation
  if (not(overAngle) and not(motorStop)) {
    // compensate voltage drop
    if (v3s > 10.0)motor_cmp_v = vtg_set_cmp - v3s;
    else motor_cmp_v = 0;
    motor_cmp_pwm += motor_cmp_v / (vtg_set_cmp * 50);
    motor_cmp_pwm = constrain(motor_cmp_pwm, 0, 150);
    //for (int i = 0; i < 4; i++) motorPwm[i] += motor_cmp_pwm;
  }



  // motor write
  if (not(overAngle) and not(motorStop)) {
    // limit pwm
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
    motorA.writeMicroseconds(1000);
    motorB.writeMicroseconds(1000);
    motorC.writeMicroseconds(1000);
    motorD.writeMicroseconds(1000);
  }

  Serial.print(motorPwm[0]);
  Serial.print("   ");
  Serial.print(motorPwm[1]);
  Serial.print("   ");
  Serial.print(motorPwm[2]);
  Serial.print("   ");
  Serial.print(motorPwm[3]);
  Serial.println("   ");

  // maintain 3ms dt
  if ((micros() - gtimer2) < dt * 1000) {
    delayMicroseconds(dt * 1000 - (micros() - gtimer2));
  }
  //Serial.println(micros() - gtimer2);
}



void task_mpu( void * pvParameters ) {
  //initiate mpu
  Wire.begin();
  byte status = mpu.begin(1, 1);
  mpu.setFilterGyroCoef(0.995);
  mpu.setFilterAccCoef(0.005);
  while (status != 0) { }
  delay(2000);
  mpu.calcOffsets(true, true);
  // get past angles
  float *p = getAngles();
  setPointZ = pAngles[2];
  for (int i = 0; i < 3; i++)pAngles[i] = *(p + i);
  vTaskDelay(1);

  //sample current angle for ever
  for (;;) {
    float *pp = getAngles();
    for (int i = 0; i < 3; i++) cAngles[i] = *(pp + i);
    mpu_update_flag_core0 += 1;
    vTaskDelay(1);
  }
}
//get angles
float * getAngles() {
  mpu.update();
  static float angles[3];
  angles[0] = mpu.getAngleX();
  angles[1] = mpu.getAngleY();
  angles[2] = mpu.getAngleZ();
  // revers particular angles
  angles[0] = -(angles[0]);
  angles[2] = -(angles[2]);
  // filter
  angles[0] = filterX.updateEstimate(angles[0]);
  angles[1] = filterY.updateEstimate(angles[1]);
  angles[2] = filterZ.updateEstimate(angles[2]);
  return angles;
}


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
  radio_timer = millis();
  // loop forever
  for (;;) {
    // recieve radio
    if (radio.available()) {
      radio.read(&radioo, sizeof(radioo));
      if (not(radioo.rpid[0] + radioo.rpid[1] + radioo.rpid[2] == 0)) {
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
      }
    }
    vTaskDelay(5);
  }
}


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
    vTaskDelay(5);
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
  v[0] = v[0] * (9.4 + 0.0) / 4.7;
  v[1] = v[1] * (14.7 + 0.0) / 4.7; //linear error ~0.70v
  v[2] = v[2] * (9.0 + 0.0) / 2.2; // linear error ~0.85v
  return v;
}


void task_ack( void * pvParameters ) {
  //prepare ack data
  for (;;) {
    ack.voltages[0] = v1s;
    ack.voltages[1] = v2s;
    ack.voltages[2] = v3s;
    ack.cAngles[0] = cAngles[0];
    ack.cAngles[1] = cAngles[1];
    ack.cAngles[2] = cAngles[2];
    motorPwm[0] = constrain(motorPwm[0], 1000, 2000);
    motorPwm[1] = constrain(motorPwm[1], 1000, 2000);
    motorPwm[2] = constrain(motorPwm[2], 1000, 2000);
    motorPwm[3] = constrain(motorPwm[3], 1000, 2000);
    ack.motorOut[0] = map(motorPwm[0], 1000, 2000, 0, 255);
    ack.motorOut[1] = map(motorPwm[1], 1000, 2000, 0, 255);
    ack.motorOut[2] = map(motorPwm[2], 1000, 2000, 0, 255);
    ack.motorOut[3] = map(motorPwm[3], 1000, 2000, 0, 255);
    vTaskDelay(5);
  }
}


// pid
float pid(float set_point, volatile float * pre_measure, volatile float measure,
          float * integral, float dt, volatile float Kp, volatile float Ki, volatile float Kd, char c) {

  float cError = set_point - measure;
  float propotion = cError * Kp;
  if(c != 'z')propotion = constrain(propotion, -max_propo, max_propo);

  *integral = *integral + cError * dt / 1000;
  *integral = constrain(*integral, -max_inter, max_inter);
  float inter_out = *integral * Ki;

  float cErrorD = *pre_measure - measure;
  float derivative = ((cErrorD) / dt) * Kd;
  derivative = constrain(derivative, -max_deriv, max_deriv);
  *pre_measure = measure;

  float out = (propotion + inter_out + derivative);
  out = constrain(out, -maxPWMout, maxPWMout);
  return out;
}


void motor_prop_balancer() {
  if (Serial.available() > 0) {
    char str  = Serial.read();
    float pre_angle[2];
    // float cur_angle[2];
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
        change_change[0] = cur_change[0] - pre_change[0];
        change_change[1] = cur_change[1] - pre_change[1];
        if (cur_change[0] > pre_change[0])max_change[0] = cur_change[0];
        if (cur_change[1] > pre_change[1])max_change[1] = cur_change[1];
        pre_angle[0] = cAngles[0];
        pre_angle[1] = cAngles[1];
        pre_change[0] = cur_change[0];
        pre_change[1] = cur_change[1];
        //Serial.print(str);
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
        //Serial.print("change_changeX:");
        //Serial.print(change_change[0], 3);
        //Serial.print("change_changeY:");
        //Serial.print(change_change[1], 3);
        Serial.println("   ");
        delay(3);
      }
  }
}
