/*
    B10(ac) A9(c)
    \\   //
     \\ //
     // \\
    //   \\
   D5(c)    C6 (ac)
*/

#include "inclu.h"


// task function
void task_nrf(void*);
void task_vtg(void*);
void task_main(void*);
void task_motor(void*);
void task_serial(void*);
TaskHandle_t TaskHandle_nrf;
TaskHandle_t TaskHandle_vtg;
TaskHandle_t TaskHandle_main;
TaskHandle_t TaskHandle_motor;
TaskHandle_t TaskHandle_serial;

volatile long task_nrf_timecount = 0;
volatile long task_vtg_timecount = 0;
volatile long task_main_timecount = 0;
volatile long task_motor_timecount = 0;
volatile long task_serial_timecount = 0;

// mpu
MPU imu;


//voltage
VTG voltage;


// nrf
NRF nrf;
#define radioTimeOut 600


//motors
volatile int motorPwm[4];
volatile int motorPwmWrite[4] = {1000, 1000, 1000, 1000};


//pid
#define dt 5 // milli
#define maxPWMout 300
#define maxAngle 70.0
volatile float setPointZ = 0; // initial position of z
PID pidX(0.9f, 1.5f, 40.0f, 250, 100, 200);
PID pidY(0.9f, 1.5f, 40.0f, 250, 100, 200);
PID pidZ(4.5f, 0.0f, 0.00f, 250, 100, 200);
// did not test gyro pid
PID pidGX(0.9f, 1.5f, 40.0f, 250, 100, 200);
PID pidGY(0.9f, 1.5f, 40.0f, 250, 100, 200);
PID pidGZ(4.5f, 0.0f, 0.00f, 250, 100, 200);


void setup() {
  // hi
  Serial.begin(250000);
  
  // begins
  imu.begin();
  nrf.begin();
  motor_begin();
  voltage.begin();

  // task inni
  xTaskCreatePinnedToCore(task_nrf, "Task_nrf", 1000,
                          NULL, 8, &TaskHandle_nrf, 0);
  xTaskCreatePinnedToCore(task_vtg, "Task_vtg", 1000,
                          NULL, 5, &TaskHandle_vtg, 0);
  xTaskCreatePinnedToCore(task_main, "Task_main", 1000,
                          NULL, 10, &TaskHandle_main, 0);
  xTaskCreatePinnedToCore(task_serial, "Task_serial", 1000,
                          NULL, 7, &TaskHandle_serial, 0);
  xTaskCreatePinnedToCore(task_motor, "Task_motor", 1000,
                          NULL, 10, &TaskHandle_motor, 0);

}


void loop() {
  // keep as less as possible codes in the void loop()
  // otherwise will decrease sample rate
  imu.update();

}


// main
void task_main( void * pvParameters ) {

  // start when mpu started
  while (imu.cur_Angle[0] == 0) vTaskDelay(4);
  
  // main loop
  for (;;) {
    unsigned long gtimer = micros();

    //convert joystick to set angle
    float setAngles[3];
    for (int i = 0; i < 2; i++)setAngles[i] =
        map(nrf.radioo.pwm[i], 0, 255, -30, 30);
    setAngles[2] = map(nrf.radioo.pwm[2], 0, 255, -60, 60);;

    // if z moved, or if move z manualy
    boolean zMove = false;
    boolean zRadio = false;
    if (imu.cur_Angle[2] - setPointZ > 1.0
        or imu.cur_Angle[2] - setPointZ < -1.0)zMove = true;
    if (nrf.radioo.pwm[2] > 135 or nrf.radioo.pwm[2] < 119)zRadio = true;

    // pid
    int tPwm = (int)nrf.radioo.pwm[3] * 4 - 20;
    int pid_out_pwm[4] = {0, 0, 0, tPwm};
    int pid_out_agl[3] = {0, 0, 0};
    int pid_out_gro[3] = {0, 0, 0};
    // calculate
    pid_out_agl[0] = pidX._calculate(setAngles[0], imu.cur_Angle[0], dt);
    pid_out_agl[1] = pidY._calculate(setAngles[1], imu.cur_Angle[1], dt);
    // updata z if move manualy
    if (zRadio == true) {
      setPointZ = imu.cur_Angle[2];
      setAngles[2] = imu.cur_Angle[2] + setAngles[2];
      pid_out_agl[2] = pidZ._calculate(setAngles[2], imu.cur_Angle[2], dt);
    }
    // if need pid z when not moving manualy
    if (zRadio == false and zMove == true) {
      setAngles[2] = setPointZ;
      pid_out_agl[2] = pidZ._calculate(setAngles[2], imu.cur_Angle[2], dt);
    }
    /*
    pid_out_gro[0] = pidGX._calculate(pid_out_agl[0], imu.cur_Gyro[0], dt);
    pid_out_gro[1] = pidGY._calculate(pid_out_agl[1], imu.cur_Gyro[1], dt);
    pid_out_gro[2] = pidGZ._calculate(pid_out_agl[2], imu.cur_Gyro[2], dt);
    */

    for(int i=0;i<3;i++) pid_out_pwm[i] = pid_out_agl[i];
    //for(int i=0;i<3;i++) pid_out_pwm[i] = pid_out_gro[i];

    // add outputs
    int motorPwm_zip[4];
    motorPwm_zip[0] = 0 - pid_out_pwm[0] - pid_out_pwm[1] - pid_out_pwm[2];
    motorPwm_zip[1] = 0 - pid_out_pwm[0] + pid_out_pwm[1] + pid_out_pwm[2];
    motorPwm_zip[2] = 0 + pid_out_pwm[0] - pid_out_pwm[1] + pid_out_pwm[2];
    motorPwm_zip[3] = 0 + pid_out_pwm[0] + pid_out_pwm[1] - pid_out_pwm[2];
    for (int i = 0; i < 4; i++)motorPwm_zip[i] = constrain(motorPwm_zip[i], -maxPWMout, maxPWMout);
  
    // 1000 is the min for pwm. out_pwm[3] is t
    for (int i = 0; i < 4; i++)motorPwm[i] = 1000 + pid_out_pwm[3] + motorPwm_zip[i];

    // stop when t is 0
    // stop when angle over 70.
    boolean motorStop = false;
    boolean overAngle = false;
    if (nrf.radioo.pwm[3] == 0 or nrf.radioo.en == 0) motorStop = true;
    if (abs(imu.cur_Angle[0]) > abs(maxAngle) or
        abs(imu.cur_Angle[1]) > abs(maxAngle))  overAngle = true;

    // motor write
    if (not(overAngle) and not(motorStop)) {
      motorPwm[0] = constrain(motorPwm[0], 1000, 2000);
      motorPwm[1] = constrain(motorPwm[1], 1000, 2000);
      motorPwm[2] = constrain(motorPwm[2], 1000, 2000);
      motorPwm[3] = constrain(motorPwm[3], 1000, 2000);
      motorPwmWrite[0] = motorPwm[0];
      motorPwmWrite[1] = motorPwm[1];
      motorPwmWrite[2] = motorPwm[2];
      motorPwmWrite[3] = motorPwm[3];
    } else {
      // stop
      pidX.integral_reset();
      pidY.integral_reset();
      pidZ.integral_reset();
      motorPwmWrite[0] = 1000;
      motorPwmWrite[1] = 1000;
      motorPwmWrite[2] = 1000;
      motorPwmWrite[3] = 1000;
    }

    int delay_time = 1000 - (micros() - gtimer);
    task_main_timecount = 1000 - delay_time;
    delayMicroseconds(delay_time);
    vTaskDelay(dt-1);
  
  }
}


// vtg
void task_vtg( void * pvParameters ) {
  for (;;) {
    unsigned long gtimer = micros();

    //check voltage
    voltage.update();

    task_vtg_timecount = micros() - gtimer;
    vTaskDelay(100);
  
  }
}


void task_motor( void * pvParameters ) {
  for (;;) {
    unsigned long gtimer = micros();
    // update motor pwm
    motorA.writeMicroseconds(motorPwmWrite[0]);
    motorB.writeMicroseconds(motorPwmWrite[1]);
    motorC.writeMicroseconds(motorPwmWrite[2]);
    motorD.writeMicroseconds(motorPwmWrite[3]);
    task_motor_timecount = micros() - gtimer;
    vTaskDelay(3);
  }

}


// radio
void task_nrf( void * pvParameters ) {
  // loop forever
  for (;;) {
    unsigned long gtimer = micros();
    static unsigned long radio_timer = millis();
    
    if (nrf.available()) {
      nrf.read(); // read
      if (not(nrf.radioo.pid[0] + nrf.radioo.pid[1] + nrf.radioo.pid[2] == 0)){
        pidX.set_pid(nrf.radioo.pid[0], nrf.radioo.pid[1],nrf.radioo.pid[2]);
        pidY.set_pid(nrf.radioo.pid[0], nrf.radioo.pid[1],nrf.radioo.pid[2]);    
        pidZ.set_pid(nrf.radioo.pidz[0], nrf.radioo.pidz[1],nrf.radioo.pidz[2]);        
      }
      
      // send ack
      nrf.writeAckPayload();
      radio_timer = millis();

    } else if (abs(millis() - radio_timer > radioTimeOut)) nrf.reset_pwm();
    
    // update ack
    nrf.set_ack_vgt(voltage.v1s, voltage.v2s, voltage.v3s);
    nrf.set_ack_pwm(motorPwm[0], motorPwm[1],motorPwm[2],motorPwm[3]);
    nrf.set_ack_agl(imu.cur_Angle[0], imu.cur_Angle[1], imu.cur_Angle[2]);

    task_nrf_timecount = micros() - gtimer;
    vTaskDelay(5);
  
  }
}


// serial output
void task_serial( void * pvParameters ) {
  for (;;) {
    unsigned long gtimer = micros();

    static char str = '1';
    if (Serial.available() > 0) {
      str  = Serial.read();
    }

    int delay_time = 50; 
    switch (str) {
      case 'a':
        // print angle
        Serial.print(imu.cur_Angle[0]);
        Serial.print("   ");
        Serial.print(imu.cur_Angle[1]);
        Serial.print("   ");
        Serial.println(imu.cur_Angle[2]);
        break;

      case 'b':
        // print gyro
        Serial.print(imu.cur_Gyro[0]);
        Serial.print("   ");
        Serial.print(imu.cur_Gyro[1]);
        Serial.print("   ");
        Serial.println(imu.cur_Gyro[2]);
        break;
      
      case 'c':
        // sample count
        Serial.println(imu.sample_count);
        imu.sample_count = 0;
        delay_time = 1000;
        break;

      case 'd':
        // loop time in microseconds
        Serial.print("   main: ");
        Serial.print(task_main_timecount);
        Serial.print("   nrf: ");
        Serial.print(task_nrf_timecount);
        Serial.print("   serial: ");
        Serial.print(task_serial_timecount);
        Serial.print("   motor: ");
        Serial.print(task_motor_timecount);
        Serial.print("   vtg: ");
        Serial.println(task_vtg_timecount);
        break;   

      case 'e':
        // motor outputs
        Serial.print("B:");
        Serial.print(motorPwmWrite[1]);
        Serial.print("  A:");
        Serial.print(motorPwmWrite[0]);
        Serial.print("  D:");
        Serial.print(motorPwmWrite[3]);
        Serial.print("  C:");
        Serial.println(motorPwmWrite[2]);
        break;
    }    

    task_serial_timecount = micros() - gtimer;
    vTaskDelay(delay_time);

  }
}

