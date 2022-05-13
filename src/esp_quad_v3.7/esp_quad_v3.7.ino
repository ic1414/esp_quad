/*

只有自稳模式!!!!!!! (因为我不会飞手动)
看的出来这是个无人机吧? (俯视)
       |
       ↓
  B(ac)  A(c)
    \\ ↑ //     <- 这里是机头 
     \\ //
     // \\
    //   \\
  D(c)   C(ac)
  A, B, C, D 是电机的代号
  (ac) 是逆时针
  (c)  是顺时针
  
关于角度
       y
    B  |  A
    \\ | //    
     \\|// 
 ------.--------x   <--中间的点是z轴
     //|\\
    // | \\
   D   |   C 
  当AB进入纸面， CD出纸， x角速度为正
  当AC进入纸面， BD出纸， y角速度为正
  当A转到C(平面)(顺时针)，z角速度为正
  
提示
一般数组都是
xxx[3] x, y, z 角度
xxx[3] x, y, z 角速度
xxx[4] x, y, z, t  <-t是油门
xxx[4] a, b, c, d 电机pwm

*/


#include "include/inclu.h"
#include "BluetoothSerial.h"

SimpleKalmanFilter filterA(75, 75, 1);
SimpleKalmanFilter filterB(75, 75, 1);
SimpleKalmanFilter filterC(75, 75, 1);
SimpleKalmanFilter filterD(75, 75, 1);

// 蓝牙
BluetoothSerial SerialBT;
volatile boolean exit_ble = false;
volatile boolean connected = false;
void server_ble();


// ESP32 多线程
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


//123
MPU imu;


//123
VTG voltage;


//123
NRF nrf;
#define radioTimeOut 600


//电调的pwm是 1000-2000
volatile int motorPwm[4] = {1000, 1000, 1000, 1000}; //电机 a，b，c，d pwm
volatile int motorPwmWrite[4] = {1000, 1000, 1000, 1000}; //电机 a, b, c, d


// pid
#define dt 5 // milli
#define maxPWMout 300 // 限制最终输出
#define maxAngle 70.0 // 限制角度， 万向锁, 安全
PID pidX(1.6f, 0.0f, 350.0f, 250, 100, 200);
PID pidY(1.6f, 0.0f, 350.0f, 250, 100, 200);
PID pidZ(4.5f, 0.0f, 0.000f, 250, 100, 200); 
PID pidGX(0.9f, 1.5f, 0.0f, 250, 100, 200); // 还没试过角速度环
PID pidGY(0.9f, 1.5f, 0.0f, 250, 100, 200); // 还没试过角速度环
PID pidGZ(4.5f, 0.0f, 0.00f, 250, 100, 200); // 还没试过角速度环


void setup() {
  // hi
  Serial.begin(250000);
  /*
  unsigned long timer_ble = millis();
  SerialBT.begin("ESP32");
  while(not(exit_ble)){
    if(millis() - timer_ble > 10000 and connected == false) exit_ble = true;
    server_ble(); // server
  }*/


  imu.begin();
  nrf.begin();
  motor_begin();
  voltage.begin();

  // task initiate， esp32多线程, void loop() 用的是核心1 (esp32有俩核心)
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
 // 最好什么代码都不要放， 会降低imu采样率
  imu.update();
}


// main
void task_main( void * pvParameters ) {
  //设置z轴初始角度， 具体下面说
  static float setPointZ = 0; 
  
  // start when mpu6050 started
  while (imu.cur_Angle[0] == 0.0f) vTaskDelay(4);
  
  // main loop
  for (;;) {

    float setAngles[3]; // pid目标角度 x，y，z 
    // 将摇杆的 0 - 255 转换成 -30 - 30 作为 x, y目标角度
    setAngles[0] = map(nrf.radioo.pwm[0], 0, 255, -30, 30);
    setAngles[1] = map(nrf.radioo.pwm[1], 0, 255, -30, 30);
    // 将z目标角度设为 -60 - 60, 这样p常数不用设置太大
    setAngles[2] = map(nrf.radioo.pwm[2], 0, 255, -60, 60);

    // 先记着， 下面说
    boolean zMove = false;
    boolean zRadio = false;
    if (abs(imu.cur_Angle[2] - setPointZ) > 0.5) zMove = true;
    if (nrf.radioo.pwm[2] > 135 or nrf.radioo.pwm[2] < 119) zRadio = true;

    // 将摇杆的油门0-255转换成0-1000
    int tPwm = (int)nrf.radioo.pwm[3] * 4 - 20;  // 255 * 4 = 1020
    int pid_out_pwm[4] = {0, 0, 0, tPwm};  // pid最终输出 x, y, z, t
    int pid_out_agl[3] = {0, 0, 0};  // pid角度环输出 x, y, z 
    int pid_out_gro[3] = {0, 0, 0};  // pid角速度环输出 x, y, z 

    // x，y目标角度是转换后的摇杆 x，y
    pid_out_agl[0] = pidX._calculate(setAngles[0], imu.cur_Angle[0], dt);
    pid_out_agl[1] = pidY._calculate(setAngles[1], imu.cur_Angle[1], dt);

    // 如果z摇杆动了， 更新setPointz为当前z角度， 并将目标角度设为当前z角度加上z摇杆
    // z目标角度不是固定的(x和y是-30 - 30)， 因为z目标角度一直是当前z角度 + z摇杆
    // 如果此刻z角度是20.1，z摇杆是20， 那么z目标角度就是40.1。 在下一个循环z角度变成20.5的话，且z摇杆不变，则z目标角度为40.5
    float cur_AngleZ = imu.cur_Angle[2]; 
    if (zRadio == true) {
      setPointZ = cur_AngleZ;
      setAngles[2] = setAngles[2] + cur_AngleZ;
      pid_out_agl[2] = pidZ._calculate(setAngles[2], cur_AngleZ, dt);
    }else if(zMove == true){
      // 如果z角度受到干扰， 则目标角度为setPointZ。 setPointZ 只有在摇杆动的时候更新
      // 如果此刻setPointZ为10且z角度也是10，什么都不做。但由于风的干扰z角度变成了15， 那z目标角度就是10(如果z摇杆没动的话)
      if(abs(setPointZ - cur_AngleZ) <= 180.0f){ // 判断最小路径. dmp输出z角度范围在-180-180.
        setAngles[2] = setPointZ;
        pid_out_agl[2] = pidZ._calculate(setAngles[2], cur_AngleZ, dt);        
      }else{
        if(setPointZ <= 0){
           setAngles[2] =  180.0f - (-180.0f - setPointZ);
          pid_out_agl[2] = pidZ._calculate(setAngles[2], cur_AngleZ, dt);           
        }else{
          setAngles[2] =  -180.0f - (180.0f - setPointZ);
          pid_out_agl[2] = pidZ._calculate(setAngles[2], cur_AngleZ, dt);                 
        }
      }
    }


    // 没试过角速度环
    /*
    pid_out_gro[0] = pidGX._calculate(pid_out_agl[0], imu.cur_Gyro[0], dt);
    pid_out_gro[1] = pidGY._calculate(pid_out_agl[1], imu.cur_Gyro[1], dt);
    pid_out_gro[2] = pidGZ._calculate(pid_out_agl[2], imu.cur_Gyro[2], dt);
    */
    for(int i=0;i<3;i++) pid_out_pwm[i] = pid_out_agl[i]; // 将角度环设为最终输出
    //for(int i=0;i<3;i++) pid_out_pwm[i] = pid_out_gro[i]; // 没试过角速度环

    // 加起来
    int motorPwm_zip[4] = {0, 0, 0, 0}; //a，b，c，d
    motorPwm_zip[0] = 0 - pid_out_pwm[0] - pid_out_pwm[1] - pid_out_pwm[2];
    motorPwm_zip[1] = 0 - pid_out_pwm[0] + pid_out_pwm[1] + pid_out_pwm[2];
    motorPwm_zip[2] = 0 + pid_out_pwm[0] - pid_out_pwm[1] + pid_out_pwm[2];
    motorPwm_zip[3] = 0 + pid_out_pwm[0] + pid_out_pwm[1] - pid_out_pwm[2];
    // 限制每个电机pid输出
    for (int i = 0; i < 4; i++)motorPwm_zip[i] = constrain(motorPwm_zip[i], -maxPWMout, maxPWMout);
    for (int i = 0; i < 4; i++)motorPwm[i] = 1000 + pid_out_pwm[3] + motorPwm_zip[i];

    // 油门是0或角度大于70，电机pwm为1000(安全)
    boolean motorStop = false;
    boolean overAngle = false;
    if (abs(imu.cur_Angle[0]) > abs(maxAngle) or
        abs(imu.cur_Angle[1]) > abs(maxAngle))  overAngle = true;
    if (nrf.radioo.pwm[3] == 0 or nrf.radioo.en == 0) motorStop = true;
    
    // motorPwmWrite 会被 void task_motor 输出给电调 
    // 我发现电机比较容易烫，所以加个滤波
    if (not(overAngle) and not(motorStop)) {
      motorPwmWrite[0] = filterA.updateEstimate(constrain(motorPwm[0], 1000, 2000));
      motorPwmWrite[1] = filterB.updateEstimate(constrain(motorPwm[1], 1000, 2000));
      motorPwmWrite[2] = filterC.updateEstimate(constrain(motorPwm[2], 1000, 2000));
      motorPwmWrite[3] = filterD.updateEstimate(constrain(motorPwm[3], 1000, 2000));
    } else {
      pidX.integral_reset();
      pidY.integral_reset();
      pidZ.integral_reset();
      motorPwmWrite[0] = 1000;
      motorPwmWrite[1] = 1000;
      motorPwmWrite[2] = 1000;
      motorPwmWrite[3] = 1000;
    }

    vTaskDelay(dt);
  }

  Serial.println("never going to reach");
}


// vtg
void task_vtg( void * pvParameters ) {

  for (;;) {
    //check voltage
    voltage.update();
    vTaskDelay(100);
  
  }

  Serial.println("never going to reach");
}


void task_motor( void * pvParameters ) {

  for (;;) {
    // update motor pwm
    motorA.writeMicroseconds(motorPwmWrite[0]);
    motorB.writeMicroseconds(motorPwmWrite[1]);
    motorC.writeMicroseconds(motorPwmWrite[2]);
    motorD.writeMicroseconds(motorPwmWrite[3]);
    vTaskDelay(3);
  }

  Serial.println("never going to reach");
}


// radio
void task_nrf( void * pvParameters ) {
  // loop forever
  for (;;) {
    // 用来检查有没有超时
    static unsigned long radio_timer = millis();

    if (nrf.available()) {
      nrf.read(); // read
      
      /*if (not(nrf.radioo.pid[0] + nrf.radioo.pid[1] + nrf.radioo.pid[2] == 0)){
        pidX.set_pid(nrf.radioo.pid[0], nrf.radioo.pid[1],nrf.radioo.pid[2]);
        pidY.set_pid(nrf.radioo.pid[0], nrf.radioo.pid[1],nrf.radioo.pid[2]);    
        pidZ.set_pid(nrf.radioo.pidz[0], nrf.radioo.pidz[1],nrf.radioo.pidz[2]);        
      } */

      // ack
      nrf.writeAckPayload();
      radio_timer = millis();

    } else if (abs(millis() - radio_timer > radioTimeOut)) nrf.reset_pwm();
    
    // update ack
    nrf.set_ack_vgt(voltage.v1s, voltage.v2s, voltage.v3s);
    nrf.set_ack_agl(imu.cur_Angle[0], imu.cur_Angle[1], imu.cur_Angle[2]);
    nrf.set_ack_pwm(motorPwmWrite[0], motorPwmWrite[1],motorPwmWrite[2],motorPwmWrite[3]);

    vTaskDelay(5);
  }

  Serial.println("never going to reach");
}


void server_ble() {

  String s = " ";
  static String select = " ";
  String output = "";

  if(SerialBT.available() > 0) {
    connected = true;
    s = SerialBT.readStringUntil('\n');
    s.remove(s.length()-1, 1);
    if(s == "exit") SerialBT.end();
    if(s == "exit") exit_ble = true;
  }

  if(s == "Angle"){
    select = "Angle";
    output += "kp: " + String(pidX.kp);
    output += "  ki: " + String(pidX.ki);
    output += "  kd: " + String(pidX.kd);
    output += "  pz: " + String(pidZ.kp);
    output += "  iz: " + String(pidZ.ki);
    output += "  dz: " + String(pidZ.kd);
    SerialBT.println(output);       
    output = "";
  }
  if(s == "Gyro"){
    select = "Gyro";
    output += "kp: " + String(pidGX.kp);
    output += "  ki: " + String(pidGX.ki);
    output += "  kd: " + String(pidGX.kd);
    output += "  pz: " + String(pidGZ.kp);
    output += "  iz: " + String(pidGZ.ki);
    output += "  dz: " + String(pidGZ.kd);
    SerialBT.println(output);     
    output = "";  
  }

  if(select == "Angle"){
    int slen = s.length();
    char temp = s[0];
    switch (s[0]) {
      case 'p':
        s = s.substring(1, slen);
        pidX.kp = s.toFloat();
        pidY.kp = s.toFloat();
        break;
      case 'i':
        s = s.substring(1, slen);
        pidX.ki = s.toFloat();
        pidY.ki = s.toFloat();
        break;
      case 'd':
        s = s.substring(1, slen);
        pidX.kd = s.toFloat();
        pidY.kd = s.toFloat();
        break;
      case 'a':
        s = s.substring(1, slen);
        pidZ.kp = s.toFloat();
        break;
      case 'b':
        s = s.substring(1, slen);
        pidZ.ki = s.toFloat();
        break;
      case 'c':
        s = s.substring(1, slen);
        pidZ.kd = s.toFloat();
        break;
    }
    if(temp == 'p' or temp == 'i' or temp == 'd'){
      output += "kp: " + String(pidX.kp);
      output += "  ki: " + String(pidX.ki);
      output += "  kd: " + String(pidX.kd);  
      SerialBT.println(output);      
    }
    if(temp == 'a' or temp == 'b' or temp == 'c'){
      output += "pz: " + String(pidZ.kp);
      output += "  iz: " + String(pidZ.ki);
      output += "  dz: " + String(pidZ.kd);  
      SerialBT.println(output);   
    }
  }

  if(select == "Gyro"){
    int slen = s.length();
    char temp = s[0];
    switch (s[0]) {
      case 'p':
        s = s.substring(1, slen);
        pidGX.kp = s.toFloat();
        pidGY.kp = s.toFloat();
        break;
      case 'i':
        s = s.substring(1, slen);
        pidGX.ki = s.toFloat();
        pidGY.ki = s.toFloat();
        break;
      case 'd':
        s = s.substring(1, slen);
        pidGX.kd = s.toFloat();
        pidGY.kd = s.toFloat();
        break;
      case 'a':
        s = s.substring(1, slen);
        pidGZ.kp = s.toFloat();
        break;
      case 'b':
        s = s.substring(1, slen);
        pidGZ.ki = s.toFloat();
        break;
      case 'c':
        s = s.substring(1, slen);
        pidGZ.kd = s.toFloat();
        break;
    }
    if(temp == 'p' or temp == 'i' or temp == 'd'){
      output += "Gkp: " + String(pidGX.kp);
      output += "  ki: " + String(pidGX.ki);
      output += "  kd: " + String(pidGX.kd); 
      SerialBT.println(output);       
    }
    if(temp == 'a' or temp == 'b' or temp == 'c'){
      output += "Gpz: " + String(pidGZ.kp);
      output += "  iz: " + String(pidGZ.ki);
      output += "  dz: " + String(pidGZ.kd); 
      SerialBT.println(output);    
    } 
  }

  delay(10);
}


// serial output
void task_serial( void * pvParameters ) {
  for (;;) {

    static char str = '1';
    if (Serial.available() > 0) str = Serial.read();

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
        // print gyro
        Serial.print(imu.cur_Acc[0]);
        Serial.print("   ");
        Serial.print(imu.cur_Acc[1]);
        Serial.print("   ");
        Serial.println(imu.cur_Acc[2]);
        break;

      case 'f':
        // print gyro
        Serial.print(imu.cur_Acc_sum[0]);
        Serial.print("   ");
        Serial.print(imu.cur_Acc_sum[1]);
        Serial.print("   ");
        Serial.println(imu.cur_Acc_sum[2]);
        break;
      
      case 'd':
        // sample count
        Serial.println(imu.sample_count);
        imu.sample_count = 0;
        delay_time = 1000;
        break;  

      case 'e':
        // motor outputs
        Serial.print("B:");
        Serial.print(motorPwm[1]);
        Serial.print("  A:");
        Serial.print(motorPwm[0]);
        Serial.print("  D:");
        Serial.print(motorPwm[3]);
        Serial.print("  C:");
        Serial.println(motorPwm[2]);
        break;    
    }
    vTaskDelay(delay_time); 
  }

  Serial.println("never going to reach");
}
