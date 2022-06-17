/*
去config.h 设置

注释没写清楚的地方可以b站评论或私信。到时候改。

不建议用arduino ide读源码。建议用visual studio code 


提示，一般数组都是
xxx[3] x, y, z 角度
xxx[3] x, y, z 角速度
xxx[4] x, y, z, t  <-t是油门
xxx[4] a, b, c, d 电机pwm
*/


#include "include/inclu.h"、


// ESP32 双核调用
void task_vtg(void*);
void task_main(void*);
void task_reset(void*);
void task_serial(void*);
void task_reciever(void*);
TaskHandle_t TaskHandle_vtg;
TaskHandle_t TaskHandle_main;
TaskHandle_t TaskHandle_reset;
TaskHandle_t TaskHandle_serial;
TaskHandle_t TaskHandle_reciever;



void setup() {
  Serial.begin(250000); // hi

  #ifdef pidBle
    EEPROM.begin(4*30);
    #ifdef initBleEEPROM
      bleEEPROMinit(); // 初始化
    #endif
    readBleEEPROM(); // 从EEPROM读取k(pid)
    bleEEPROMcheckSetup(); // 查看是否进入调参模式
  #endif

  imu.begin();
  voltage.begin();
  reciever.begin();

  // 任务初始化, void loop() 用的是核心1， 下边的任务用核心0(esp32有俩核心)
  xTaskCreatePinnedToCore(task_vtg, "Task_vtg", 1000,
                          NULL, 1, &TaskHandle_vtg, 0);
  xTaskCreatePinnedToCore(task_main, "Task_main", 1000,
                          NULL, 8, &TaskHandle_main, 0);
  xTaskCreatePinnedToCore(task_reset, "Task_reset", 1000,
                          NULL, 3, &TaskHandle_reset, 0);
  xTaskCreatePinnedToCore(task_serial, "Task_serial", 1000,
                          NULL, 5, &TaskHandle_serial, 0);
  xTaskCreatePinnedToCore(task_reciever, "Task_reciever", 1000,
                          NULL, 10, &TaskHandle_reciever, 0);
}



void loop(){

  imu.update(); // update

  //loop 里不要放太多代码
  #ifdef pidBle
    bleEEPROMcheckLoop(); // 查看是否进入调参模式
  #endif

}



// main
void task_main( void * pvParameters ){
  // start when mpu6050 started
  while (imu.getAngle(X) == 0.0f) vTaskDelay(5);
  
  // main loop
  for(;;){
    
    // 将摇杆的 0 - 255 转换成 -30 - 30 作为 x, y目标角度
    // 将z目标角度设为 -60 - 60, 这样p不用设置太大
    float setAngles[3]; // pid目标角度 x，y，z
    setAngles[X] = map(reciever.getJoyStick(X), recie_minJoyAnalog, recie_maxJoyAnalog, -recie_maxJoyAngleX, recie_maxJoyAngleX);
    setAngles[Y] = map(reciever.getJoyStick(Y), recie_minJoyAnalog, recie_maxJoyAnalog, -recie_maxJoyAngleY, recie_maxJoyAngleY);
    setAngles[Z] = map(reciever.getJoyStick(Z), recie_minJoyAnalog, recie_maxJoyAnalog, -recie_maxJoyAngleZ, recie_maxJoyAngleZ);

    // 清除积分, 可以不用
    if(reciever.getExtra() == false){
      pidX.setIntegral(0);
      pidY.setIntegral(0);
      pidZ.setIntegral(0);
      pidGX.setIntegral(0);
      pidGY.setIntegral(0);
      pidGZ.setIntegral(0);  
    }

    int pid_out_agl[3] = {0, 0, 0}; // pid角度环输出 x, y, z 
    int pid_out_pwm[3] = {0, 0, 0}; // pid最终输出 x, y, z, t
    // x，y目标角度是转换后的摇杆 x，y
    if(abs(setAngles[X]) > recie_joyInnerDeadZoneX) pid_out_agl[X] = pidX.calculate(setAngles[X], imu.getAngle(X), pidLoopDelay, &dtermFilterX);
    else pid_out_agl[X] = pidX.calculate(0, imu.getAngle(X), pidLoopDelay, &dtermFilterX);
    if(abs(setAngles[Y]) > recie_joyInnerDeadZoneY) pid_out_agl[Y] = pidY.calculate(setAngles[Y], imu.getAngle(Y), pidLoopDelay, &dtermFilterY);
    else pid_out_agl[Y] = pidY.calculate(0, imu.getAngle(Y), pidLoopDelay, &dtermFilterY);
    /*
      if
      如果z摇杆动了， 更新setPointz为当前z角度， 并将z目标角度设为当前z角度加上z摇杆
      z目标角度不是固定的(x和y是-30 - 30)， 因为z目标角度一直是当前z角度 + z摇杆
      如果此刻z角度是20.1，z摇杆是20， 那么z目标角度就是40.1。 在下一个循环z角度变成20.5的话，且z摇杆不变，则z目标角度为40.5
      else
      如果z角度受到干扰， 则目标角度为setPointZ。 setPointZ 只有在摇杆动的时候更新
      如果此刻setPointZ为10且z角度也是10，什么都不做。在下个循环由于风的干扰z角度变成了15， 那z目标角度就是10(如果z摇杆没动的话)。
    */
    static float setPointZ = 0.0; 
    float temp_cur_AngleZ = imu.getAngle(Z); //复制一下
    if (abs(setAngles[Z]) > recie_joyInnerDeadZoneZ){
      setPointZ = temp_cur_AngleZ;
      setAngles[Z] += temp_cur_AngleZ;
      pid_out_agl[Z] = pidZ.calculate(setAngles[Z], temp_cur_AngleZ, pidLoopDelay, &dtermFilterZ);
    }else{
      if(abs(setPointZ - temp_cur_AngleZ) <= 180.0f){ // 判断最小路径. z角度范围在-180-180.
        setAngles[Z] = setPointZ;
        pid_out_agl[Z] = pidZ.calculate(setAngles[Z], temp_cur_AngleZ, pidLoopDelay, &dtermFilterZ);        
      }else{
        if(setPointZ <= 0) setAngles[Z] = 180.0f - (-180.0f - setPointZ);         
        if(setPointZ  > 0) setAngles[Z] = -180.0f - (180.0f - setPointZ);
        pid_out_agl[Z] = pidZ.calculate(setAngles[Z], temp_cur_AngleZ, pidLoopDelay, &dtermFilterZ);                 
      }
    }

    // 设角度环为最终输出
    for(int i=0; i<3; i++) pid_out_pwm[i] = pid_out_agl[i]; 

    // 加起来
    int motorPwm[4]; //a，b，c，d
    int motorPwm_zip[4]; //a，b，c，d
    motorPwm_zip[mA] = 0 - pid_out_pwm[X] - pid_out_pwm[Y] - pid_out_pwm[Z];
    motorPwm_zip[mB] = 0 - pid_out_pwm[X] + pid_out_pwm[Y] + pid_out_pwm[Z];
    motorPwm_zip[mC] = 0 + pid_out_pwm[X] - pid_out_pwm[Y] + pid_out_pwm[Z];
    motorPwm_zip[mD] = 0 + pid_out_pwm[X] + pid_out_pwm[Y] - pid_out_pwm[Z];
    for (int i = 0; i < 4; i++)motorPwm_zip[i] = constrain(motorPwm_zip[i], -pid_maxPwmOut, pid_maxPwmOut);

    int tPwm = map(reciever.getJoyStick(T), recie_minJoyAnalog, recie_maxJoyAnalog, recie_minJoyTpwm, recie_maxJoyTpwm); // 获取摇杆油门
    for (int i = 0; i < 4; i++)motorPwm[i] = motor_minPwm + tPwm + motorPwm_zip[i];

    // 油门是0或姿态角度大于75，电机关闭
    bool motorStop = false;
    bool overAngle = false;
    if(reciever.getEnable() == false) motorStop = true;
    if(abs(imu.getAngle(X)) > abs(pid_maxAngle) or abs(imu.getAngle(Y)) > abs(pid_maxAngle))  overAngle = true;

    motorA.setAStandardisedPwm(motorPwm[mA]);
    motorB.setAStandardisedPwm(motorPwm[mB]);
    motorC.setAStandardisedPwm(motorPwm[mC]);
    motorD.setAStandardisedPwm(motorPwm[mD]);
    if (not(overAngle) and not(motorStop)){
      motorA.setStandardisedPwm_write(motorPwm[mA], true);
      motorB.setStandardisedPwm_write(motorPwm[mB], true);
      motorC.setStandardisedPwm_write(motorPwm[mC], true);
      motorD.setStandardisedPwm_write(motorPwm[mD], true);
    } else {
      motorA.setStandardisedPwm_write(motor_minPwm, false);
      motorB.setStandardisedPwm_write(motor_minPwm, false);
      motorC.setStandardisedPwm_write(motor_minPwm, false);
      motorD.setStandardisedPwm_write(motor_minPwm, false);
    }

    vTaskDelay(pidLoopDelay);
  }
  
}


// vtg
void task_vtg(void *pvParameters){  for(;;){ voltage.update(); vTaskDelay(100);}  }

void task_reciever(void *pvParameters){  for(;;){ reciever.update(); vTaskDelay(recieverLoopDelay);}  }

// serial output
void task_serial(void *pvParameters){  for(;;){ int delay_time = serial_zip(); vTaskDelay(delay_time);}  }


// reset
void task_reset( void * pvParameters ){
  // start when mpu6050 started
  while (imu.getAngle(X) == 0.0f) vTaskDelay(5);

  for (;;) {
    static int joyStick_count = 0;
    static unsigned long timer_reset1;
    static unsigned long timer_reset2;
    
    // 摇晃飞控进入调参
    if(not(reciever.getEnable())){
      // 移动x和y摇杆到-30进入调参
      if(millis() - timer_reset1 > 200){
        int x = map(reciever.getJoyStick(X), recie_minJoyAnalog, recie_maxJoyAnalog, -recie_maxJoyAngleX, recie_maxJoyAngleX);
        int y = map(reciever.getJoyStick(Y), recie_minJoyAnalog, recie_maxJoyAnalog, -recie_maxJoyAngleY, recie_maxJoyAngleY);
        if(x + y < -50) joyStick_count += 1;
        timer_reset1 = millis();
      }
      if(millis() - timer_reset2 > 2000){
        joyStick_count = 0;
        timer_reset2 = millis();
      }

      //如果飞机上下翻转，也进入调参
      static bool upsideDown = false;
      if(abs(imu.getAngle(X)) > 90.0f) upsideDown = true;

      #ifdef pidBle
        // 查看条件
        if(upsideDown or joyStick_count > 3){
          motorA.setStandardisedPwm_write(motor_minPwm, false);
          motorB.setStandardisedPwm_write(motor_minPwm, false);
          motorC.setStandardisedPwm_write(motor_minPwm, false);
          motorD.setStandardisedPwm_write(motor_minPwm, false);
          setBleEnable(true);
        }
      #endif
    }
    vTaskDelay(10); 
  }

}


