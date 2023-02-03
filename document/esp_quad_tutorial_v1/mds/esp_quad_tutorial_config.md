<br></br>
<br></br>
<br></br>
The code below is the configuration file([config.h](https://github.com/ic1414/esp_quad/blob/main/src/esp_quad_v4.1/include/config.h)).

```c++

#ifndef _config_h_
#define _config_h_

#pragma once

/*


// #define initEEPROM

// please do the follow before configuration
// uncomment the #define initEEPROM
// upload to the flight controller
// comment the #define initEEPROM
// upload to the flight controller again
// start configuration



This is your drone(top view)
       |
       ↓
  B(ac)  A(c)
    \\ ↑ //     <- here is head
     \\ //
     // \\
    //   \\
  D(c)   C(ac)
  A, B, C, D are motors
  (ac) rotating anticlockwise
  (c)  rotating clockwise



drone angles(top view)
       y(roll)
    B  |  A
    \\ | //    
     \\|// 
 ------.--------x(pitch)   <--the dot at the center is z axis
     //|\\
    // | \\
   D   |   C 
  when AB into the plane, CD out of the palne, angular velocity of x is positive.
  when AC into the plane, BD out of the palne, angular velocity of y is positive.
  when A rotates to C about the z axis, angular velocity of z is positive.

* if you mount the MPU6050 in other orientation, you need to set the output of the MPU6050 as said in the "drone angles(top view)"



joystick angles
*The values of the single axis of the joystick are from 0~255. if you are using SBUS, 172~1811 wil be converted to 0~255

0~255 will be converted to -30~30(degree) for the x and y axis, and -60~60(degree) for the z-axis to be the setpoints of the PID controller

            y(roll)
           (255)30
(joystick1) |
            |
            |
(0)-30------.-------- (255)30 x(pitch)   <--the dot is the stick of the joystick
            |
            |
            |
           (0)-30


the t axis of the joystick will be convert from 0~255 to (0us~800us)*. See "*some notes(us means microsecond)" for more details
            t(throttle)
          (255)800
(joystick2) |
            |
            |
(0)-60------.-------- (255)60 z(yaw)   <--the dot is the stick of the joystick
            |
            |
            |
          (0)0

*some notes(us means microsecond)
The default output of the PID controller is from 0us to 350us (pid_maxPwmOut). But the PWM protocol of the ESC is 1000us to 2000us, so the final output will be motor_minPwm(1000us) + throttle(0us to 800us) + pid_output(0us to 350us). The final output will be constrained within 2000us.
(do not worry if you are using SBUS because the SBUS function will convert PWM values to SBUS values)


*some history
This flight controller was designed initially for PWM ESCs, so you may notice that the motor or PID outputs in the code use PWM protocol as the standard.





*/



/*
How to see drone data from the serial monitor of the Arduino IDE.


Open serial monitor, set the bound rate to 250000 and choose no terminator.

Send single charactors like a, b, c, d, e, f, g. you can find more details in the file seriale.h
*/



enum PIDD{P, I, D}; // not recommended to change
enum XYZT{X, Y, Z, T}; // not recommended to change
enum V1234{V1, V2, V3, V4}; // not recommended to change
enum mABCD{mA, mB, mC, mD}; // not recommended to change


~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


// choose 1 receiver protocol
#define nrff
// sbus uses uart2 of esp32
//#define sbus 


// default PCB is pcb_v1, smd means pcb_v2
#define smd 

// ESC protocol
#define dshot
// #define pwmm
// #define brushed // did not test



// filters-----------------------------------------------------------------------------------------------------------------
// go here https://github.com/denyssene/SimpleKalmanFilter to find out details about the filter 


// *default values perform quite well
// higher filtering, smoother output, higher delay
// lower filtering, rougher output, lower delay
SimpleKalmanFilter dtermFilterX(4.0f, 4.0f, 0.25); // d term filter of x axis(PID)
SimpleKalmanFilter dtermFilterY(4.0f, 4.0f, 0.25); // d term filter of y axis(PID)
SimpleKalmanFilter dtermFilterZ(5.0f, 5.0f, 0.25); // d term filter of z axis(PID)

// go to mpu.h for more details if you want to change the values
#define dmpAngleWeight 0.5
#define accAngleWeight 0.002 

// imu angle filter
// angle filter is not very necessary
//#define angleFilter // uncomment for using the filter
SimpleKalmanFilter filterX(1.5f, 1.5f, 0.05f);
SimpleKalmanFilter filterY(1.5f, 1.5f, 0.05f);

// imu anfular velocity filter
// *default values perform quite well
#define gyroFilter // uncomment for using the filter
SimpleKalmanFilter filterGX(5.0f, 5.0f, 0.1f);
SimpleKalmanFilter filterGY(5.0f, 5.0f, 0.1f);
SimpleKalmanFilter filterGZ(5.0f, 5.0f, 0.1f);



// wifi PID tunning-----------------------------------------------------------------------------------------------------------

#define serverNameOTA "your wifi name~~~~~~~~ ~~~~~~~~~ "
#define serverPassOTA "wifi pass ~~ ~~~~~~~~ ~~~~~~~~~~~"

#ifdef smd 
    #define serverNamePID "esp32smd"
    #define serverPassPID "gtwhhhh111"
#else
    #define serverNamePID "esp32"
    #define serverPassPID "gtwhhhh111"
#endif

/*
How to tune PID by the webpage

you can begin with these values whenPID tunning
lx means max output in microseconds of p component in PID
           p    i       d    lp    li   ld
PID pidX(1.6f, 0.0f, 350.0f, 250, 100, 200);
PID pidY(1.6f, 0.0f, 350.0f, 250, 100, 200);
PID pidZ(4.5f, 0.0f, 0.000f, 250, 100, 200); 


1. disable the arm signal by the toggle switch.

2. set the x and y of the joystick both to -30 degree/bottom left.(you can find more details in the task_reset() of the main code)

3. coonect to the wifi named esp32smd/esp32, default password gtwhhhh111.

4. open a browser, go to 192.168.4.1 or http://192.168.4.1(if you can not find the webpage, connect the flight controller to the serial monitor,then reboot, see the IP address of ESP32.)

5. configure PID

6. click exit.
*/



// something--------------------------------------------------------------------------------------------------------------
#define pidLoopDelay 5 // PID loop delay in ms
#define recieverLoopDelay 2 // receiver loop delay in ms


// pid----------------------------------------------------------------------------------------------------------------------
#define pid_maxPwmOut 350 
#define pid_maxAngle 70.0 // if the angle of the drone exceeds this value, turn off the motors for safty. (exceed the range -70~70)



// motor-------------------------------------------------------------------------------------------------------------------
#define motor_minPwm 1000 // not recommended to change
#define motor_minPwmRunning 1070 //when you arm the drone, motor will start running at this value.
#define motor_midPwm 1500 // not recommended to change
#define motor_maxPwm 2000 // not recommended to change
#ifdef smd
    #define motorA_pin 25
    #define motorB_pin 33
    #define motorC_pin 26
    #define motorD_pin 32
#else
    #define motorA_pin 32
    #define motorB_pin 33
    #define motorC_pin 27
    #define motorD_pin 13
#endif



// use serial monitor to help you if you mount the flight controller in other orientation.
// you need to set the output of the MPU6050 as said in the "drone angles(top view)"

// mpu6050--------------------------------------------------------------------------------------------------------
#ifdef smd
  //#define mpu_invertX // x = -x
  #define mpu_invertY // y = -y
  // #define mpu_invertZ // z = -z
  #define mpu_swapXY // x = y, y = x
#else
  // #define mpu_invertX // x = -x
  // #define mpu_invertY // y = -y
  // #define mpu_invertZ // z = -z
  // #define mpu_swapXY // x = y, y = x
#endif


// put the drone on a flat plane, connect to serial monitor, power the drone and write down the offset values.

// uncomment #define mpu_manu_gyro_offset if you have offset values.
//#define mpu_manu_gyro_offset
#define mpu_gyro_offsetX  0
#define mpu_gyro_offsetY  0
#define mpu_gyro_offsetZ  0

// uncomment #define mpu_manu_acc_offset if you have offset values.
//#define mpu_manu_acc_offset
#define mpu_acc_offsetX  0
#define mpu_acc_offsetY  0
#define mpu_acc_offsetZ  0



// reciever----------------------------------------------------------------------------------------------------------------------
#define recie_timeOut 600 // unit is ms
#define recie_minJoyAnalog 0   // not recommended to change
#define recie_midJoyAnalog 127 // not recommended to change
#define recie_maxJoyAnalog 255 // not recommended to change
// PID setpoints 
#define recie_maxJoyAngleX 30 // max setpoint x(degree) -30 to 30
#define recie_maxJoyAngleY 30  // max setpoint y(degree) -30 to 30
#define recie_maxJoyAngleZ 60 //  // max setpoint z(degree) -60 to 60. z axis PID uses different logic, higher value equals faster yaw rotation.
// 油门pwm
#define recie_minJoyTpwm 0 // not recommended to change
#define recie_maxJoyTpwm 800 

// use serial monitor to help you
// following are some defaut values.
// 14 means when joystick is within -14~14(unit is dregree), flight controller will not responde.
#define recie_joyInnerDeadZoneX 1 
#define recie_joyInnerDeadZoneY 1
#define recie_joyInnerDeadZoneZ 14

#ifdef sbus
    #define sbus_minVal 172 // not recommended to change
    #define sbus_midVal 992 // not recommended to change
    #define sbus_maxVal 1811 // not recommended to change

    // SBUS  has a total of 16 channels, so the position values are form 0 to 15.
    // example--> SBUS_DATA[16] = {x, y, z, t, a.........};
    // SBUS_DARA[0] is joystick x values. 0 is the position.

    #define sbus_channelX 0 // position of the joystickX in the sbus data pakect.
    #define sbus_channelY 1 // position of the joystickY in the sbus data pakect.
    #define sbus_channelZ 2 // position of the joystickZ in the sbus data pakect.
    #define sbus_channelT 3 // position of the joystickT in the sbus data pakect.
    #define sbus_channelEnable 4 // position of the arm signal in the sbus data pakect.
    #define sbus_channelExtra 5
#else
    #ifdef smd
        #define nrf_cePin 27 // nrf24l01 ce
        #define nrf_csnPin 13 // nrf24l01 csn
    #else
        #define nrf_cePin 26 // nrf24l01 ce
        #define nrf_csnPin 25 // nrf24l01 csn
    #endif
    #define nrf_sendAddress "00001"
    #define nrf_recieveAddress "00002"
#endif

/*
nrf data format

//receive
struct nrf {
    bool en; //
    bool extra;
    byte joyStick[4]; // x, y, z, t  
    uint8_t synchronizeByte = 0; // ignore this
};
//send to transmmiter
struct drone_data {
    float voltage; // battery voltage
    byte motorOut[4]; // a, b, c, d
    int cur_gyro[3]; // current gyro
    float cur_angle[3]; // current angle
};
*/


// voltage--------------------------------------------------------------------------------------------------------------------
// v1s_hr means high side resistor
// v1s_lr means low side resistor

#ifdef smd
    #define v1sPin 35 
    #define v2sPin 34
    #define v3sPin 39
    #define v4sPin 39
    #define v1s_lr 3.0f
    #define v1s_hr 5.1f
    #define v2s_lr 5.1f
    #define v2s_hr 10.0f
    #define v3s_lr 5.1f
    #define v3s_hr 18.0f
    #define v4s_lr 3.0f
    #define v4s_hr 18.0f
#else
    #define v1sPin 14
    #define v2sPin 15
    #define v3sPin 4
    #define v4sPin 4
    #define v1s_lr 4.70f
    #define v1s_hr 9.40f
    #define v2s_lr 4.70f
    #define v2s_hr 14.7f
    #define v3s_lr 2.20f
    #define v3s_hr 9.00f
    #define v4s_lr 1.0f
    #define v4s_hr 1.0f
#endif



#endif
```