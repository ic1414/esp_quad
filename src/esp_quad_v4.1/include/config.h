#ifndef _config_h_
#define _config_h_

#pragma once

/*

注释没写清楚的地方可以b站评论或私信。到时候改。


*** pwm值的范围是1000-2000， 但注意下边pwm范围是0-1000

只有自稳模式!!! (因为我不会飞手动)

看的出来这是个无人机吧?
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


关于默认姿态角度
       y(滚转)
    B  |  A
    \\ | //    
     \\|// 
 ------.--------x(俯仰)   <--中间的点是z轴
     //|\\
    // | \\
   D   |   C 

  当AB入纸， CD出纸， x角速度正
  当AC入纸， BD出纸， y角速度正
  当A往C转(平面)，    z角速度正


关于默认摇杆角度
0到255 是接收机的摇杆值
-30到30和-60到60是pid的目标角度
0到800是pwm油门信号。最大为1000
            y(滚转)
           (255)30
            |
            |
            |
(0)-30------.-------- (255)30 x(俯仰)   <--中间的点是摇杆
            |
            |
            |
           (0)-30

            t(油门)
          (255)800
            |
            |
            |
(0)-60------.-------- (255)60 z(偏航)   <--中间的点是摇杆
            |
            |
            |
          (0)0


*如何在串口监视器看飞控数据
打开串口监视器，设波特率为250000，选择没有结束符。
发送字母 a, b, c, d, e, f, g。 更多信息可以在seriale.h找到


*如何进入网页调参
1.飞控上下翻转或把x和y摇杆转到-30 即可进入调参状态。(更多信息可以在espxx.ino 中的 task_reset()找到)
2. 手机/电脑 连接ssid为 esp32smd/esp32 的wifi。密码是gtwhhhh111
3. 打开浏览器，搜索网址 192.168.4.1 或 http://192.168.4.1。(如果搜不到，飞控连串口监视器并设波特率为250000。重复步骤1查看串口输出的ip地址。)
4，设置pid
5，退出/exit 并自动保存
(设置好后记得点 exit)
*/


enum PIDD{P, I, D};
enum XYZT{X, Y, Z, T};
enum V1234{V1, V2, V3, V4};
enum mABCD{mA, mB, mC, mD};


// 下边开始修改~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


// 下面2个选一个
#define nrff
//#define sbus // 取消注释使用sbus。sbus使用esp32的uart2管脚。(pcb_v2才专门有预留， pcb_v1需要自己去飞线)


// 默认是 pcb_v1
//#define smd // 取消注释, 如果你用的是pcv_2

// 下面4个选一个
#define dshot // 取消注释使用dshot
// #define pwmm // 取消注释使用dshot
// #define brushed // 取消注释使用类似analogWrite"的方法控制电机



// filters-----------------------------------------------------------------------------------------------------------------
// 注意滤波越小越好。滤的越多，延迟越高。
SimpleKalmanFilter dtermFilterX(4.0f, 4.0f, 0.25); // x d控制器滤波
SimpleKalmanFilter dtermFilterY(4.0f, 4.0f, 0.25); // y d控制器滤波
SimpleKalmanFilter dtermFilterZ(5.0f, 5.0f, 0.25); // z d控制器滤波

// 具体看mpu.h中源码
#define dmpAngleWeight 0.5 //100hz采样
#define accAngleWeight 0.002 //1000hz采样

// imu角度滤波
//#define angleFilter // 取消注释使用滤波
SimpleKalmanFilter filterX(1.5f, 1.5f, 0.05f);
SimpleKalmanFilter filterY(1.5f, 1.5f, 0.05f);

// imu 角速度滤波
#define gyroFilter // 取消注释使用滤波
SimpleKalmanFilter filterGX(5.0f, 5.0f, 0.1f);
SimpleKalmanFilter filterGY(5.0f, 5.0f, 0.1f);
SimpleKalmanFilter filterGZ(5.0f, 5.0f, 0.1f);



// wifi调参-----------------------------------------------------------------------------------------------------------------
// 新芯片最好取消下行注释上传一遍。上传后再将下行注释，再上传。
// #define initEEPROM

#define serverNameOTA "MEO-813190"
#define serverPassOTA "b7a5dafa3b"

#ifdef smd 
    #define serverNamePID "esp32smd"
    #define serverPassPID "gtwhhhh111"
#else
    #define serverNamePID "esp32"
    #define serverPassPID "gtwhhhh111"
#endif



// something--------------------------------------------------------------------------------------------------------------
#define pidLoopDelay 5 // 每5ms运行一次pid
#define recieverLoopDelay 2 // 每2ms读取一次接收机



// pid----------------------------------------------------------------------------------------------------------------------
#define pid_maxPwmOut 350 // 每个电机最大pwm可调值。建议300 到400
#define pid_maxAngle 70.0 // 如果x或y姿态大于这个角度则关闭电机



// motor-------------------------------------------------------------------------------------------------------------------
#define motor_minPwm 1000 // pwm最小值。不建议改
#define motor_minPwmRunning 1070 //接收机发送使能信号(arm)后电机会以较低速度转。类似betaflight的airMode。
#define motor_midPwm 1500 // pwm中位值，不建议改
#define motor_maxPwm 2000 // pwm最大值。不建议改
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



// mpu6050--------------------------------------------------------------------------------------------------------
// 用串口监视器辅助设置
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


// 注意飞控每次上电都会自动校准。如果你不要每次都校准，先找个较平的地，飞控连串口监视器后复位就可以看到校准值。
// 如果已经有校准值，取消下一行的注释
//#define mpu_manu_gyro_offset
#define mpu_gyro_offsetX  0
#define mpu_gyro_offsetY  0
#define mpu_gyro_offsetZ  0

//如果已经有校准值，取消下一行的注释
//#define mpu_manu_acc_offset
#define mpu_acc_offsetX  0
#define mpu_acc_offsetY  0
#define mpu_acc_offsetZ  0



// reciever----------------------------------------------------------------------------------------------------------------------
#define recie_timeOut 600 // 超过600ms没接收机信号则判断信号丢失，电机将停转
// 接收机摇杆值是0到255
#define recie_minJoyAnalog 0   // nrf接收机摇杆最低值。如果用sbus的话可以改
#define recie_midJoyAnalog 127 // nrf接收机摇杆中位值。如果用sbus的话可以改
#define recie_maxJoyAnalog 255 // nrf接收机摇杆最大值。如果用sbus的话可以改
// pid目标角度
#define recie_maxJoyAngleX 30 // x pid的最大目标角度。 -30-30
#define recie_maxJoyAngleY 30 // y pid的最大目标角度。 -30-30
#define recie_maxJoyAngleZ 60 // z pid的最大目标角度。z没有固定角度，数值越大则转的越快。设为-60-60, 这样kpz不用设置太大。
// 油门pwm
#define recie_minJoyTpwm 0 // 油门最低pwm
#define recie_maxJoyTpwm 800 // 油门最大pwm。可以设置到1000。

#define recie_joyInnerDeadZoneX 1 //x摇杆角度内圈死区 
#define recie_joyInnerDeadZoneY 1 //y摇杆角度内圈死区 
#define recie_joyInnerDeadZoneZ 14 //z摇杆角度内圈死区 

#ifdef sbus
    #define sbus_minVal 172 // 接收机sbus摇杆最低值
    #define sbus_midVal 992 // 接收机sbus摇杆中位值
    #define sbus_maxVal 1811 // 接收机sbus摇杆最大值
    #define sbus_channelX 0 // x摇杆位置， 数组范围0-15
    #define sbus_channelY 1 // y摇杆位置， 数组范围0-15
    #define sbus_channelZ 2 // z摇杆位置， 数组范围0-15
    #define sbus_channelT 3 // t摇杆位置， 数组范围0-15
    #define sbus_channelEnable 4 // arm或使能，位置， 数组范围0-15
    #define sbus_channelExtra 5 // 预留，位置， 数组范围0-15
#else
    #ifdef smd
        #define nrf_cePin 27 // nrf24l01 ce 管脚
        #define nrf_csnPin 13 // nrf24l01 csn 管脚
    #else
        #define nrf_cePin 26 // nrf24l01 ce 管脚
        #define nrf_csnPin 25 // nrf24l01 csn 管脚
    #endif
    #define nrf_sendAddress "00001" // nrf24l01 发送地址
    #define nrf_recieveAddress "00002"// nrf24l01 接收地址
#endif

/*
用nrf的话，根据您的需求去nrf.h修改
nrf 数据格式，根据您的遥控改
//接收
struct nrf {
    bool en; //
    bool extra;
    byte joyStick[4]; // x, y, z, t  
    uint8_t synchronizeByte = 0;
};
//回传给遥控
struct drone_data {
    float voltage; // 当前电压
    byte motorOut[4]; // a, b, c, d
    int cur_gyro[3];  // 当前角速度 x, y, z
    float cur_angle[3]; // 当前角度 x, y, z
};
*/


// voltage--------------------------------------------------------------------------------------------------------------------
// 注意电压测出来不怎么准
// 根据你的配置改。默认用的是3s电池，所以下边4s参数就随便设置了。
// 1s 管脚
// 2s 管脚
// 3s 管脚
// 4s 管脚
// 1s 低位电阻
// 1s 高位电阻
// 2s 低位电阻
// 2s 高位电阻
// 3s 低位电阻
// 3s 高位电阻
// 4s 低位电阻
// 4s 高位电阻
#ifdef smd
    #define v1sPin 35 
    #define v2sPin 34
    #define v3sPin 39
    #define v4sPin 36
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