## **Jump**
* [Intro](#intro)    
* [Features](#features)   
* [Parameters](#parameters)
* [Getting started](#getting-started)
* [Hardware](#hardware)
* [Software configuration](#software-configuration)
* [Upload the code](#upload-the-code)
* [Note](#note)
<br/><br/>
<br/><br/>

## **Intro**
A flight controller based on ESP32 for quadcopters. <u>In this document, I will use [esp_quad_v4.1](https://github.com/ic1414/esp_quad/tree/main/src/esp_quad_v4.1) and [pcb_v2](https://github.com/ic1414/esp_quad/tree/main/pcb/pcb_v2) to make examples.</u>
<br/><br/>
<br/><br/>

## **Features**
* Using Arduino IDE  
* Supports PID tuning by webpage
<br/><br/>
<br/><br/>

## **Parameters**
* **MCU:** ESP32-WROOM-UE or ESP32-WROOM-32D  <br/><br/>
* **Gyro+Accelero:** MPU6050(GY521 module)  <br/><br/>
* **ESC protocol:** DSHOT, PWM(1ms-2ms), PWM(analogWrite())  <br/><br/>
* **Receiver protocol:** SBUS, builtin NRF24L01  <br/><br/>
* **Mounting hole:** 30.5mm * 30.5mm  <br/><br/>
* **Size:** 39.75mm * 39.12mm  <br/><br/>
* **Input voltage:** 5V/3.3V  <br/><br/>
* **Working voltage:** 3.3V  <br/><br/>
* **Connectors:** JST SH1.0 4PIN and JST SH1.0 6PIN
<br/><br/>
<br/><br/>

## **Getting started**
**Install ESP32 boards(version 1.0.6):** If you dont know how to install it, you may follow this [tutorial](https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/) I found online..  
**Install libraries:** Download the latest version of the [code](https://github.com/ic1414/esp_quad/tree/main/src/esp_quad_v4.1), unzip the [library folder](https://github.com/ic1414/esp_quad/blob/main/src/esp_quad_v4.1/esp_quad_v4.1_lib.zip) and paste the **contents of the library folder** to <u>C:/Users/(___user_abc___)/Documents/Arduino/libraries</u>
<br/><br/>
<br/><br/>

## **Hardware**  
This PCB does not have silk screen for identifying component, so please use [Fig1](#fig1)  and [Fig2](#fig2) for references.

The <u>22uf 6.3V typeA capacitor</u> in [Fig1](#fig1) is a tantalum capacitor, make sure it is mounted in the  correct orientation because it has polarity.  
The 2 resistors that are not circled are 0 ohm resistors, so we can simply short the pads by solder, but we only need to short one of them. You can find the details [here](#about-the-resistors-and-power-input).  

#### **Fig1**
<img decoding="async" src="images/soldering top.jpeg" width="100%"> 
<br/><br/>

The 8 resistors that are circled in [Fig2](#fig2) are voltage divider resistors. You can find the details [here](#voltage-divider).  

#### **Fig2** 
<img decoding="async" src="images/soldering bot.jpeg" width="100%">   

### **About the resistors and power input**  
One of the methods for supplying power to the board is to connect the battery to a buck converter(make the voltage lower) and then supply to the LDO on the board.  

The 2 resistors circled in pink([Fig3](#fig3)) select which pin of the <u>battery input port</u>([Fig4](#fig4)) is going to connect with the <u>buck converter port</u>([Fig4](#fig4)). If you want to connect the 4s pin of the battery input port to the buck converter port, short the right pad([Fig3](#fig3)); if 3s, solder the left one.

#### **Fig3**
<img decoding="async" src="images/soldering power select.jpeg" width="100%"> 

#### **Fig4**
<img decoding="async" src="images/board pins completed.jpeg" width="100%">   


### **Voltage divider**  
If you are using commercial transmitters, you will not need to set up the voltage divider because you can not receive data from the flight controller.  
#### **Fig5**
<img decoding="async" src="images/soldering power logic.jpeg" width="100%">  

### **Gyro & Accelero**
[Fig6](#fig6) shows how MPU6050 is soldered.  
#### **Fig6**
<img decoding="async" src="images/soldering mpu6050.jpeg" width="100%">  
<br/><br/>

## **Software configuration**
Please follow this [tutorial](mds/esp_quad_tutorial_config.md) for more details.  
<br/><br/>

## **Upload the code**
Open Arduino IDE and set the development board like [Fig7](#fig7).  
#### **Fig7**
<img decoding="async" src="images/upload ide settings.jpeg" width="100%">  

Setup a circuit like [Fig8](#fig8). (The green thing is a switch, in order to upload the code, we need to pull IO0 of ESP32 to low when it boots).  
#### **Fig8**
<img decoding="async" src="images/upload circuit.jpeg" width="100%">
<br/><br/>

## **Note**
This is the default software orientation. Go [here](mds/esp_quad_tutorial_config.md) for more details.
<img decoding="async" src="images/orientation.jpeg" width="100%">
