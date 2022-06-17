#ifndef _ble_h_
#define _ble_h_

#include "config.h"
#include <EEPROM.h>
#include <BluetoothSerial.h>

#define px ('p' * 1000 + 'x')
#define ix ('i' * 1000 + 'x')
#define dx ('d' * 1000 + 'x')

#define py ('p' * 1000 + 'y')
#define iy ('i' * 1000 + 'y')
#define dy ('d' * 1000 + 'y')

#define pz ('p' * 1000 + 'z')
#define iz ('i' * 1000 + 'z')
#define dz ('d' * 1000 + 'z')


#define addr_ble_enable 72
enum pidAddress {addr_a_px, addr_a_ix, addr_a_dx, addr_a_py, addr_a_iy, addr_a_dy, addr_a_pz, addr_a_iz, addr_a_dz, 
                 addr_g_px, addr_g_ix, addr_g_dx, addr_g_py, addr_g_iy, addr_g_dy, addr_g_pz, addr_g_iz, addr_g_dz};


BluetoothSerial SerialBT;


void ble_zip(String);


bool bleEnable = false;
void setBleEnable(bool value){
    bleEnable = value;
}
bool getBleEnable(){
    return bleEnable;
}

void server_ble_eeprom(){
    unsigned long timer_ble = millis();
    bool connected = false;
    
    SerialBT.begin(bleServerName);

    while(true){
        String s = " ";
        if(SerialBT.available() > 0) {
            connected = true;
            s = SerialBT.readStringUntil('\n');
            s.remove(s.length()-1, 1);
            if(s == "Exit") break;
        }
        // 如果15秒内没接收到数据则退出
        if(not(connected) and millis()-timer_ble > 15000) break;
        ble_zip(s); // zip
        delay(10);
    }
    ESP.restart();
}


void ble_zip(String s){

    String output = "";
    static String select;
    if(s == "Save") EEPROM.commit();
    if(s == "Save") SerialBT.println("saved !!!");

    if(s == "Angle"){
        select = "Angle";
        output += "px: " + String(pidX.getPid(P));
        output += " ix: " + String(pidX.getPid(I));
        output += " dx: " + String(pidX.getPid(D));
        output += " py: " + String(pidY.getPid(P));
        output += " iy: " + String(pidY.getPid(I));
        output += " dy: " + String(pidY.getPid(D));
        output += " pz: " + String(pidZ.getPid(P));
        output += " iz: " + String(pidZ.getPid(I));
        output += " dz: " + String(pidZ.getPid(D));
        SerialBT.println(output);       
        output = "";
    }

    if(select == "Angle"){
        int slen = s.length();
        int temp  = s[0] * 1000 + s[1];

        switch (temp) {
            case px:
                s = s.substring(2, slen);
                pidX.setPid(s.toFloat(), pidX.getPid(I), pidX.getPid(D));
                EEPROM.writeFloat(addr_a_px*4, pidX.getPid(P));
                break;
            case ix:
                s = s.substring(2, slen);
                pidX.setPid(pidX.getPid(P), s.toFloat(), pidX.getPid(D));
                EEPROM.writeFloat(addr_a_ix*4, pidX.getPid(I));
                break;
            case dx:
                s = s.substring(2, slen);
                pidX.setPid(pidX.getPid(P),  pidX.getPid(I), s.toFloat());
                EEPROM.writeFloat(addr_a_dx*4, pidX.getPid(D));
                break;

            case py:
                s = s.substring(2, slen);
                pidY.setPid(s.toFloat(), pidY.getPid(I), pidY.getPid(D));
                EEPROM.writeFloat(addr_a_py*4, pidY.getPid(P));
                break;
            case iy:
                s = s.substring(2, slen);
                pidY.setPid(pidY.getPid(P), s.toFloat(), pidY.getPid(D));
                EEPROM.writeFloat(addr_a_iy*4, pidY.getPid(I));
                break;
            case dy:
                s = s.substring(2, slen);
                pidY.setPid(pidY.getPid(P),  pidY.getPid(I), s.toFloat());
                EEPROM.writeFloat(addr_a_dy*4, pidY.getPid(D));
                break;

            case pz:
                s = s.substring(2, slen);
                pidZ.setPid(s.toFloat(), pidZ.getPid(I), pidZ.getPid(D));
                EEPROM.writeFloat(addr_a_pz*4, pidZ.getPid(P));
                break;
            case iz:
                s = s.substring(2, slen);
                pidZ.setPid(pidZ.getPid(P), s.toFloat(), pidZ.getPid(D));
                EEPROM.writeFloat(addr_a_iz*4, pidZ.getPid(I));
                break;
            case dz:
                s = s.substring(2, slen);
                pidZ.setPid(pidZ.getPid(P),  pidZ.getPid(I), s.toFloat());
                EEPROM.writeFloat(addr_a_dz*4, pidZ.getPid(D));
                break;
        }
        if(temp == px or temp == ix or temp == dx){
            output += "px: " + String(pidX.getPid(P));
            output += "  ix: " + String(pidX.getPid(I));
            output += "  dx: " + String(pidX.getPid(D));  
            SerialBT.println(output);      
        }
        if(temp == py or temp == iy or temp == dy){
            output += "py: " + String(pidY.getPid(P));
            output += "  iy: " + String(pidY.getPid(I));
            output += "  dy: " + String(pidY.getPid(D));  
            SerialBT.println(output);      
        }
        if(temp == pz or temp == iz or temp == dz){
            output += "pz: " + String(pidZ.getPid(P));
            output += "  iz: " + String(pidZ.getPid(I));
            output += "  dz: " + String(pidZ.getPid(D));  
            SerialBT.println(output);   
        }
    }

    if(s == "Gyro"){
        select = "Gyro";
        output += "px: " + String(pidGX.getPid(P));
        output += " ix: " + String(pidGX.getPid(I));
        output += " dx: " + String(pidGX.getPid(D));
        output += " py: " + String(pidGY.getPid(P));
        output += " iy: " + String(pidGY.getPid(I));
        output += " dy: " + String(pidGY.getPid(D));
        output += " pz: " + String(pidGZ.getPid(P));
        output += " iz: " + String(pidGZ.getPid(I));
        output += " dz: " + String(pidGZ.getPid(D));
        SerialBT.println(output);       
        output = "";
    }

    if(select == "Gyro"){
        int slen = s.length();
        int temp  = s[0] * 1000 + s[1];

        switch (temp) {
            case px:
                s = s.substring(2, slen);
                pidGX.setPid(s.toFloat(), pidGX.getPid(I), pidGX.getPid(D));
                EEPROM.writeFloat(addr_g_px*4, pidGX.getPid(P));
                break;
            case ix:
                s = s.substring(2, slen);
                pidGX.setPid(pidGX.getPid(P), s.toFloat(), pidGX.getPid(D));
                EEPROM.writeFloat(addr_g_ix*4, pidGX.getPid(I));
                break;
            case dx:
                s = s.substring(2, slen);
                pidGX.setPid(pidGX.getPid(P),  pidGX.getPid(I), s.toFloat());
                EEPROM.writeFloat(addr_g_dx*4, pidGX.getPid(D));
                break;

            case py:
                s = s.substring(2, slen);
                pidGY.setPid(s.toFloat(), pidGY.getPid(I), pidGY.getPid(D));
                EEPROM.writeFloat(addr_g_py*4, pidGY.getPid(P));
                break;
            case iy:
                s = s.substring(2, slen);
                pidGY.setPid(pidGY.getPid(P), s.toFloat(), pidGY.getPid(D));
                EEPROM.writeFloat(addr_g_iy*4, pidGY.getPid(I));
                break;
            case dy:
                s = s.substring(2, slen);
                pidGY.setPid(pidGY.getPid(P),  pidGY.getPid(I), s.toFloat());
                EEPROM.writeFloat(addr_g_dy*4, pidGY.getPid(D));
                break;

            case pz:
                s = s.substring(2, slen);
                pidGZ.setPid(s.toFloat(), pidGZ.getPid(I), pidGZ.getPid(D));
                EEPROM.writeFloat(addr_g_pz*4, pidGZ.getPid(P));
                break;
            case iz:
                s = s.substring(2, slen);
                pidGZ.setPid(pidGZ.getPid(P), s.toFloat(), pidGZ.getPid(D));
                EEPROM.writeFloat(addr_g_iz*4, pidGZ.getPid(I));
                break;
            case dz:
                s = s.substring(2, slen);
                pidGZ.setPid(pidGZ.getPid(P),  pidGZ.getPid(I), s.toFloat());
                EEPROM.writeFloat(addr_g_dz*4, pidGZ.getPid(D));
                break;
        }
        if(temp == px or temp == ix or temp == dx){
            output += "px: " + String(pidGX.getPid(P));
            output += "  ix: " + String(pidGX.getPid(I));
            output += "  dx: " + String(pidGX.getPid(D));  
            SerialBT.println(output);      
        }
        if(temp == py or temp == iy or temp == dy){
            output += "py: " + String(pidGY.getPid(P));
            output += "  iy: " + String(pidGY.getPid(I));
            output += "  dy: " + String(pidGY.getPid(D));  
            SerialBT.println(output);      
        }
        if(temp == pz or temp == iz or temp == dz){
            output += "pz: " + String(pidGZ.getPid(P));
            output += "  iz: " + String(pidGZ.getPid(I));
            output += "  dz: " + String(pidGZ.getPid(D));  
            SerialBT.println(output);   
        }
    }

}


//初始化
void bleEEPROMinit(){
    for(int i=addr_a_px; i<(addr_g_dz); i++) EEPROM.writeFloat(i*4, 0.0);
    EEPROM.commit();
}

void readBleEEPROM(){
    float read_temp[9];
    for(int i=addr_a_px; i<addr_a_dz+1; i++) read_temp[i] = EEPROM.readFloat(i * 4);
    pidX.setPid(read_temp[0], read_temp[1], read_temp[2]);
    pidY.setPid(read_temp[3], read_temp[4], read_temp[5]);
    pidZ.setPid(read_temp[6], read_temp[7], read_temp[8]);
    float read_temp2[9];
    for(int i=addr_a_px; i<addr_a_dz+1; i++) read_temp2[i] = EEPROM.readFloat((addr_a_dz+1)*4 + (i * 4));
    pidGX.setPid(read_temp2[0], read_temp2[1], read_temp2[2]);
    pidGY.setPid(read_temp2[3], read_temp2[4], read_temp2[5]);
    pidGZ.setPid(read_temp2[6], read_temp2[7], read_temp2[8]);
}


void bleEEPROMcheckSetup(){
    // 如果地址addr_ble_enable的值是ture，进入蓝牙调参模式
    // 具体如何进入可以在主程序的task_reset()找到
    if(EEPROM.readBool(addr_ble_enable) != false) {
        Serial.println("\n\nstart ble server!!!");
        EEPROM.writeBool(addr_ble_enable, false);
        EEPROM.commit();
        server_ble_eeprom();
    }
}

void bleEEPROMcheckLoop(){
    static unsigned long timer_eeprom = millis();
    // 每隔一秒查看是否进入调参
    if(millis() - timer_eeprom > 1000){
        if(getBleEnable() == true){
            EEPROM.writeBool(addr_ble_enable, true);
            EEPROM.commit();
            ESP.restart(); // 复位  
        }  
        timer_eeprom = millis();
    }
}

#endif