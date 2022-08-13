#ifndef _wifiServerr_h_
#define _wifiServerr_h_

#include "config.h"
#include "wirelessServerHtml.h"
#include <EEPROM.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>


#define EEPROMaddress_pid_enable 72
#define EEPROMaddress_ota_enable 73
enum EEPROMaddress_pid {addr_a_px, addr_a_ix, addr_a_dx, addr_a_py, addr_a_iy, addr_a_dy, addr_a_pz, addr_a_iz, addr_a_dz};




class WIFISERVER_BASE{
    protected:
        uint EEPROMaddress_enable;
        const char* ssid;
        const char* password;
        bool enable = false;

    public:
        WIFISERVER_BASE(uint, const char*, const char*);

        void begin();

        void setEnable(bool val){ enable = val; }
        
        bool getEnable(){ return enable;}

        void check_if_enable_server();

        void check_if_go_to_setup(){
            static unsigned long timer_eeprom = millis();
            if(millis() - timer_eeprom > 1000){
                if(getEnable() == true){
                    EEPROM.writeBool(EEPROMaddress_enable, true);
                    EEPROM.commit();
                    ESP.restart(); // 复位  
                }  
                timer_eeprom = millis();
            }
        }

};


WIFISERVER_BASE::WIFISERVER_BASE(uint address, const char* sid, const char* pasword){
    EEPROMaddress_enable = address;
    ssid = sid;
    password = pasword;
}




class WIFISERVER_PID:public WIFISERVER_BASE{
    private:
        
    public:
        WIFISERVER_PID(uint a, const char* s, const char* p) : WIFISERVER_BASE(a, s, p){}
        void begin();
        void check_if_enable_server(){
            if(EEPROM.readBool(EEPROMaddress_enable) == true){
                EEPROM.writeBool(EEPROMaddress_enable, false);
                EEPROM.commit();
                begin();
            }
        }
};


void WIFISERVER_PID::begin(){
    AsyncWebServer server(80);
    
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);

    html_mid = "";
    html_mid += "px:" + (String)pidX.getPid(P) + "   ix:" + (String)pidX.getPid(I) + "   dx:" + (String)pidX.getPid(D);
    html_mid += "<br>";
    html_mid += "py:" + (String)pidY.getPid(P) + "   iy:" + (String)pidY.getPid(I) + "   dy:" + (String)pidY.getPid(D);
    html_mid += "<br>";
    html_mid += "pz:" + (String)pidZ.getPid(P) + "   iz:" + (String)pidZ.getPid(I) + "   dz:" + (String)pidZ.getPid(D);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", (String)html_top + html_mid + (String)html_bot);
    });

    server.onNotFound([](AsyncWebServerRequest *request){
        request->send(404, "text/plain", "Not found");
    });

    server.on("/exit", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", "exit");
        EEPROM.commit();
        EEPROM.end();
        ESP.restart();
    });

    server.on("/savee", HTTP_GET, [](AsyncWebServerRequest *request) {
        String s;

        if (request->hasParam("px")) {
            s = request->getParam("px")->value();
            pidX.setPid(s.toFloat(), pidX.getPid(I), pidX.getPid(D));
            EEPROM.writeFloat(addr_a_px*4, pidX.getPid(P));
        }

        else if (request->hasParam("ix")) {
            s = request->getParam("ix")->value();
            pidX.setPid(pidX.getPid(P), s.toFloat(), pidX.getPid(D));
            EEPROM.writeFloat(addr_a_ix*4, pidX.getPid(I));
        }

        else if (request->hasParam("dx")) {
            s = request->getParam("dx")->value();
            pidX.setPid(pidX.getPid(P),  pidX.getPid(I), s.toFloat());
            EEPROM.writeFloat(addr_a_dx*4, pidX.getPid(D));
        }


        else if (request->hasParam("py")) {
            s = request->getParam("py")->value();
            pidY.setPid(s.toFloat(), pidY.getPid(I), pidY.getPid(D));
            EEPROM.writeFloat(addr_a_py*4, pidY.getPid(P));
        }

        else if (request->hasParam("iy")) {
            s = request->getParam("iy")->value();
            pidY.setPid(pidY.getPid(P), s.toFloat(), pidY.getPid(D));
            EEPROM.writeFloat(addr_a_iy*4, pidY.getPid(I));
        }

        else if (request->hasParam("dy")) {
            s = request->getParam("dy")->value();
            pidY.setPid(pidY.getPid(P),  pidY.getPid(I), s.toFloat());
            EEPROM.writeFloat(addr_a_dy*4, pidY.getPid(D));
        }


        else if (request->hasParam("pz")) {
            s = request->getParam("pz")->value();
            pidZ.setPid(s.toFloat(), pidZ.getPid(I), pidZ.getPid(D));
            EEPROM.writeFloat(addr_a_pz*4, pidZ.getPid(P));
        }

        else if (request->hasParam("iz")) {
            s = request->getParam("iz")->value();
            pidZ.setPid(pidZ.getPid(P), s.toFloat(), pidZ.getPid(D));
            EEPROM.writeFloat(addr_a_iz*4, pidZ.getPid(I));
        }

        else if (request->hasParam("dz")) {
            s = request->getParam("dz")->value();
            pidZ.setPid(pidZ.getPid(P),  pidZ.getPid(I), s.toFloat());
            EEPROM.writeFloat(addr_a_dz*4, pidZ.getPid(D));
        }
        
        html_mid = "";
        html_mid += "px:" + (String)pidX.getPid(P) + "   ix:" + (String)pidX.getPid(I) + "   dx:" + (String)pidX.getPid(D);
        html_mid += "<br>";
        html_mid += "py:" + (String)pidY.getPid(P) + "   iy:" + (String)pidY.getPid(I) + "   dy:" + (String)pidY.getPid(D);
        html_mid += "<br>";
        html_mid += "pz:" + (String)pidZ.getPid(P) + "   iz:" + (String)pidZ.getPid(I) + "   dz:" + (String)pidZ.getPid(D);
        request->send(200, "text/html", (String)html_top + html_mid + (String)html_bot);
    });

    server.begin();
    while(true){};
}






class WIFISERVER_OTA:public WIFISERVER_BASE{
    private:
        enum EEPROMaddress_ota {addr_a_px, addr_a_ix, addr_a_dx, addr_a_py, addr_a_iy, addr_a_dy, addr_a_pz, addr_a_iz, addr_a_dz};
    public:
        WIFISERVER_OTA(uint a, const char* s, const char* p) : WIFISERVER_BASE(a, s, p){}
        void begin();
        void check_if_enable_server(){
            if(EEPROM.readBool(EEPROMaddress_enable) == true){
                EEPROM.writeBool(EEPROMaddress_enable, false);
                EEPROM.commit();
                begin();
            }
        }
};


void WIFISERVER_OTA::begin(){

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    delay(5000);
    ESP.restart();
  }

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  while(true) ArduinoOTA.handle();

}



WIFISERVER_PID serverPID(EEPROMaddress_pid_enable, serverNamePID, serverPassPID);
WIFISERVER_OTA serverOTA(EEPROMaddress_ota_enable, serverNameOTA, serverPassOTA);

#endif