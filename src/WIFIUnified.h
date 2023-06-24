#ifndef MY_WIFI_H
#define MY_WIFI_H
#include <WiFi.h>
#include "secrets.h"
#include <Arduino.h>
#include "secrets.h"
#include "web/index.h"
#include <Arduino_Portenta_OTA.h>
#include <Portenta_H7_AsyncWebServer.h>

#define PORT 4000
#define OTA_FILE_LOCATION "http://192.168.100.53:4000/download"

class WIFIUnified {
  public:
    void localIP();
    void setUpOTA();
    void setUpWiFi();
    void reconnect();
    bool isConnected();
    void checkVersion();                                /* added for portenta support*/
    void setUpWebServer();
    bool refreshWiFiStatus();
    bool getConnectionStatus();
  private:
    WiFiClient client;
    bool last_connection_state = false;
    void update();                                      /* added for portenta support*/  
    String decodeVersion(const String& message);        /* added for portenta support*/
};
#endif
