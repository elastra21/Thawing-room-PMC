#include "WIFI.h"

void WIFI::setUpWiFi(){
  WiFi.begin(SECRET_SSID, SECRET_PASS);
  uint32_t notConnectedCounter = 0;
  // EEPROM.begin(32);
  while (WiFi.status() != WL_CONNECTED) {
    delay(2000);
    Serial.println("Wifi connecting...");
      
    notConnectedCounter++;
    if(notConnectedCounter > 150) { // Reset board if not connected after 5s
      Serial.println("Resetting due to Wifi not connecting...");
      // const uint8_t num_of_tries = EEPROM.readInt(1);
      // if (num_of_tries == 3) break;          
      // else {
        // EEPROM.writeInt(1, num_of_tries + 1);
        // EEPROM.commit();
        // EEPROM.end();
        // ESP.restart();          
      // }
      NVIC_SystemReset();
    }
  }

  // EEPROM.writeInt(1, 0);
  // EEPROM.commit();
  // EEPROM.end();

  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// void WIFI::setUpOTA(){
//   if(isConnected()){
//     ArduinoOTA.setHostname(HOST_NAME);
//     ArduinoOTA.onStart([]() {
//     String type;
//     type = ArduinoOTA.getCommand() == U_FLASH ? "sketch" : "filesystem";
//       // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
//     Serial.println("Start updating " + type);
//     }).onEnd([]() {
//       Serial.println("\nEnd");
//     }).onProgress([](unsigned int progress, unsigned int total) {
//       Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
//     }).onError([](ota_error_t error) {
//       Serial.printf("Error[%u]: ", error);
//       if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
//       else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
//       else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
//       else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
//       else if (error == OTA_END_ERROR) Serial.println("End Failed");
//     });
//     ArduinoOTA.begin();
//   }
// }

// void WIFI::loopOTA(){
//   ArduinoOTA.handle();
// }

bool WIFI::refreshWiFiStatus(){
  const bool connection = isConnected();
  if (connection != last_connection_state){
    last_connection_state = connection;
    return true;
  }
  return false;
}

bool WIFI::getConnectionStatus(){
  return last_connection_state;
}

bool WIFI::isConnected(){
  return WiFi.status() == WL_CONNECTED;
}

void WIFI::reconnect(){
  WiFi.begin(SECRET_SSID, SECRET_PASS);
  uint8_t timeout = 0;
  delay( 2000 );
  while ( WiFi.status() != WL_CONNECTED ){
    delay( 2000 );
    Serial.println(" waiting on wifi connection" );
    timeout++;
    if (timeout == 2) return;
  }
}



