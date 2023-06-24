#include "WIFIUnified.h"

AsyncWebServer server(80);
IPAddress IP(192,168,100,53);  // IP address for example.com (no DNS)

Arduino_Portenta_OTA_QSPI ota(QSPI_FLASH_FATFS_MBR, 2);
Arduino_Portenta_OTA::Error ota_err = Arduino_Portenta_OTA::Error::None;

void handleNotFound(AsyncWebServerRequest *request) {
	String message = "File Not Found\n\n";

	message += "URI: ";
	//message += server.uri();
	message += request->url();
	message += "\nMethod: ";
	message += (request->method() == HTTP_GET) ? "GET" : "POST";
	message += "\nArguments: ";
	message += request->args();
	message += "\n";

	for (uint8_t i = 0; i < request->args(); i++) {
		message += " " + request->argName(i) + ": " + request->arg(i) + "\n";
	}
	request->send(404, "text/plain", message);
}

void WIFIUnified::setUpWiFi(){
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

void WIFIUnified::setUpOTA(){
  if (!ota.isOtaCapable()){
    Serial.println("Higher version bootloader required to perform OTA.");
    Serial.println("Please update the bootloader.");
    Serial.println("File -> Examples -> STM32H747_System -> STM32H747_updateBootloader");
    return;
  }

  Serial.println("Initializing OTA storage");
  if ((ota_err = ota.begin()) != Arduino_Portenta_OTA::Error::None){
    Serial.print  ("Arduino_Portenta_OTA::begin() failed with error code ");
    Serial.println((int)ota_err);
    return;
  }
}

void WIFIUnified::checkVersion(){
  if (client.connect(IP, 4000)) {
  Serial.println("connected to IP");
  client.println("GET /version HTTP/1.1");
  client.print("Host: ");
  client.println(IP);
  client.println("Connection: close");
  client.println();
}else{
  Serial.println("connection failed");
  return;
}

bool headersReceived = false;

while (!client.available()) delay(20);

while (client.available()) {
  if (!headersReceived && client.read() == '\r' && client.read() == '\n' && client.read() == '\r' && client.read() == '\n') {
    headersReceived = true;
    Serial.println();
  } 
  else if (headersReceived) {
    const String version = client.readStringUntil('\n');
    const String decodedVersion = decodeVersion(version);

    if (decodedVersion == VERSION) Serial.println("Version is up to date");
    else {
      Serial.println("Version is not up to date");
      update();
    }
  }
}

if (!client.connected()) client.stop();
}

void WIFIUnified::update(){
  Serial.println("Updating...");
  Serial.println("Starting download to QSPI ...");
  int const ota_download = ota.download(OTA_FILE_LOCATION, false /* is_https */);
  if (ota_download <= 0){
    Serial.print  ("Arduino_Portenta_OTA_QSPI::download failed with error code ");
    Serial.println(ota_download);
    return;
  }
  Serial.print  (ota_download);
  Serial.println(" bytes stored.");


  Serial.println("Decompressing LZSS compressed file ...");
  int const ota_decompress = ota.decompress();
  if (ota_decompress < 0){
    Serial.print("Arduino_Portenta_OTA_QSPI::decompress() failed with error code");
    Serial.println(ota_decompress);
    return;
  }
  Serial.print(ota_decompress);
  Serial.println(" bytes decompressed.");


  Serial.println("Storing parameters for firmware update in bootloader accessible non-volatile memory ...");
  if ((ota_err = ota.update()) != Arduino_Portenta_OTA::Error::None){
    Serial.print  ("ota.update() failed with error code ");
    Serial.println((int)ota_err);
    return;
  }

  Serial.println("Performing a reset after which the bootloader will update the firmware.");
  Serial.println("Hint: Portenta H7 LED will blink Red-Blue-Green.");
  delay(1000); /* Make sure the serial message gets out before the reset. */
  ota.reset();
}

String WIFIUnified::decodeVersion(const String& message){
  int startIdx = message.indexOf("\"version\":\"");
  if (startIdx == -1) return "";  // Version not found
  
  startIdx += 11;  // Move past the "version":" part
  
  int endIdx = message.indexOf("\"", startIdx);
  if (endIdx == -1)  return "";  // Invalid message format
  
  return message.substring(startIdx, endIdx);
}

bool WIFIUnified::refreshWiFiStatus(){
  const bool connection = isConnected();
  if (connection != last_connection_state){
    last_connection_state = connection;
    return true;
  }
  return false;
}

bool WIFIUnified::getConnectionStatus(){
  return last_connection_state;
}

bool WIFIUnified::isConnected(){
  return WiFi.status() == WL_CONNECTED;
}

void WIFIUnified::reconnect(){
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

void WIFIUnified::setUpWebServer(){
  server.on("/style.css", [](AsyncWebServerRequest * request) {
		request->send(200, "text/css", styleCSS);
	});

	server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
		request->send(200, "text/html", indexHTML);
	});

	server.on("/serverIndex", HTTP_GET, [](AsyncWebServerRequest *request) {
    	request->send(200, "text/html", serverIndexHTML);
  	});
  	/*handling uploading firmware file */
  	server.on("/reset", HTTP_POST, [](AsyncWebServerRequest *request) {
    	request->send(200, "text/plain", "Resetting...");
    	// ESP.restart(); 
		NVIC_SystemReset();
  	});

	server.onNotFound(handleNotFound);

	server.begin();

	// Serial.print("HTTP Async_HelloServer2 started @ IP : ");
	Serial.print("@IP : ");
	Serial.println(WiFi.localIP());
}






