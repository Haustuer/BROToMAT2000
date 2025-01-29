#include <Arduino.h>

// Init Enviromnet Sensor
#include <Adafruit_AHTX0.h>
Adafruit_AHTX0 aht;
#include <ElegantOTA.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h> 
#include <ESP8266WiFi.h>
#include <ArduinoHA.h>
#include <Adafruit_NeoPixel.h>

unsigned long ota_progress_millis = 0;

ESP8266WebServer server(80);

void setup() {
 
 
}

void loop() {
  
}
