#include <Arduino.h>

// Init Enviromnet Sensor
#include <Adafruit_AHTX0.h>
Adafruit_AHTX0 aht;

#include "WebPage.h"
#include  "credentials.h"
#include <Encoder.h>

#define ENCODER_SW D3
#define ENCODER_CA D5
#define ENCODER_CB D6
#define RELAYS D7

Encoder myEnc(ENCODER_CA, ENCODER_CB);
long oldPosition = -999;

#include <ElegantOTA.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266WiFi.h>
#include <ArduinoHA.h>
#include <Adafruit_NeoPixel.h>

#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

unsigned long ota_progress_millis = 0;
ESP8266WebServer server(80);

// OTA
void onOTAStart()
{
  Serial.println("OTA update started!");
}

void onOTAProgress(size_t current, size_t final)
{
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000)
  {
    ota_progress_millis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success)
{
  // Log when OTA has finished
  if (success)
  {
    Serial.println("OTA update finished successfully!");
  }
  else
  {
    Serial.println("There was an error during OTA update!");
  }
  // <Add your own code here>
}

void setupOTA()
{

  // ota
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  server.on("/", []()
            { server.send(200, "text/html", MAIN_page); });

  ElegantOTA.begin(&server); // Start ElegantOTA
  // ElegantOTA callbacks
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);

  server.begin();
  Serial.println("HTTP server started");
}

#define MAXTEMP 45
#define MINTEMP 10

#define MAXTIME 10000
#define MINTIME 0

double airTemp = 0;
double irTemp = 0;
double airHum = 0;
double sensorTemperatur = 24;
double targetTemperatur = 24;
unsigned int timeInMin = 20;
int active = 0;

enum menue
{
  setTemp,
  setTime,
  setState
};

enum menue Heater = setTemp;



int hum;

void setupIrTemp()
{
  while (!mlx.begin())
  {
    // Serial.println("Error connecting to MLX sensor. Check wiring.");
    int counter = 0;
    counter++;
    if (counter % 1000 == 0)
    {
      Serial.print(".");
    }
  }
}

void readIrTemp()
{
  irTemp = mlx.readObjectTempC(); 
}

void sendState()
{
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data

  airTemp = temp.temperature;
  airHum = humidity.relative_humidity; 
}

void setupEncoder()
{  pinMode(ENCODER_SW, INPUT);
  pinMode(RELAYS, OUTPUT);
  digitalWrite(RELAYS, false);
}

void setupServer()
{
  byte mac[WL_MAC_ADDR_LENGTH];
  WiFi.macAddress(mac);
  Serial.print("Setting up WIFI device:  [ ");
  for (int i = 0; i < WL_MAC_ADDR_LENGTH; i++)
  {
    Serial.print(mac[i], HEX);
    Serial.print(" ");
  }
  Serial.println("]");

   WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(100); // waiting for the connection
  }
  Serial.println();
  Serial.println("Connected to the network");
  Serial.println(WiFi.localIP());
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting BROT O MAT 2000");
  setupServer();
  setupOTA();


  Serial.println("Initializing temperature and humidity sensor!");
  if (!aht.begin())
  {
    Serial.println("Could not find Sensor? Check wiring");
    while (1)
      delay(10);
  }
  setupIrTemp();
  setupEncoder();
}

unsigned long TS = 0;
unsigned long DT = 1000;
unsigned long fTS = 0;
unsigned long fDT = 1000;

bool EncoderLastState = true;
unsigned long encoderTS = 0;
unsigned long encoderDB = 10;
void encoderCheckChange()
{
  bool newState = digitalRead(ENCODER_SW);
  if (newState == EncoderLastState)
  {
    // nothing
  }
  else
  {

    if (millis() - encoderTS > encoderDB)
    {
      EncoderLastState = newState;
      encoderTS = millis();
      if (newState)
      {
        Serial.println("Push");

        switch (Heater)
        {
        case setTemp:
          Heater = setTime;
          break;
        case setTime:
          Heater = setState;
          break;
        case setState:
          Heater = setTemp;
          break;

        default:
          break;
        }
      }
      else
      {
        Serial.println("Release");
      }
    }
  }
}

unsigned long encoderRoTS = 0;
unsigned long encoderRoDB = 500;

int increment = 0;
void stateMashine()
{
  if (millis() - encoderRoTS > encoderRoDB)
  {
    encoderRoTS = millis();

    increment = -(myEnc.readAndReset() / 4);
    switch (Heater)
    {
    case setTime:

      timeInMin += increment * 5;
      timeInMin = constrain(timeInMin, MINTIME, MAXTIME);     

      break;
    case setTemp:
      targetTemperatur += increment;
      targetTemperatur = constrain(targetTemperatur, MINTEMP, MAXTEMP);     
      break;
    case setState:

      if (increment > 0)
      {
        active = true;
        //   Serial.println("Der Ofen an ");
      }
      else if (increment < 0)
      {
        active = false;
        // Serial.println("Der Ofen aus ");
      }
      break;

    default:
      //  Serial.println("error in state mashine");
      break;
    }
  }
}
bool heaterState = false;
void heatControler()
{
  heaterState = false;
  if (active)
  {
    if (targetTemperatur > irTemp)
    {

      if (sensorTemperatur < 80)
      {
        heaterState = true;
        }
    } else {  }
  }  else  {  }
  digitalWrite(RELAYS, heaterState);
}

int seconds = 0;
void timer()
{
  if (active)
  {
    seconds--;
    if (seconds <= 0)
    {
      timeInMin--;
      seconds = 60;
    }

    timeInMin = constrain(timeInMin, MINTIME, MAXTIME);
  }
  if (timeInMin <= 0)
  {

    active = false;
  }
}

int Ro = 10, B = 3950; // Nominal resistance 50K, Beta constant
int Rseries = 10;      // Series resistor 10K
float To = 298.15;     // Nominal Temperature

double NTCTemp()
{
  /*Read analog outputof NTC module,
    i.e the voltage across the thermistor */
  float Vi = analogRead(A0) * (3.3 / 1023.0);
  // Convert voltage measured to resistance value
  // All Resistance are in kilo ohms.
  float R = (Vi * Rseries) / (3.3 - Vi);
  /*Use R value in steinhart and hart equation
    Calculate temperature value in kelvin*/
  float T = 1 / ((1 / To) + ((log(R / Ro)) / B));
  float Tc = T - 273.15; // Converting kelvin to celsius
  // Serial.println((String)"Temperature in celsius    :" + Tc + "°C");
  sensorTemperatur = Tc;
  return Tc;
}

void loop()
{
   // OTA Loop
  server.handleClient();
  ElegantOTA.loop();

  
  encoderCheckChange();
  if (millis() - TS > DT)
  {
    TS = millis();
    sendState();
    readIrTemp();
    timer();
  }
  stateMashine();
  heatControler();

  if (millis() - fTS > fDT)
  {
    fTS = millis();
    Serial.print(">Oven:");
    if (active)
    {
      Serial.print("1");
    }
    else
    {
      Serial.print("0");
    }
    Serial.print(",State:");

    Serial.print(Heater);
    Serial.print(",Heater:");
    Serial.print(heaterState);

    Serial.print(",TargetTemp:");
    Serial.print(targetTemperatur);
    Serial.print(",AirTemp:");
    Serial.print(airTemp);
    Serial.print(",IrTemp:");
    Serial.print(irTemp);
    Serial.print(",AirHum:");
    Serial.print(airHum);
    Serial.print(",Time:");
    Serial.print(timeInMin);
    //  Serial.print("Nt:");
    // Serial.print(analogRead(A0));
    Serial.print(",SensorTemp:");
    Serial.print(NTCTemp());
    Serial.println("");
  }
}
