#include <Arduino.h>

// Init Enviromnet Sensor
#include <Adafruit_AHTX0.h>
Adafruit_AHTX0 aht;

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

ESP8266WebServer server(80);

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
  // Serial.print("Temperature: ");
  // Serial.print(tempObjF);
  // Serial.println(" degrees F");
}

void sendState()
{
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data

  airTemp = temp.temperature;
  airHum = humidity.relative_humidity;
  /*
  Serial.print("Temperature: ");
  Serial.print(airTemp);
  Serial.println(" degrees F");
  // Temp.setValue(temp.temperature);
  Serial.print("Humidity: ");
  Serial.print(airHum);
  Serial.println("% rH");
  */
  // Hum.setValue(humidity.relative_humidity);
}
/*
// int turnCounter=0;
int increment = 0;

IRAM_ATTR void ISR()
{
  bool CB = digitalRead(ENCODER_CB);
 // bool CA = digitalRead(ENCODER_CA);
  if (CB)
  {
    increment = +1;
  }
  else
  {
    increment = -1;
  }
}
*/
void setupEncoder()
{

  pinMode(ENCODER_SW, INPUT);
  pinMode(RELAYS, OUTPUT);
  digitalWrite(RELAYS, false);
  // pinMode(ENCODER_CB, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(ENCODER_CA), ISR, RISING);
}

// int16_t inputDelta = 0; // Counts up or down depending which way the encoder is turned
// bool printFlag = false; // Flag to indicate that the value of inputDelta should be printed

/*
void readEncoder() {
    static uint8_t state = 0;
    bool CLKstate = digitalRead(ENCODER_CA);
    bool DTstate = digitalRead(ENCODER_CB);
    switch (state) {
        case 0:                         // Idle state, encoder not turning
            if (!CLKstate){             // Turn clockwise and CLK goes low first
                state = 1;
            } else if (!DTstate) {      // Turn anticlockwise and DT goes low first
                state = 4;
            }
            break;
        // Clockwise rotation
        case 1:
            if (!DTstate) {             // Continue clockwise and DT will go low after CLK
                state = 2;
            }
            break;
        case 2:
            if (CLKstate) {             // Turn further and CLK will go high first
                state = 3;
            }
            break;
        case 3:
            if (CLKstate && DTstate) {  // Both CLK and DT now high as the encoder completes one step clockwise
                state = 0;
                ++inputDelta;
                printFlag = true;
            }
            break;
        // Anticlockwise rotation
        case 4:                         // As for clockwise but with CLK and DT reversed
            if (!CLKstate) {
                state = 5;
            }
            break;
        case 5:
            if (DTstate) {
                state = 6;
            }
            break;
        case 6:
            if (CLKstate && DTstate) {
                state = 0;
                --inputDelta;
                printFlag = true;
            }
            break;
    }
}
*/

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting BROT O MAT 2000");
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
      //  Serial.print("NewTime: ");
      // Serial.println(timeInMin);

      break;
    case setTemp:
      targetTemperatur += increment;
      targetTemperatur = constrain(targetTemperatur, MINTEMP, MAXTEMP);
      // Serial.print("New Target Temp: ");
      // Serial.println(targetTemperatur);
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
     

      // digitalWrite(RELAYS,true);
    }
    else
    {
      
      //heaterState = false;
    }
  }
  else
  {
   // heaterState = false;
  }

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
  encoderCheckChange();

  // long newPosition = myEnc.read();
  /* if (newPosition != oldPosition)
   {
     oldPosition = newPosition;
     Serial.println(newPosition);
   }*/

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
