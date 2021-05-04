/*
   This code is used for getting the sensor readings using Arduino.

   You can find code for running air quality sensors:
   - MQ135 (Benzene, Alcohol, smoke)
   - MQ6 (LPG, butane gas)
   - MQ4 (Methane, CNG Gas)
   - MQ7 (Carbon Monoxide)
   - SGP30 (Co2)
   And temperature and humidity sensor:
   - DHT11
*/

#include <Wire.h>
#include "Adafruit_SGP30.h"
#include "DHTesp.h"

#define PINMQ135 13
#define PINMQ6 25
#define PINMQ4 26
#define PINMQ7 27
#define PINDHT 32      // Pin for DHT readings
#define DHTTYPE DHT11 // Defines sensor type

DHTesp dht;                           // Constructor for DHT
TaskHandle_t tempTaskHandle = NULL;   // Task handle for the light value read task
Adafruit_SGP30 sgp;                   // Constructor for SGP30

// Sensor readings variables
float temp = 0.0;
float hum = 0.0;
float tvoc = 0.0;
float eco2 = 0.0;
int sensorMQ135 = 0.0;
int sensorMQ6 = 0.0;
int sensorMQ4 = 0.0;
int sensorMQ7 = 0.0;

// SGP30 must have its baseline values.
// Sensor must be recalibrated befor usage.
// Before callibration, sensor must work for at least 24h.
bool calibrate = false;
uint16_t TVOC_base, eCO2_base;

/**
   Project setup.

   Function that runs only once.

*/
void setup()
{
  Wire.begin();
  Serial.begin(115200);

  pinMode(PINMQ135, INPUT);
  pinMode(PINMQ6, INPUT);
  pinMode(PINMQ4, INPUT);
  pinMode(PINMQ7, INPUT);

  // Starts DTH11 sensor
  dht.setup(PINDHT, DHTesp::DHT11);

  // Starts SGP30 sensor
  while (!sgp.begin()) {
    Serial.println("SGP30 not found!");
    delay(1000);
  }
  Serial.println("SGP30 OK!");

  // Get temperature and humidity and set values to increase the precision of SGP sensor
  GetTempHum(false);
  sgp.setHumidity(getAbsoluteHumidity(temp, hum));

  // SGP30 must have its baseline values.
  // Sensor must be recalibrated befor usage.
  // Before callibration, sensor must work for at least 24h.
  //sgp.setIAQBaseline(0x0, 0x0);
}

/**
   Main code.
*/
void loop()
{
  //  if (calibrate)
  //  {
  //    CalibrateSgp(); // Calibrates SGP sensor
  //  }
  //  calibrate = false;


  GetSgp(true);        // Gets SGP readings
  GetTempHum(true);    // Gets temperature and humidity readings
  GetAllSensors(true); // Gets readings from all MQX sensors

  Serial.println("#################################################");

  delay(3000);
  // exit(0);
}

/**
   Converts ESP32 dev board readings to voltage (3.3V pins)

   @param reading ESP32 reading (0-4095)
   @return (float) Real voltage on pin
*/
double ToVolt(int reading)
{
  return reading / 4096.0 * 3.3;
}

/**
   Gets temperature and humidity sensor readings

   All values are saved to global variables at the top of this document

   @param plot If true, results will be shown in Serial window
*/
void GetTempHum(bool plot)
{
  TempAndHumidity lastValues = dht.getTempAndHumidity();
  delay(100);
  lastValues = dht.getTempAndHumidity();

  temp = lastValues.temperature;
  hum = lastValues.humidity;

  if (isnan(hum) || isnan(temp))
  {
    Serial.println("Failed to read from DHT sensor!");
  }

  if (plot)
  {
    Serial.println("Temp: " + String(temp) + " °C");
    Serial.println("Hum: " + String(hum) + " %");
  }
}

/**
   Gets readings from all MQX sensors

   All values are saved to global variables at the top of this document

   @param plot If true, results will be shown in Serial window
*/

void GetAllSensors(bool plot)
{
  sensorMQ135 = analogRead(PINMQ135);
  sensorMQ6 = analogRead(PINMQ6);
  sensorMQ4 = analogRead(PINMQ4);
  sensorMQ7 = analogRead(PINMQ7);

  if (plot)
  {
    Serial.println("MQ135 read: " + String(sensorMQ135) + " U: " + String(ToVolt(sensorMQ135)) + " V");
    Serial.println("MQ6 read: " + String(sensorMQ6) + " U: " + String(ToVolt(sensorMQ6)) + " V");
    Serial.println("MQ4 read: " + String(sensorMQ4) + " U: " + String(ToVolt(sensorMQ4)) + " V");
    Serial.println("MQ7 read: " + String(sensorMQ7) + " U: " + String(ToVolt(sensorMQ7)) + " V");
  }
}

/**
 * Calibrates SGP30 sensor
 * 
 * Sets and prints SGP30 baseline values
 *  
 */
void CalibrateSgp()
{

  if (!sgp.getIAQBaseline(&eCO2_base, &TVOC_base))
  {
    Serial.println("Failed to get baseline SGP");
    exit(1);
  }
  else
  {
    Serial.print("Baseline -> eCO2: 0x");
    Serial.print(eCO2_base, HEX);
    Serial.print(" & TVOC: 0x");
    Serial.println(TVOC_base, HEX);
    // TODO: set baseline vals
    // sgp.setIAQBaseline(0x0, 0x0);
  }
  delay(1000);
}

/**
 * Gets readings from SGP30 sensor
 * 
 * All values are saved to global variables at the top of this document
 * 
 * @param plot If true, results will be shown in Serial window 
 */
void GetSgp(bool plot)
{
  // Check if SGP30 can get sensor reading
  if (!sgp.IAQmeasure())
  {
    Serial.println("SGP30 read failed!");
    return;
  }
  else
  {
    tvoc = sgp.TVOC;
    eco2 = sgp.eCO2;
  }
  if (plot)
  {
    Serial.println("TVOC: " + String(tvoc) + " ppb");
    Serial.println("eCO2: " + String(eco2) + " ppm");
  }
}

// Borowed from https://github.com/Makerfabs/Project_Touch-Camera-ILI9341/tree/master/example/CO2_Monitor
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
  // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
  return absoluteHumidityScaled;
}