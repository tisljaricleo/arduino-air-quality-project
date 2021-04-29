/*
 * This code is used for getting the sensor readings using Arduino.
 *
 * You can find code for running air quality sensors:
 * - MQ135 (Benzene, Alcohol, smoke)
 * - MQ6 (LPG, butane gas)
 * - MQ4 (Methane, CNG Gas)
 * - MQ7 (Carbon Monoxide)
 * - SGP30 (Co2)
 * And temperature and humidity sensor:
 * - DHT11
 */

#include <Wire.h>
#include "Adafruit_SGP30.h"
#include "DHT.h"

#define PINMQ135 0
#define PINMQ6 1
#define PINMQ4 2
#define PINMQ7 3
#define PINDHT 2      // Pin for DHT readings
#define DHTTYPE DHT11 // Defines sensor type

DHT dht(PINDHT, DHTTYPE); // Constructor for DHT
Adafruit_SGP30 sgp;       // Constructor for SGP30

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
bool calibrate = true;
uint16_t TVOC_base, eCO2_base;

/**
 * Project setup.
 * 
 * Function that runs only once.
 *  
 */
void setup()
{
  Serial.begin(9600);

  pinMode(PINMQ135, INPUT);
  pinMode(PINMQ6, INPUT);
  pinMode(PINMQ4, INPUT);
  pinMode(PINMQ7, INPUT);

  // Starts DTH11 sensor
  dht.begin();

  // Starts SGP30 sensor
  if (!sgp.begin())
  {
    Serial.println(F("SGP30 not found!"));
    while (1)
      ; // shut down program
  }
  Serial.println(F("SGP30 OK!"));

  // SGP30 must have its baseline values.
  // Sensor must be recalibrated befor usage.
  // Before callibration, sensor must work for at least 24h.
  sgp.setIAQBaseline(0x0, 0x0);
}

/**
 * Main code. 
 */
void loop()
{

  if (calibrate)
  {
    CalibrateSgp(); // Calibrates SGP sensor
  }
  calibrate = false;

  GetSgp(true);        // Gets SGP readings
  GetTempHum(true);    // Gets temperature and humidity readings
  GetAllSensors(true); // Gets readings from all MQX sensors

  Serial.println("#################################################");

  delay(3000);
  // exit(0);
}

/**
 * Converts Arduino readings to voltage (5V pins)
 * 
 * @param reading Arduino reading (0-1023)
 * @return (float) Real voltage on pin
 */
double ToVolt(int reading)
{
  return reading / 1024.0 * 5.0;
}

/**
 * Gets temperature and humidity sensor readings
 * 
 * All values are saved to global variables at the top of this document
 * 
 * @param plot If true, results will be shown in Serial window 
 */
void GetTempHum(bool plot)
{
  temp = dht.readTemperature();
  hum = dht.readHumidity();

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
 * Gets readings from all MQX sensors
 * 
 * All values are saved to global variables at the top of this document
 * 
 * @param plot If true, results will be shown in Serial window 
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
    Serial.println("SGP30 read failed! Shutting down...");
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
