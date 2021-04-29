#include <Wire.h>
#include "Adafruit_SGP30.h"
#include "DHT.h"

#define DHTPIN 2
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);
Adafruit_SGP30 sgp;


float temp = 0.0;
float hum = 0.0;
float tvoc = 0.0;
float eco2 = 0.0;

int sensorMQ135;    // Benzene, Alcohol, smoke
int sensorMQ6;      // LPG, butane gas -> If reading > 600
int sensorMQ4;      // Methane, CNG Gas
int sensorMQ7;      // Carbon Monoxide

int pinMQ135 = 0;
int pinMQ6 = 1;
int pinMQ4 = 2;
int pinMQ7 = 3;

bool calibrate = true;
uint16_t TVOC_base, eCO2_base; // SGP baseline variables

void setup() {
  Serial.begin(9600);

  pinMode(pinMQ135, INPUT);
  pinMode(pinMQ6, INPUT);
  pinMode(pinMQ4, INPUT);
  pinMode(pinMQ7, INPUT);
  
  dht.begin();

  if (! sgp.begin()) {
    Serial.println(F("SGP30 not found!"));
    while (1); // shut down program
  }
  Serial.println(F("SGP30 OK!"));
  sgp.setIAQBaseline(0x0, 0x0);
  
}

void loop() {
  
  if (calibrate) {
    CalibrateSgp();
  }
  calibrate = false;

  GetSgp(true);
  GetTempHum(true);
  GetAllSensors(true);

  Serial.println("#################################################");

  delay(3000);
  // exit(0);
}

double ToVolt(int reading)
{
  return reading / 1024.0 * 5.0;
}

void GetTempHum(bool plot) {
  temp = dht.readTemperature();
  hum = dht.readHumidity();

  if (isnan(hum) || isnan(temp)) {
    Serial.println("Failed to read from DHT sensor!");
  }

  if (plot) {
    Serial.println("Temp: " + String(temp) + " °C");
    Serial.println("Hum: " + String(hum) + " %");
  }
}

void GetAllSensors(bool plot) {
  sensorMQ135 = analogRead(pinMQ135);
  sensorMQ6 = analogRead(pinMQ6);
  sensorMQ4 = analogRead(pinMQ4);

  if (plot) {
    Serial.println("MQ135 read: " + String(sensorMQ135) + " U: " + String(ToVolt(sensorMQ135)) + " V");
    Serial.println("MQ6 read: " + String(sensorMQ6) + " U: " + String(ToVolt(sensorMQ6)) + " V");
    Serial.println("MQ4 read: " + String(sensorMQ4) + " U: " + String(ToVolt(sensorMQ4)) + " V");
    Serial.println("MQ7 read: " + String(sensorMQ7) + " U: " + String(ToVolt(sensorMQ7)) + " V");
  }
}

void CalibrateSgp() {
  
  if (! sgp.getIAQBaseline(&eCO2_base, &TVOC_base)) {
    Serial.println("Failed to get baseline SGP");
    exit(1);
  } else {
    Serial.print("Baseline -> eCO2: 0x"); Serial.print(eCO2_base, HEX);
    Serial.print(" & TVOC: 0x"); Serial.println(TVOC_base, HEX);
  }
  delay(1000);
}

void GetSgp(bool plot) {
  if (! sgp.IAQmeasure()) {
    Serial.println("SGP30 read failed! Shutting down...");
    return;
  }
  else {
    tvoc = sgp.TVOC;
    eco2 = sgp.eCO2;
  }
  if (plot) {
    Serial.println("TVOC: " + String(tvoc) + " ppb");
    Serial.println("eCO2: " + String(eco2) + " ppm");
  }
}
