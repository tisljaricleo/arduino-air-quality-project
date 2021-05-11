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
#include <WiFi.h>
#include <HTTPClient.h>
#include "MHZ19.h"

#define PINMQ135 13
#define PINMQ6 25
#define PINMQ4 26
#define PINMQ7 27
#define PINDHT 32             // Pin for DHT readings
#define DHTTYPE DHT11         // Defines sensor type
#define WIFISSID "LeoWifi"    // Wifi name
#define WIFIPSW "wifi12345"   // Wifi password
#define RXD2 18
#define TXD2 19

DHTesp dht;                           // Constructor for DHT
TaskHandle_t tempTaskHandle = NULL;   // Task handle for the light value read task
Adafruit_SGP30 sgp;                   // Constructor for SGP30
MHZ19 myMHZ19;

// Main server address
String hostAdress = "192.168.155.58";

// Sensor readings variables
float temp = 0.0;
float hum = 0.0;
float tvoc = 0.0;
float eco2 = 0.0;
int sensorMQ135 = 0.0;
int sensorMQ6 = 0.0;
int sensorMQ4 = 0.0;
int sensorMQ7 = 0.0;
int MHCO2;
int8_t MHTemp;
struct pms5003data {//deklaracija uint16_t koji ima vrijednost od 0 do 65,535
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
struct pms5003data data;

// SGP30 must have its baseline values.
// Sensor must be recalibrated befor usage.
// Before callibration, sensor must work for at least 24h.
bool calibrate = false;
uint16_t TVOC_base, eCO2_base;

// Testing vars
// Add one sensor with post req
bool addOne = true;

unsigned long getDataTimer = 0;

/**
   Communication with server using HTTP requests

   Used for sending HTTP requests.
   Example usage:
   1) POST: ServerCom("POST", "http://jsonplaceholder.typicode.com/posts", "");
   2) GET: ServerCom("GET", "http://jsonplaceholder.typicode.com/comments?id=10", "");

   @param m HTTP method
   @param host Host url for request
   @param header Define custom header
   @return void
*/
void ServerCom(String m, String host, String header = "") {
  if ((WiFi.status() == WL_CONNECTED)) { //Check the current connection status
    Serial.println("WIFI Ready!");


    if (m == "GET") {
      HTTPClient http;
      http.begin(host); //Specify the URL
      int httpCode = http.GET();                                        //Make the request
      Serial.println(String(httpCode));

      if (httpCode > 0) { //Check for the returning code
        String payload = http.getString();
        Serial.println(httpCode);
        Serial.println(payload);
      }
      else {
        Serial.println("Error on GET request");
      }
      http.end(); //Free the resources
    }

    if (m == "POST") {
      HTTPClient http;
      http.begin(host);  //Specify destination for HTTP request
      // http.addHeader("Content-Type", "text/plain");             //Specify content-type header


      http.addHeader("Content-Type", "application/json");
      int httpResponseCode = http.POST("{\"s_name\":\"arduino1\",\"s_type\":\"MQArduino\",\"s_reading\":\"999999\"}");

      
      // int httpResponseCode = http.POST("POSTING from ESP32");   //Send the actual POST request

      if (httpResponseCode > 0) {
        String response = http.getString();                       //Get the response to the request
        Serial.println(httpResponseCode);   //Print return code
        Serial.println(response);           //Print request answer
      }
      else {
        Serial.print("Error on sending POST: ");
        Serial.println(httpResponseCode);
      }
      http.end();  //Free resources
    }
  }
  else {
    Serial.println("Wifi errorr! Request is not sent!");
  }
}

/**
   Project setup.

   Function that runs only once.

*/
void setup()
{
  Wire.begin();
  Serial.begin(115200);
  delay(4000);   //Delay needed before calling the WiFi.begin

  Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  myMHZ19.begin(Serial1);
  myMHZ19.autoCalibration(true);
  
  Serial2.begin(9600);//Linija za komunkikaciju s PMS senzorom

  pinMode(PINMQ135, INPUT);
  pinMode(PINMQ6, INPUT);
  pinMode(PINMQ4, INPUT);
  pinMode(PINMQ7, INPUT);

  // Connect to Wifi
  scanNetworks();
  connectToNetwork();
  Serial.println(WiFi.macAddress());
  Serial.println(WiFi.localIP());

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

  GetCO2Temp(true);

  if (readPMSdata(&Serial2)) {
    PlotParticles(true);
  }
  
  //  if (calibrate)
  //  {
  //    CalibrateSgp(); // Calibrates SGP sensor
  //  }
  //  calibrate = false;

  ServerCom("GET", "http://" + hostAdress + "/api/sensor/", "");

  if (addOne) {
    ServerCom("POST", "http://" + hostAdress + "/api/sensor/", "");
    addOne = false;
  }
  
  Serial.println("#################################################");

  delay(5000);


  GetSgp(true);        // Gets SGP readings
  GetTempHum(true);    // Gets temperature and humidity readings
  GetAllSensors(true); // Gets readings from all MQX sensors

  Serial.println("#################################################");

  delay(2000);

} 
// END LOOP

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
   Calibrates SGP30 sensor

   Sets and prints SGP30 baseline values

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
   Gets readings from SGP30 sensor

   All values are saved to global variables at the top of this document

   @param plot If true, results will be shown in Serial window
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

/**
   Absolute humidity

   Gets AbsoluteHumidity
   Borrowed from https://github.com/Makerfabs/Project_Touch-Camera-ILI9341/tree/master/example/CO2_Monitor
*/
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
  // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
  return absoluteHumidityScaled;
}

/**
   Encryption type

   Gets encryption type from wifi network
   Borrowed from https://techtutorialsx.com
*/
String translateEncryptionType(wifi_auth_mode_t encryptionType) {
  switch (encryptionType) {
    case (WIFI_AUTH_OPEN):
      return "Open";
    case (WIFI_AUTH_WEP):
      return "WEP";
    case (WIFI_AUTH_WPA_PSK):
      return "WPA_PSK";
    case (WIFI_AUTH_WPA2_PSK):
      return "WPA2_PSK";
    case (WIFI_AUTH_WPA_WPA2_PSK):
      return "WPA_WPA2_PSK";
    case (WIFI_AUTH_WPA2_ENTERPRISE):
      return "WPA2_ENTERPRISE";
  }
}

/**
   Scan Wifi networks

   Scans all available wifi networks and prints to Console
   Borrowed from https://techtutorialsx.com
*/
void scanNetworks() {
  int numberOfNetworks = WiFi.scanNetworks();
  Serial.print("Number of networks found: ");
  Serial.println(numberOfNetworks);
  for (int i = 0; i < numberOfNetworks; i++) {
    Serial.print("Network name: ");
    Serial.println(WiFi.SSID(i));
    Serial.print("Signal strength: ");
    Serial.println(WiFi.RSSI(i));
    Serial.print("MAC address: ");
    Serial.println(WiFi.BSSIDstr(i));
    Serial.print("Encryption type: ");
    String encryptionTypeDescription = translateEncryptionType(WiFi.encryptionType(i));
    Serial.println(encryptionTypeDescription);
    Serial.println("-----------------------");
  }
}

/**
   Connect to wifi

   Connects to wifi network defined with SSID and password in WIFISSID and WIFIPSW variables
   Borrowed from https://techtutorialsx.com
*/
void connectToNetwork() {
  WiFi.begin(WIFISSID, WIFIPSW);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Establishing connection to WiFi..");
  }
  Serial.println("Connected to network");
}

/**
   Gets readings from MH-Z19C sensor (CO2 and Temp)

   @param plot True for ploting on console 
*/
void GetCO2Temp(bool plot) {

  if (millis() - getDataTimer >= 2000)
  {
    

    /* note: getCO2() default is command "CO2 Unlimited". This returns the correct CO2 reading even
      if below background CO2 levels or above range (useful to validate sensor). You can use the
      usual documented command with getCO2(false) */

    MHCO2 = myMHZ19.getCO2();
    MHTemp = myMHZ19.getTemperature();

    if (plot) {
      Serial.print("CO2 (ppm): ");
      Serial.println(MHCO2);
      Serial.print("Temperature (C): ");
      Serial.println(MHTemp);
    }
    getDataTimer = millis();
  }


}


/**
   Gets readings from PMS5003 sensor

   @return (boolean) True if sensor works.
*/
boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }

  //čitaj byte po bit dok ne dobijemo '0x42' startni byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }

  // Sada pročitaj sva 32 bytes
  if (s->available() < 32) {
    return false;
  } uint8_t buffer[32]; uint16_t sum = 0; s->readBytes(buffer, 32);

  // provjerimo sve s kontrolnim zbrojem
  for (uint8_t i = 0; i < 30; i++) {
    sum += buffer[i];
  }
  // Podatci dolaze u endian'd protokolu, ovaj dio koda omogućava podržanost na svim platformama
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  // spremimo sve u strukturu
  memcpy((void *)&data, (void *)buffer_u16, 30);

  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // kada sve uspije vraćamo rezultat
  return true;
}


/**
   Plots readings from PMS5003 sensor

   @param plot True for ploting on console
*/
void PlotParticles(bool plot)
{
  if (plot)
  {
    Serial.println("---------------------------------------");
    Serial.print("Cestica u zraku > 0.3um / 0.1L zraka:");
    Serial.println(data.particles_03um);//Čestica većih od 0.3 um
    Serial.print("Cestica u zraku > 0.5um / 0.1L zraka:");
    Serial.println(data.particles_05um);//Čestica većih od 0.5 um
    Serial.print("Cestica u zraku > 1.0um / 0.1L zraka:");
    Serial.println(data.particles_10um);//Čestica većih od 1 um
    Serial.print("Cestica u zraku > 2.5um / 0.1L zraka:");
    Serial.println(data.particles_25um);//Čestica većih od 2.5 um
    Serial.print("Cestica u zraku > 5.0um / 0.1L zraka:");
    Serial.println(data.particles_50um);//Čestica većih od 5 um
    Serial.print("Cestica u zraku > 10.0 um / 0.1L zraka:");
    Serial.println(data.particles_100um);//Čestica većih od 10 um
    Serial.println("---------------------------------------");
  }
}
