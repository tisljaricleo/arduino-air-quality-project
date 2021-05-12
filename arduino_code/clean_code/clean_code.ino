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
#include "DHTesp.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "MHZ19.h"

#define PINMQ6 25
#define PINDHT 32             // Pin for DHT readings
#define DHTTYPE DHT11         // Defines sensor type
#define WIFISSID "LeoWifi"    // Wifi name
#define WIFIPSW "wifi12345"   // Wifi password
#define RXD2 18
#define TXD2 19

DHTesp dht;                           // Constructor for DHT
TaskHandle_t tempTaskHandle = NULL;   // Task handle for the light value read task
MHZ19 myMHZ19;

// Main server address
String hostAdress = "192.168.155.58";

// Sensor readings variables
float temp = 0.0;
float hum = 0.0;
int sensorMQ6 = 0.0;
int MHCO2;
int8_t MHTemp;
struct pms5003data {    // uint16_t values from 0 to 65,535
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
struct pms5003data data;

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
  if ((WiFi.status() == WL_CONNECTED)) {    //Check the current connection status
    Serial.println("WIFI Ready!");


    if (m == "GET") {
      HTTPClient http;
      http.begin(host);                         //Specify the URL
      int httpCode = http.GET();                //Make the request
      Serial.println(String(httpCode));

      if (httpCode > 0) {                       //Check for the returning code
        String payload = http.getString();
        Serial.println(httpCode);
        Serial.println(payload);
      }
      else {
        Serial.println("Error on GET request");
      }
      http.end();                               //Free the resources
    }

    if (m == "POST") {
      HTTPClient http;
      http.begin(host);                         //Specify destination for HTTP request
      http.addHeader("Content-Type", "application/json");
      int httpResponseCode = http.POST("{\"s_name\":\"arduino1\",\"s_type\":\"MQArduino\",\"s_reading\":\"999999\"}");

      if (httpResponseCode > 0) {
        String response = http.getString();     //Get the response to the request
        Serial.println(httpResponseCode);       //Print return code
        Serial.println(response);               //Print request answer
      }
      else {
        Serial.print("Error on sending POST: ");
        Serial.println(httpResponseCode);
      }
      http.end();                               //Free resources
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
  Serial.begin(115200);
  delay(4000);                                    //Delay needed before calling the WiFi.begin

  Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);    // MH-z19 sensor
  myMHZ19.begin(Serial1);
  myMHZ19.autoCalibration(true);

  Serial2.begin(9600);                            // PMS sensor

  pinMode(PINMQ6, INPUT);

  // Connect to Wifi
  //  scanNetworks();
  //  connectToNetwork();
  //  Serial.println(WiFi.macAddress());
  //  Serial.println(WiFi.localIP());
 
  // Starts DTH11 sensor
  dht.setup(PINDHT, DHTesp::DHT11);
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
  //  ServerCom("GET", "http://" + hostAdress + "/api/sensor/", "");
  //
  //  if (addOne) {
  //    ServerCom("POST", "http://" + hostAdress + "/api/sensor/", "");
  //    addOne = false;
  //  }

  Serial.println("#################################################");

  delay(2000);

  GetTempHum(true);    // Gets temperature and humidity readings
  GetFlammable(true); // Gets readings from all MQX6 sensor
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

void GetFlammable(bool plot)
{
  sensorMQ6 = analogRead(PINMQ6);
  if (plot) {
    Serial.println("MQ6 read: " + String(sensorMQ6) + " U: " + String(ToVolt(sensorMQ6)) + " V");
  }
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
  //read byte by byte untill '0x42'
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }
  // Read all 32 bytes
  if (s->available() < 32) {
    return false;
  } uint8_t buffer[32]; uint16_t sum = 0; s->readBytes(buffer, 32);
  // Checksum
  for (uint8_t i = 0; i < 30; i++) {
    sum += buffer[i];
  }
  // Using endian'd protocol
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }
  // Save to struct
  memcpy((void *)&data, (void *)buffer_u16, 30);
  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
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
