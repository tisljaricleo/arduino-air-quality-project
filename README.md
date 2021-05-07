# arduino-air-quality-project
A project that includes Arduino code for getting the air quality sensors readings and sends it to Raspberry-based server.

## This project will include
- [x] Arduino code for air quality sensors readings
  - [x] MQ6 - LPG, butane gas
  - [x] MQ135 - Benzene, Alcohol, smoke
  - [x] MQ4 - Methane, CNG Gas
  - [x] MQ7 - Carbon Monoxide
- [x] Arduino code for air quality sensor SGP30  
- [x] Arduino code for temperature and humidity sensor DTH11
- [x] Tutorial for making server using Raspberry PI
- [ ] Django application for recieving and storing sensor readings
- [ ] Frontend with dashboard

## Usage

### Raspberry PI
- We use Raspberry pi 400 as a server  
- Configuration of the Apache can be found [here](https://github.com/tisljaricleo/my_cheatsheets/blob/main/raspberry-pi-apache-django-cheatsheet.txt)   

### ESP32D Development board
- In this project we decided to use ESP32D because it has Wifi module
- [This](https://e-radionica.com/en/croduino-nova32.html?___from_store=hr) module is used.
- To successfuly use this module:
  - [Install CH340 drivers](https://e-radionica.com/productdata/CH34x_Install_Windows_v3_4.EXE)
  - [Install ESP32 Board Definition](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md)
  - In Arduino IDE go to `Tools>Board>ESP32 Dev module`
- Install following modules using `Tools>Manage libraries..`
  - Adafruit_SGP30, DHTesp

## Contact
[Leo Tisljaric](https://www.linkedin.com/in/leo-ti%C5%A1ljari%C4%87-28a56b123/)
