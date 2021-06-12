/*  
  PrecAg.cpp - Library for Precision Agriculture sensing.
  Created for ENGG4201/8201 by Andrew Miller, Phil McMillan
*/
#include "Arduino.h"
#include "PrecAg.h"

Adafruit_seesaw ss;
Adafruit_BME280 bme(BME_CS); // hardware SPI

const byte nitroBytes[] = {0x01,0x03, 0x00, 0x1e, 0x00, 0x01, 0xe4, 0x0c};
const byte phosBytes[] = {0x01,0x03, 0x00, 0x1f, 0x00, 0x01, 0xb5, 0xcc};
const byte potaBytes[] = {0x01,0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xc0};
 
volatile byte values[11];

PrecAg::PrecAg() {
  _temperature = 0;
  _pressure = 0;
  _humidity = 0;
  _altitude = 0;
  _light = 0;
  for (int i=0; i < FILTER_TERMS; i++)
    _moisture[i] = 0;
  _nitrogen = 0;
  _phosphorous = 0;
  _potassium = 0;
  _autoValveEnabled = false;
}

void PrecAg::setupSensors() {
  // initialise the UART for the NPK sensor
  Serial1.begin(9600);
  while(!Serial1);

  // initialise the Seesaw sensor for Soil Moisture (I2C)
  if  (!ss.begin(SOIL_SENSOR)) {
    Serial.println("ERROR! seesaw not found");
    while(1);
  }
  // default settings
  unsigned status = bme.begin();  
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      while (1);
  }

  // initialise GPS
  /* if (!GPS.begin()) {
    Serial.println("Failed to initialize GPS!");
    while (1);
  } */

  // initialise actuator
  pinMode(ACTUATOR, OUTPUT);
  digitalWrite(ACTUATOR, LOW);
  _valveOpen = false;
}

void PrecAg::takeReadings() {
  // take readings from BME280
  _temperature = bme.readTemperature();
  _pressure = bme.readPressure() / 100.0F;
  _altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  _humidity = bme.readHumidity();

  // take reading from analog light sensor
  _light = analogRead(0);

  // take reading from soil sensor, maintain multiple terms for averaging
  for (int i=0; i < FILTER_TERMS - 1; i++) {
    _moisture[i]=_moisture[i+1];
  }
  _moisture[FILTER_TERMS - 1] = ss.touchRead(0);
}  

void PrecAg::sendNReq() {
  Serial1.write(nitroBytes,sizeof(nitroBytes));
}

void PrecAg::sendPReq() {
  Serial1.write(phosBytes,sizeof(phosBytes));
}

void PrecAg::sendKReq() {
  Serial1.write(potaBytes,sizeof(potaBytes));
}

void PrecAg::getNResp() {
  for(byte i=0;i<7;i++){
    values[i] = Serial1.read();
  }
  _nitrogen = values[4];
}

void PrecAg::getPResp() {
  for(byte i=0;i<7;i++){
    values[i] = Serial1.read();
  }
  _phosphorous = values[4];
}

void PrecAg::getKResp() {
  for(byte i=0;i<7;i++){
    values[i] = Serial1.read();
  }
  _potassium = values[4];
}

float PrecAg::getTemperature() { return _temperature; }
float PrecAg::getPressure() { return _pressure; }
float PrecAg::getAltitude() { return _altitude; }
float PrecAg::getHumidity() { return _humidity; }
int PrecAg::getLight() { return _light; }
uint16_t PrecAg::getMoisture() {
  int sum=0;
  for (int i=0; i < FILTER_TERMS; i++)
    sum += _moisture[i];
  return (sum/FILTER_TERMS);
}
byte PrecAg::getNitrogen()  { return _nitrogen; }
byte PrecAg::getPhosphorous()  { return _phosphorous; }
byte PrecAg::getPotassium() { return _potassium; }

void PrecAg::checkValve(byte hours, byte mins, byte secs) {
  int currentTime = secs + (mins * 60) + (hours * 3600);
  int openTime = _valveOpenSecs + (_valveOpenMins * 60) + (_valveOpenHours * 3600);
  int closeTime = _valveCloseSecs + (_valveCloseMins * 60) + (_valveCloseHours * 3600);

  if (_autoValveEnabled == true) {
    if (currentTime >= closeTime) {
      digitalWrite(ACTUATOR, LOW);
      _valveOpen = false;
    }
    else if (currentTime >= openTime) {
      digitalWrite(ACTUATOR, HIGH);
      _valveOpen = true;
    }
    else {
      digitalWrite(ACTUATOR, LOW);
      _valveOpen = false;
    }
  }
}

void PrecAg::setValveOpenTime(byte valveOpenHours, byte valveOpenMins, byte valveOpenSecs,
      byte valveCloseHours, byte valveCloseMins, byte valveCloseSecs) {
  _autoValveEnabled = true;
  _valveOpenHours = valveOpenHours;
  _valveOpenMins = valveOpenMins;
  _valveOpenSecs = valveOpenSecs;
  _valveCloseHours = valveCloseHours;
  _valveCloseMins = valveCloseMins;
  _valveCloseSecs = valveCloseSecs;
}

bool PrecAg::getValve() { return _valveOpen; }