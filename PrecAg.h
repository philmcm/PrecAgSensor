/*  
  PrecAg.h - Library for Precision Agriculture sensing.
  Created for ENGG4201/8201 by Andrew Miller, Phil McMillan
*/
#ifndef PRECAG_H
#define PRECAG_H

#include "Arduino.h"

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
// #include <Arduino_MKRGPS.h>
#include "Adafruit_seesaw.h"

// I2C address
#define SOIL_SENSOR (char) 0x36

#define BME_CS 7
#define SEALEVELPRESSURE_HPA (1013.25)

class PrecAg {
  public:
      PrecAg();
      void setupSensors();
      void takeReadings();
      void sendNReq();
      void sendPReq();
      void sendKReq();
      void getNResp();
      void getPResp();
      void getKResp();
      float getTemperature();
      float getPressure();
      float getAltitude();
      float getHumidity();
      int getLight();
      uint16_t getMoisture();
      byte getNitrogen();
      byte getPhosphorous();
      byte getPotassium();
  private:
      float _temperature;
      float _pressure;
      float _humidity;
      float _altitude;
      int _light;
      uint16_t _moisture;
      byte _nitrogen;
      byte _phosphorous;
      byte _potassium;
};
#endif