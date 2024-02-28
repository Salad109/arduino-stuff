// DFRduino Nano
// BME280
// nRF24L01
// DFRobot Solar Power Manager + 18650 Cell + 0,4W Solar panel

#include <Wire.h>
#include <SPI.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define DEBUGMODE 1  // 1 = Serial debugging messages, 0 = none

void setup() {
  initializeBME();
}

void loop() {
  // put your main code here, to run repeatedly:
}
void initializeBME() {
  Adafruit_BME280 bme;
  bool BMEinit = bme.begin();
  if (BMEinit || DEBUGMODE) {
    Serial.println("BME280 init success");
  } else {
    Serial.println("BME280 init failure");
    while (1) {}
  }
}
