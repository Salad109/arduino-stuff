// DFRduino Nano
// BME280 I2C mode
// nRF24L01+
// DFRobot Solar Power Manager + 18650 Cell + 0,4W Solar panel

#include <Wire.h>
#include <forcedClimate.h>

#define DEBUGMODE 1  // 1 = Serial debugging messages, 0 = none

ForcedClimate climateSensor = ForcedClimate();

void setup() {
  initializeSerial();
  Wire.begin();
  climateSensor.begin();
}

void loop() {
  climateSensor.takeForcedMeasurement();
  if (DEBUGMODE) {
    printData();
  }
  delay(1000);
}

//===========================================================

void initializeSerial() {
  if (DEBUGMODE) {
    Serial.begin(9600);
    Serial.println("Hello world");
  }
}

void printData() {
  Serial.print("temperature:");
  Serial.print(climateSensor.getTemperatureCelcius());
  Serial.print("*C   ");

  Serial.print("pressure:");
  Serial.print(climateSensor.getPressure() / 100.0F);
  Serial.print("hPa   ");

  Serial.print("humidity:");
  Serial.print(climateSensor.getRelativeHumidity());
  Serial.println("%");
}