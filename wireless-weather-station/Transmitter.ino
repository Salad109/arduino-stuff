/*==========================================================
BME280    Arduino 
VCC    -> 3V3
GND    -> GND
SDA    -> A4
SCL    -> A5
ADDR   -> N/C
CS     -> N/C
============================================================
nRF24L01+    Arduino
CE    -> 9
CSN   -> 10 
MOSI  -> 11 
MISO  -> 12 
SCK   -> 13 
IRQ   -> N/C
GND   -> GND
VCC                -> Transistor collector
Transistor base    -> D8
Transistor emitter -> 3V3
============================================================
DFRobot Solar Power Manager    Arduino
EN=BAT
5V                          -> 5V
GND                         -> GND
==========================================================*/
#include <Wire.h>
#include <forcedClimate.h>
#include "LowPower.h"
#include "SPI.h"
#include "NRFLite.h"

#define DEBUGMODE 1     // 1 = Serial debugging messages, 0 = Serial disabled
#define NRF_POWERPIN 8  // nRF power transistor's base

ForcedClimate climateSensor = ForcedClimate();


const static uint8_t RADIO_ID = 1;              // TX RADIO ID(me)
const static uint8_t DESTINATION_RADIO_ID = 0;  // RX radio ID
const static uint8_t PIN_RADIO_CE = 9;
const static uint8_t PIN_RADIO_CSN = 10;

struct RadioPacket {
  uint8_t FromRadioId;
  uint32_t OnTimeMillis;
  uint32_t FailedTxCount;
  float temp;
  float pres;
  float humi;
};


NRFLite _radio;
RadioPacket _radioData;

void setup() {
  digitalWrite(NRF_POWERPIN, HIGH);
  if (DEBUGMODE) {
    Serial.begin(9600);
    Serial.println("Hello world");
  }
  Wire.begin();
  climateSensor.begin();

  if (!_radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN, NRFLite::BITRATE250KBPS, 0)) {
    Serial.println("Cannot communicate with radio");
    while (1) {}
  }
  _radioData.FromRadioId = RADIO_ID;
}

void loop() {
  climateSensor.takeForcedMeasurement();
  digitalWrite(NRF_POWERPIN, HIGH);  // Power on the radio module
  _radioData.OnTimeMillis = millis();
  if (DEBUGMODE) {  // DEBUGGING LOOP
    printData();

    Serial.print("Sending ");
    Serial.print(_radioData.OnTimeMillis);
    Serial.print(" ms");

    if (_radio.send(DESTINATION_RADIO_ID, &_radioData, sizeof(_radioData))) {
      Serial.println("...Success");
    } else {
      Serial.println("...Failed");
      _radioData.FailedTxCount++;
    }
  } else {  // STEALTH LOOP
    if (!_radio.send(DESTINATION_RADIO_ID, &_radioData, sizeof(_radioData))) {
      _radioData.FailedTxCount++;
    }
  }

  digitalWrite(NRF_POWERPIN, LOW);  // Power off the radio module
  for (int i = 0; i < 1; i++)
    LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
}

//===========================================================

void printData() {
  Serial.print("Temperature:");
  Serial.print(climateSensor.getTemperatureCelcius());
  Serial.print("*C    ");

  Serial.print("Pressure:");
  Serial.print(climateSensor.getPressure() / 100.0F);
  Serial.print("hPa    ");

  Serial.print("Humidity:");
  Serial.print(climateSensor.getRelativeHumidity());
  Serial.print("%    ");
}