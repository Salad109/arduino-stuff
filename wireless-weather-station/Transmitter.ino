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
#include "forcedClimate.h"
#include "LowPower.h"
#include "NRFLite.h"

#define DEBUG_MODE 1     // 1 = Serial debugging messages, 0 = Serial disabled

ForcedClimate climateSensor = ForcedClimate(Wire, 0x76);


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
  if (DEBUG_MODE) {
    Serial.begin(9600);
    Serial.println("Hello world");
  }
  Wire.begin();
  climateSensor.begin();

  if (!_radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN, NRFLite::BITRATE250KBPS)) {
    Serial.println("Cannot communicate with radio");
    while (1) {}
  }
  _radioData.FromRadioId = RADIO_ID;
}

void loop() {
  getData();                          // Get readings then save them to the _radioData object.
  _radioData.OnTimeMillis = millis();
  if (DEBUG_MODE) {  // DEBUGGING LOOP
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

  LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
  /*
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
  */
}

//===========================================================

void printData() {
  Serial.print("Temperature:");
  Serial.print(_radioData.temp);
  Serial.print("*C    ");

  Serial.print("Pressure:");
  Serial.print(_radioData.pres);
  Serial.print("hPa    ");

  Serial.print("Humidity:");
  Serial.print(_radioData.humi);
  Serial.print("%    ");
}
void getData() {
  climateSensor.takeForcedMeasurement();
  _radioData.temp = climateSensor.getTemperatureCelcius();
  _radioData.pres = climateSensor.getPressure();
  _radioData.humi = climateSensor.getRelativeHumidity();
}