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
VCC   -> 3V3
============================================================
DFRobot Solar Power Manager    Arduino
EN=BAT
5V                          -> 5V
GND                         -> GND
==========================================================*/
#include <forcedClimate.h>
#include "LowPower.h"
#include "NRFLite.h"
#include "avr/power.h"

#define DEBUG_MODE 1  // 1 = Serial debugging messages, 0 = Serial disabled

ForcedClimate climateSensor = ForcedClimate(Wire, 0x76);  //  CS low = 0x76, CS high = 0x77


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
  disableStuff();  // Various modules which are never used

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
  getData();  // Get readings then save them to our _radioData object

  if (DEBUG_MODE) {  // Debugging loop
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
  } else {  // Stealth loop
    if (!_radio.send(DESTINATION_RADIO_ID, &_radioData, sizeof(_radioData))) {
      _radioData.FailedTxCount++;
    }
  }

  goToSleep();
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
  _radioData.OnTimeMillis = millis();
}

void disableStuff() {
  for (int i = 0; i < 20; i++) {  // Set all unused pins as low outputs
    if (i != 4 && i != 5 && (i < 9 || i > 13)) {
      pinMode(i, OUTPUT);
      digitalWrite(i, LOW);
    }
  }
  MCUCR = bit(BODS) | bit(BODSE);  // Disable brown-out detector
  MCUCR = bit(BODS);               //

  ADCSRA = 0;              // Disable ADC
  ADCSRA &= ~(1 << ADEN);  //

  ACSR = (1 << ACD);  // Disable ACD

  DIDR0 = 0x3F;                         // ?
  DIDR1 = (1 << AIN1D) | (1 << AIN0D);  //
}

void goToSleep() {
  power_all_disable();
  power_twi_disable();     // I2C
  power_usart0_disable();  // Serial


  power_spi_disable();       // SPI
  byte old_ADCSRA = ADCSRA;  //
  ADCSRA = 0;                //

  //power_timer0_disable();  // Needed for delay and millis()
  //power_timer1_disable();  //
  //power_timer2_disable();  // Needed for asynchronous 32kHz operation

  LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
  /*
  for (int i = 0; i < 8; i++) {                      // 64s wakeup interval
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  //
  }                                                  //
  */

  power_all_enable();
  power_twi_enable();      // I2C
  power_usart0_disable();  // Serial

  ADCSRA = old_ADCSRA;  // SPI
  power_spi_enable();   //

  //power_timer0_enable();
  //power_timer1_enable();
  //power_timer2_enable();
}