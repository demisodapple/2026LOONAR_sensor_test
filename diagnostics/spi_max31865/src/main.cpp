#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_MAX31865.h>

constexpr uint8_t MAX_CS = 7;
constexpr uint8_t MAX_MOSI = 8;
constexpr uint8_t MAX_MISO = 10;
constexpr uint8_t MAX_SCK = 9;
constexpr float RTD_NOMINAL = 1000.0f;  // PT1000
constexpr float RTD_REFERENCE = 4630.0f;

Adafruit_MAX31865 max31865(MAX_CS);

void printFault(uint8_t fault) {
  if (fault & MAX31865_FAULT_HIGHTHRESH) Serial.println("FAULT: RTD high threshold");
  if (fault & MAX31865_FAULT_LOWTHRESH) Serial.println("FAULT: RTD low threshold");
  if (fault & MAX31865_FAULT_REFINLOW) Serial.println("FAULT: REFIN- > 0.85 x bias");
  if (fault & MAX31865_FAULT_REFINHIGH) Serial.println("FAULT: REFIN- < 0.85 x bias (FORCE- open)");
  if (fault & MAX31865_FAULT_RTDINLOW) Serial.println("FAULT: RTDIN- < 0.85 x bias (FORCE- open)");
  if (fault & MAX31865_FAULT_OVUV) Serial.println("FAULT: under/over voltage");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    delay(10);
  }
  SPI.begin();

  Serial.println("MAX31865 SPI TEST (MKR Zero)");
  Serial.print("PINMAP: CS=");
  Serial.print(MAX_CS);
  Serial.print(", MOSI=");
  Serial.print(MAX_MOSI);
  Serial.print(", MISO=");
  Serial.print(MAX_MISO);
  Serial.print(", SCK=");
  Serial.println(MAX_SCK);
  Serial.println("MODULE PINS: CS<-7, SDI<-8, SDO->10, CLK<-9");
}

void loop() {
  const bool beginOk = max31865.begin(MAX31865_3WIRE);
  if (!beginOk) {
    Serial.println("BEGIN: FAIL");
    delay(2000);
    return;
  }

  // begin() does not check for a response from the MAX31865. Verify SPI
  // by writing known threshold values and reading them back.
  max31865.setThresholds(0x2468, 0x5A5A);
  const uint16_t lowReadback = max31865.getLowerThreshold();
  const uint16_t highReadback = max31865.getUpperThreshold();
  max31865.setThresholds(0x0000, 0xFFFF);

  const bool spiOk = lowReadback == 0x2468 && highReadback == 0x5A5A;
  Serial.print("SPI=");
  Serial.print(spiOk ? "OK" : "FAIL");
  Serial.print(", LOW=0x");
  Serial.print(lowReadback, HEX);
  Serial.print(", HIGH=0x");
  Serial.println(highReadback, HEX);
  if (!spiOk) {
    Serial.println("MAX31865 did not return the written register values.");
    delay(2000);
    return;
  }

  const uint16_t raw = max31865.readRTD();
  const uint8_t fault = max31865.readFault();

  if (fault == 0 && raw > 0 && raw < 32767) {
    const float resistance = (raw / 32768.0f) * RTD_REFERENCE;
    const float temperature = max31865.calculateTemperature(raw, RTD_NOMINAL, RTD_REFERENCE);
    Serial.print("RAW=");
    Serial.print(raw);
    Serial.print(", R=");
    Serial.print(resistance, 2);
    Serial.print(" ohm, T=");
    Serial.print(temperature, 2);
    Serial.print(" C, FAULT=0x");
    Serial.println(fault, HEX);
  } else {
    Serial.print("INVALID READING: RAW=");
    Serial.print(raw);
    Serial.print(", FAULT=0x");
    Serial.println(fault, HEX);
  }
  if (fault) {
    printFault(fault);
    max31865.clearFault();
  }

  delay(2000);
}
