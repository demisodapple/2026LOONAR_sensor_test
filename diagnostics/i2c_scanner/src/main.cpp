#include <Arduino.h>
#include <Wire.h>

namespace {
constexpr uint32_t SCAN_INTERVAL_MS = 2000;

void printHexByte(uint8_t value) {
  if (value < 0x10) Serial.print('0');
  Serial.print(value, HEX);
}

void printKnownDevice(uint8_t address) {
  if (address == 0x5A) {
    Serial.print(" (MLX90614)");
  } else if (address == 0x1C || address == 0x1E) {
    Serial.print(" (LIS3MDL)");
  }
}

void scanI2cBus() {
  Serial.println("I2C SCAN START");

  uint8_t count = 0;

  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    const uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("FOUND: 0x");
      printHexByte(address);
      printKnownDevice(address);
      Serial.println();
      ++count;
    }
  }

  Serial.print("I2C SCAN END, COUNT=");
  Serial.println(count);
}
}  // namespace

void setup() {
  Serial.begin(115200);
  const uint32_t serialStartMs = millis();
  while (!Serial && millis() - serialStartMs < 3000) delay(10);

  Wire.begin();
  Wire.setClock(100000);

  Serial.println();
  Serial.println("MKR ZERO I2C BUS CHECK");
  Serial.println("SDA=11, SCL=12");
  Serial.println("Expected: MLX90614=0x5A, LIS3MDL=0x1C or 0x1E");
}

void loop() {
  scanI2cBus();
  delay(SCAN_INTERVAL_MS);
}
