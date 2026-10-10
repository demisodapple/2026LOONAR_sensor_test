#pragma once
#include <Arduino.h>
#include <Wire.h>

// QST QMC5883L, +/-8G: 3000 LSB/G, little-endian signed XYZ.
class Qmc5883l {
 public:
  static constexpr uint8_t kAddress = 0x0D;
  explicit Qmc5883l(TwoWire& wire) : wire_(wire) {}
  bool begin() {
    if (!writeRegister(0x0A, 0x80)) return false;
    delay(10);
    if (!writeRegister(0x0A, 0) || !writeRegister(0x0B, 1) ||
        !writeRegister(0x09, 0x15)) return false;
    // OSR512, +/-8G, 50Hz, continuous; verify actual register writes.
    uint8_t control = 0, period = 0;
    if (!readRegisters(0x09, &control, 1) || control != 0x15 ||
        !readRegisters(0x0B, &period, 1) || period != 1) return false;
    delay(30);
    return true;
  }
  bool readStatus(uint8_t& status) { return readRegisters(0x06, &status, 1); }
  bool readMagneticField(float& x, float& y, float& z) {
    x = y = z = NAN;
    uint8_t status = 0;
    const uint32_t started = millis();
    do {
      if (!readStatus(status)) return false;
      if (status & 0x03) break;
      delay(1);
    } while (millis() - started < 60);
    if (!(status & 0x03)) return false;
    uint8_t data[6];
    // Drain XYZ even on overflow to clear latched status.
    if (!readRegisters(0x00, data, 6) || (status & 0x02)) return false;
    // DOR is expected at 2Hz logging / 50Hz conversion: use latest data.
    constexpr float uTPerCount = 100.0f / 3000.0f;
    x = decode(data) * uTPerCount;
    y = decode(data + 2) * uTPerCount;
    z = decode(data + 4) * uTPerCount;
    return true;
  }
 private:
  TwoWire& wire_;
  static int16_t decode(const uint8_t* data) {
    const uint16_t value = static_cast<uint16_t>(data[0]) |
                          (static_cast<uint16_t>(data[1]) << 8);
    return static_cast<int16_t>(value >= 0x8000 ?
        static_cast<int32_t>(value) - 65536 : value);
  }
  bool writeRegister(uint8_t reg, uint8_t value) {
    wire_.beginTransmission(kAddress);
    wire_.write(reg);
    wire_.write(value);
    return wire_.endTransmission() == 0;
  }
  bool readRegisters(uint8_t reg, uint8_t* data, uint8_t count) {
    wire_.beginTransmission(kAddress);
    wire_.write(reg);
    if (wire_.endTransmission(false) != 0) return false;
    const uint8_t received = wire_.requestFrom(kAddress, count,
                                              static_cast<uint8_t>(true));
    if (received != count) {
      while (wire_.available()) wire_.read();
      return false;
    }
    for (uint8_t i = 0; i < count; ++i) {
      if (!wire_.available()) return false;
      data[i] = wire_.read();
    }
    return true;
  }
};
