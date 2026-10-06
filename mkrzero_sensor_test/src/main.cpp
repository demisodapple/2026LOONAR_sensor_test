#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <wiring_private.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_MAX31865.h>

// D0=PA22/SERCOM3 PAD0 (SDA), D1=PA23/SERCOM3 PAD1 (SCL).
// Sensor libraries call begin() again: restore the mux on every begin.
class InfraredWire : public TwoWire {
 public:
  InfraredWire() : TwoWire(&sercom3, 0, 1) {}
  void begin() override {
    TwoWire::begin();
    pinPeripheral(0, PIO_SERCOM);
    pinPeripheral(1, PIO_SERCOM);
    setClock(50000);
  }
};
InfraredWire irWire;
void SERCOM3_Handler() { irWire.onService(); }

namespace {
constexpr uint8_t RTD_CS = 7;
constexpr uint32_t I2C_HZ = 50000;
// Match the actual RTD and the reference resistor fitted to the module.
constexpr float RTD_R0 = 1000.0f;
constexpr float RTD_RREF = 4630.0f;
Adafruit_LIS3MDL mag;
Adafruit_MLX90614 ir;
Adafruit_MAX31865 rtd(RTD_CS, &SPI);
bool magReady = false, irReady = false, rtdReady = false;
uint32_t lastSample = 0, lastRetry = 0;

bool busIdle(uint8_t sda, uint8_t scl) {
  return digitalRead(sda) == HIGH && digitalRead(scl) == HIGH;
}

void initMissing() {
  if (TEST_MAG && !busIdle(11, 12)) {
    Serial.print("# LIS3MDL I2C not idle SDA11="); Serial.print(digitalRead(11));
    Serial.print(" SCL12="); Serial.println(digitalRead(12));
  } else {
    if (TEST_MAG && !magReady) {
      magReady = mag.begin_I2C(0x1C, &Wire) || mag.begin_I2C(0x1E, &Wire);
      Wire.setClock(I2C_HZ);
      if (magReady) {
        mag.setPerformanceMode(LIS3MDL_MEDIUMMODE);
        mag.setOperationMode(LIS3MDL_CONTINUOUSMODE);
        mag.setDataRate(LIS3MDL_DATARATE_20_HZ);
        mag.setRange(LIS3MDL_RANGE_4_GAUSS);
      }
      Serial.println(magReady ? "# LIS3MDL READY" : "# LIS3MDL FAIL");
    }
  }
  if (TEST_IR && !busIdle(0, 1)) {
    Serial.print("# MLX90614 I2C not idle SDA0="); Serial.print(digitalRead(0));
    Serial.print(" SCL1="); Serial.println(digitalRead(1));
  } else {
    if (TEST_IR && !irReady) {
      irReady = ir.begin(0x5A, &irWire);
      irWire.setClock(I2C_HZ);
      if (irReady) {
        irReady = isfinite(ir.readAmbientTempC()) && isfinite(ir.readObjectTempC());
      }
      Serial.println(irReady ? "# MLX90614 READY" : "# MLX90614 FAIL");
    }
  }
  if (TEST_RTD && !rtdReady) {
    rtdReady = rtd.begin(MAX31865_3WIRE);
    // begin alone does not prove SPI wiring: verify writable registers.
    if (rtdReady) {
      rtd.setThresholds(0x2468, 0x5A5A);
      rtdReady = rtd.getLowerThreshold() == 0x2468 &&
                 rtd.getUpperThreshold() == 0x5A5A;
      rtd.setThresholds(0, 0xFFFF);
    }
    Serial.println(rtdReady ? "# MAX31865 READY" : "# MAX31865 SPI FAIL");
  }
}

void field(float value, uint8_t digits = 3) {
  Serial.print(',');
  if (isfinite(value)) Serial.print(value, digits);
  else Serial.print("nan");
}
}

void setup() {
  Serial.begin(115200);
  const uint32_t started = millis();
  while (!Serial && millis() - started < 5000) {}
  delay(1000);
  pinMode(RTD_CS, OUTPUT);
  digitalWrite(RTD_CS, HIGH);
  SPI.begin();
  Wire.begin();
  Wire.setClock(I2C_HZ);
  irWire.begin();
  Serial.println("# MKRZERO SENSOR TEST v2 split I2C; raw magnetometer, no calibration");
  Serial.print("# ENABLED MAG="); Serial.print(TEST_MAG);
  Serial.print(" IR="); Serial.print(TEST_IR);
  Serial.print(" RTD="); Serial.println(TEST_RTD);
  Serial.println("# LIS3MDL SDA=11 SCL=12; MLX90614 SDA=0 SCL=1; both 50kHz; SPI MOSI=8 MISO=10 SCK=9 CS=7");
  initMissing();
  Serial.println("time_ms,mag_x_uT,mag_y_uT,mag_z_uT,mag_norm_uT,ir_ambient_C,ir_object_C,rtd_raw,rtd_ohm,rtd_C,mag_valid,ir_valid,rtd_valid,rtd_fault");
  lastRetry = millis();
}

void loop() {
  uint32_t now = millis();
  // Commands: r retries initialization, h reprints the CSV header.
  while (Serial.available()) {
    const char command = Serial.read();
    if (command == 'r') {
      magReady = irReady = rtdReady = false;
      initMissing();
    }
    if (command == 'h') Serial.println("time_ms,mag_x_uT,mag_y_uT,mag_z_uT,mag_norm_uT,ir_ambient_C,ir_object_C,rtd_raw,rtd_ohm,rtd_C,mag_valid,ir_valid,rtd_valid,rtd_fault");
  }
  if (now - lastRetry >= 5000) { lastRetry = now; initMissing(); }
  now = millis();
  if (now - lastSample < 500) return;
  lastSample = now;
  float x = NAN, y = NAN, z = NAN, norm = NAN;
  float ambient = NAN, object = NAN, ohm = NAN, temp = NAN;
  uint16_t raw = 0;
  uint8_t fault = 0;
  bool mv = false, iv = false, rv = false;
  if (magReady && busIdle(11, 12)) {
    mv = mag.readMagneticField(x, y, z) && isfinite(x) && isfinite(y) && isfinite(z);
    if (mv) norm = sqrtf(x*x + y*y + z*z);
    else x = y = z = NAN;
    if (!mv) magReady = false;
  }
  if (irReady && busIdle(0, 1)) {
    ambient = ir.readAmbientTempC(); object = ir.readObjectTempC();
    iv = isfinite(ambient) && isfinite(object) && ambient >= -40 && ambient <= 125 && object >= -70 && object <= 380;
    if (!iv) { ambient = object = NAN; irReady = false; }
  }
  if (rtdReady) {
    raw = rtd.readRTD(); fault = rtd.readFault();
    rv = fault == 0 && raw > 0 && raw < 32767;
    if (rv) {
      ohm = raw * RTD_RREF / 32768.0f;
      temp = rtd.calculateTemperature(raw, RTD_R0, RTD_RREF);
      rv = isfinite(temp);
    }
    if (!rv) ohm = temp = NAN;
    if (fault) rtd.clearFault();
  }
  Serial.print(now);
  field(x); field(y); field(z); field(norm); field(ambient, 2); field(object, 2);
  Serial.print(','); Serial.print(raw); field(ohm, 2); field(temp, 2);
  Serial.print(','); Serial.print(mv); Serial.print(','); Serial.print(iv);
  Serial.print(','); Serial.print(rv); Serial.print(",0x");
  if (fault < 16) Serial.print('0'); Serial.println(fault, HEX);
}
