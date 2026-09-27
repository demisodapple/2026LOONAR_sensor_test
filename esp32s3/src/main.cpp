#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <Adafruit_LIS3MDL.h>
#include <Adafruit_MAX31865.h>
#include <Adafruit_MLX90614.h>

namespace {
// ESP32-S3-DevKitC-1 pin assignment. ESP32-S3 GPIO Matrix allows these
// hardware-bus signals to be mapped explicitly.
constexpr uint8_t PIN_I2C_SDA = 8;
constexpr uint8_t PIN_I2C_SCL = 9;
constexpr uint8_t PIN_MAX31865_CS = 10;
constexpr uint8_t PIN_SPI_MOSI = 11;
constexpr uint8_t PIN_SPI_SCK = 12;
constexpr uint8_t PIN_SPI_MISO = 13;

constexpr uint8_t MLX90614_ADDRESS = 0x5A;
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t I2C_CLOCK_HZ = 50000;
constexpr uint32_t POWER_STABILIZE_MS = 5000;
constexpr uint32_t BUS_STABILIZE_MS = 500;
constexpr uint32_t RETRY_DELAY_MS = 150;
constexpr uint32_t RETRY_INTERVAL_MS = 2000;
constexpr uint32_t SAMPLE_INTERVAL_MS = 500;
constexpr uint8_t MAX_BEGIN_ATTEMPTS = 5;

constexpr float RTD_NOMINAL_OHM = 1000.0f;
constexpr float RTD_REFERENCE_OHM = 4630.0f;

Adafruit_LIS3MDL magnetometer;
Adafruit_MLX90614 infrared;
Adafruit_MAX31865 rtd(PIN_MAX31865_CS, &SPI);

bool magnetometerReady = false;
bool infraredReady = false;
bool rtdReady = false;
uint8_t magnetometerAddress = 0;
uint32_t lastSampleMs = 0;
uint32_t lastRetryMs = 0;

void printHexByte(uint8_t value) {
  if (value < 0x10) Serial.print('0');
  Serial.print(value, HEX);
}

void printFloatOrNan(float value, uint8_t digits) {
  if (isfinite(value)) Serial.print(value, digits);
  else Serial.print("nan");
}

bool i2cAddressResponds(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void scanI2cBus() {
  Serial.println("# I2C SCAN BEGIN");
  uint8_t count = 0;
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    const uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("# I2C FOUND 0x");
      printHexByte(address);
      if (address == 0x1C || address == 0x1E) Serial.print(" LIS3MDL");
      if (address == MLX90614_ADDRESS) Serial.print(" MLX90614");
      Serial.println();
      ++count;
    }
  }
  Serial.print("# I2C SCAN END count=");
  Serial.println(count);
}

bool beginMagnetometer() {
  if (magnetometer.begin_I2C(0x1C, &Wire)) magnetometerAddress = 0x1C;
  else if (magnetometer.begin_I2C(0x1E, &Wire)) magnetometerAddress = 0x1E;
  else {
    magnetometerAddress = 0;
    Serial.println("# LIS3MDL FAIL");
    return false;
  }

  magnetometer.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  magnetometer.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  magnetometer.setDataRate(LIS3MDL_DATARATE_20_HZ);
  magnetometer.setRange(LIS3MDL_RANGE_4_GAUSS);
  Serial.print("# LIS3MDL READY address=0x");
  printHexByte(magnetometerAddress);
  Serial.println();
  return true;
}

bool beginInfrared() {
  if (!i2cAddressResponds(MLX90614_ADDRESS)) {
    Serial.println("# MLX90614 FAIL: no ACK at 0x5A");
    return false;
  }
  if (!infrared.begin(MLX90614_ADDRESS, &Wire)) {
    Serial.println("# MLX90614 FAIL: begin failed");
    return false;
  }
  Serial.println("# MLX90614 READY address=0x5A");
  return true;
}

bool beginRtd() {
  if (!rtd.begin(MAX31865_3WIRE)) {
    Serial.println("# MAX31865 FAIL: begin failed");
    return false;
  }

  // Confirm real SPI communication by writing and reading threshold registers.
  constexpr uint16_t TEST_LOW = 0x2468;
  constexpr uint16_t TEST_HIGH = 0x5A5A;
  rtd.setThresholds(TEST_LOW, TEST_HIGH);
  const uint16_t lowReadback = rtd.getLowerThreshold();
  const uint16_t highReadback = rtd.getUpperThreshold();
  rtd.setThresholds(0x0000, 0xFFFF);

  if (lowReadback != TEST_LOW || highReadback != TEST_HIGH) {
    Serial.print("# MAX31865 FAIL: LOW=0x");
    Serial.print(lowReadback, HEX);
    Serial.print(" HIGH=0x");
    Serial.println(highReadback, HEX);
    return false;
  }
  Serial.println("# MAX31865 READY: SPI read-back OK, mode=1, clock=1MHz");
  return true;
}

bool beginWithRetries(const char* name, bool (*beginSensor)()) {
  for (uint8_t attempt = 1; attempt <= MAX_BEGIN_ATTEMPTS; ++attempt) {
    Serial.printf("# %s begin attempt %u/%u\n", name, attempt,
                  MAX_BEGIN_ATTEMPTS);
    if (beginSensor()) return true;
    if (attempt < MAX_BEGIN_ATTEMPTS) delay(RETRY_DELAY_MS);
  }
  Serial.printf("# %s unavailable after %u attempts\n", name,
                MAX_BEGIN_ATTEMPTS);
  return false;
}

void retryMissingSensors(uint32_t now) {
  if (now - lastRetryMs < RETRY_INTERVAL_MS) return;
  lastRetryMs = now;
  if (!magnetometerReady)
    magnetometerReady = beginWithRetries("LIS3MDL", beginMagnetometer);
  if (!infraredReady)
    infraredReady = beginWithRetries("MLX90614", beginInfrared);
  if (!rtdReady) rtdReady = beginWithRetries("MAX31865", beginRtd);
}

void printCsvHeader() {
  Serial.println(
      "time_ms,mag_x_uT,mag_y_uT,mag_z_uT,mag_norm_uT,"
      "ir_ambient_C,ir_object_C,rtd_raw,rtd_ohm,rtd_C,"
      "mag_valid,ir_valid,rtd_valid,rtd_fault");
}
}  // namespace

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(POWER_STABILIZE_MS);

  // Keep the SPI slave deselected before starting the SPI peripheral.
  pinMode(PIN_MAX31865_CS, OUTPUT);
  digitalWrite(PIN_MAX31865_CS, HIGH);
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_MAX31865_CS);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);
  delay(BUS_STABILIZE_MS);

  Serial.println();
  Serial.println("# ESP32-S3 THREE-SENSOR TEST");
  Serial.printf("# I2C SDA=%u SCL=%u clock=%luHz\n", PIN_I2C_SDA,
                PIN_I2C_SCL, I2C_CLOCK_HZ);
  Serial.printf("# SPI CS=%u MOSI=%u MISO=%u SCK=%u mode=1 clock=1MHz\n",
                PIN_MAX31865_CS, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCK);

  scanI2cBus();
  magnetometerReady = beginWithRetries("LIS3MDL", beginMagnetometer);
  infraredReady = beginWithRetries("MLX90614", beginInfrared);
  rtdReady = beginWithRetries("MAX31865", beginRtd);
  printCsvHeader();
  lastRetryMs = millis();
}

void loop() {
  const uint32_t now = millis();
  retryMissingSensors(now);
  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) return;
  lastSampleMs = now;

  float magX = NAN, magY = NAN, magZ = NAN, magNorm = NAN;
  float irAmbientC = NAN, irObjectC = NAN;
  uint16_t rtdRaw = 0;
  float rtdOhm = NAN, rtdC = NAN;
  uint8_t rtdFault = 0;
  bool magValid = false, irValid = false, rtdValid = false;

  if (magnetometerReady) {
    magValid = magnetometer.readMagneticField(magX, magY, magZ);
    if (magValid) magNorm = sqrtf(magX * magX + magY * magY + magZ * magZ);
    else {
      Serial.println("# LIS3MDL read failed; scheduling reinitialization");
      magnetometerReady = false;
    }
  }

  if (infraredReady) {
    irAmbientC = infrared.readAmbientTempC();
    irObjectC = infrared.readObjectTempC();
    irValid = isfinite(irAmbientC) && isfinite(irObjectC) &&
              irAmbientC > -70.0f && irAmbientC < 390.0f &&
              irObjectC > -70.0f && irObjectC < 390.0f;
    if (!irValid && !i2cAddressResponds(MLX90614_ADDRESS)) {
      Serial.println("# MLX90614 disconnected; scheduling reinitialization");
      infraredReady = false;
    }
  }

  if (rtdReady) {
    rtdRaw = rtd.readRTD();
    rtdFault = rtd.readFault();
    rtdOhm = (static_cast<float>(rtdRaw) / 32768.0f) * RTD_REFERENCE_OHM;
    rtdC = rtd.calculateTemperature(rtdRaw, RTD_NOMINAL_OHM,
                                    RTD_REFERENCE_OHM);
    rtdValid = rtdFault == 0 && rtdRaw > 0 && rtdRaw < 32767 &&
               isfinite(rtdC) && rtdC >= -100.0f && rtdC <= 500.0f;
    if (rtdFault) rtd.clearFault();
  }

  Serial.print(now);
  Serial.print(','); printFloatOrNan(magX, 3);
  Serial.print(','); printFloatOrNan(magY, 3);
  Serial.print(','); printFloatOrNan(magZ, 3);
  Serial.print(','); printFloatOrNan(magNorm, 3);
  Serial.print(','); printFloatOrNan(irAmbientC, 2);
  Serial.print(','); printFloatOrNan(irObjectC, 2);
  Serial.print(','); Serial.print(rtdRaw);
  Serial.print(','); printFloatOrNan(rtdOhm, 2);
  Serial.print(','); printFloatOrNan(rtdC, 2);
  Serial.print(','); Serial.print(magValid ? 1 : 0);
  Serial.print(','); Serial.print(irValid ? 1 : 0);
  Serial.print(','); Serial.print(rtdValid ? 1 : 0);
  Serial.print(",0x"); printHexByte(rtdFault);
  Serial.println();
}
