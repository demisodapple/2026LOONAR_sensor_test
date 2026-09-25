#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <Adafruit_LIS3MDL.h>
#include <Adafruit_MAX31865.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_Sensor.h>

namespace {
// Arduino MKR Zero hardware buses.
constexpr uint8_t PIN_MAX31865_CS = 7;
constexpr uint8_t BOARD_SPI_MOSI = 8;
constexpr uint8_t BOARD_SPI_SCK = 9;
constexpr uint8_t BOARD_SPI_MISO = 10;
constexpr uint8_t PIN_I2C_SDA = 11;
constexpr uint8_t PIN_I2C_SCL = 12;

constexpr uint8_t MLX90614_ADDRESS = 0x5A;
constexpr uint32_t I2C_CLOCK_HZ = 100000;
constexpr uint32_t SAMPLE_INTERVAL_MS = 500;
constexpr uint32_t RETRY_INTERVAL_MS = 2000;

constexpr float RTD_NOMINAL_OHM = 1000.0f;  // PT1000
constexpr float RTD_REFERENCE_OHM = 4630.0f;

Adafruit_LIS3MDL magnetometer;
Adafruit_MLX90614 infrared;
Adafruit_MAX31865 rtd(PIN_MAX31865_CS);

bool magnetometerReady = false;
bool infraredReady = false;
bool rtdReady = false;
uint8_t magnetometerAddress = 0;
uint32_t lastSampleMs = 0;
uint32_t lastRetryMs = 0;

bool i2cAddressResponds(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void printHexByte(uint8_t value) {
  if (value < 0x10) Serial.print('0');
  Serial.print(value, HEX);
}

void printFloatOrNan(float value, uint8_t digits) {
  if (isfinite(value)) Serial.print(value, digits);
  else Serial.print("nan");
}

void scanI2cBus() {
  Serial.println("# I2C scan begin");
  uint8_t count = 0;
  for (uint8_t address = 1; address < 127; ++address) {
    if (i2cAddressResponds(address)) {
      Serial.print("# I2C found 0x");
      printHexByte(address);
      Serial.println();
      ++count;
    }
  }
  if (count == 0) Serial.println("# I2C no devices found");
}

bool beginMagnetometer() {
  if (magnetometer.begin_I2C(0x1C, &Wire)) {
    magnetometerAddress = 0x1C;
  } else if (magnetometer.begin_I2C(0x1E, &Wire)) {
    magnetometerAddress = 0x1E;
  } else {
    magnetometerAddress = 0;
    Serial.println("# LIS3MDL FAIL: no response at 0x1C or 0x1E");
    return false;
  }

  magnetometer.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  magnetometer.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  magnetometer.setDataRate(LIS3MDL_DATARATE_20_HZ);
  magnetometer.setRange(LIS3MDL_RANGE_4_GAUSS);

  Serial.print("# LIS3MDL READY at 0x");
  printHexByte(magnetometerAddress);
  Serial.println();
  return true;
}

bool beginInfrared() {
  if (!i2cAddressResponds(MLX90614_ADDRESS)) {
    Serial.println("# MLX90614 FAIL: no response at 0x5A");
    return false;
  }
  if (!infrared.begin(MLX90614_ADDRESS, &Wire)) {
    Serial.println("# MLX90614 FAIL: library initialization failed");
    return false;
  }
  Serial.println("# MLX90614 READY at 0x5A");
  return true;
}

bool beginRtd() {
  if (!rtd.begin(MAX31865_3WIRE)) {
    Serial.println("# MAX31865 FAIL: begin failed");
    return false;
  }

  // begin() alone does not prove that the chip answered. Verify read/write SPI.
  constexpr uint16_t TEST_LOW = 0x2468;
  constexpr uint16_t TEST_HIGH = 0x5A5A;
  rtd.setThresholds(TEST_LOW, TEST_HIGH);
  const uint16_t lowReadback = rtd.getLowerThreshold();
  const uint16_t highReadback = rtd.getUpperThreshold();
  rtd.setThresholds(0x0000, 0xFFFF);

  if (lowReadback != TEST_LOW || highReadback != TEST_HIGH) {
    Serial.print("# MAX31865 FAIL: SPI readback LOW=0x");
    Serial.print(lowReadback, HEX);
    Serial.print(" HIGH=0x");
    Serial.println(highReadback, HEX);
    return false;
  }

  Serial.println("# MAX31865 READY (3-wire PT1000)");
  return true;
}

void retryMissingSensors(uint32_t now) {
  if (now - lastRetryMs < RETRY_INTERVAL_MS) return;
  lastRetryMs = now;

  if (!magnetometerReady) magnetometerReady = beginMagnetometer();
  if (!infraredReady) infraredReady = beginInfrared();
  if (!rtdReady) rtdReady = beginRtd();
}

void printCsvHeader() {
  Serial.println(
      "time_ms,mag_x_uT,mag_y_uT,mag_z_uT,mag_norm_uT,"
      "ir_ambient_C,ir_object_C,rtd_raw,rtd_ohm,rtd_C,"
      "mag_valid,ir_valid,rtd_valid,rtd_fault");
}
}  // namespace

void setup() {
  Serial.begin(115200);
  const uint32_t serialStartMs = millis();
  while (!Serial && millis() - serialStartMs < 3000) delay(10);

  Wire.begin();  // SDA=11, SCL=12 on MKR Zero.
  Wire.setClock(I2C_CLOCK_HZ);
  SPI.begin();   // MOSI=8, SCK=9, MISO=10 on MKR Zero.

  Serial.println();
  Serial.println("# MKR ZERO THREE-SENSOR LOGGER");
  Serial.print("# SPI MAX31865: CS=");
  Serial.print(PIN_MAX31865_CS);
  Serial.print(" MOSI=");
  Serial.print(BOARD_SPI_MOSI);
  Serial.print(" MISO=");
  Serial.print(BOARD_SPI_MISO);
  Serial.print(" SCK=");
  Serial.println(BOARD_SPI_SCK);
  Serial.print("# I2C LIS3MDL + MLX90614: SDA=");
  Serial.print(PIN_I2C_SDA);
  Serial.print(" SCL=");
  Serial.println(PIN_I2C_SCL);

  delay(300);  // Allow MLX90614 to settle after power-up.
  magnetometerReady = beginMagnetometer();
  infraredReady = beginInfrared();
  rtdReady = beginRtd();

  if (!magnetometerReady || !infraredReady) scanI2cBus();
  printCsvHeader();
  lastRetryMs = millis();
}

void loop() {
  const uint32_t now = millis();
  retryMissingSensors(now);

  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) return;
  lastSampleMs = now;

  float magX = NAN;
  float magY = NAN;
  float magZ = NAN;
  float magNorm = NAN;
  float irAmbientC = NAN;
  float irObjectC = NAN;
  uint16_t rtdRaw = 0;
  float rtdOhm = NAN;
  float rtdC = NAN;
  uint8_t rtdFault = 0;
  bool magValid = false;
  bool irValid = false;
  bool rtdValid = false;

  if (magnetometerReady) {
    sensors_event_t event;
    magValid = magnetometer.getEvent(&event);
    if (magValid) {
      magX = event.magnetic.x;
      magY = event.magnetic.y;
      magZ = event.magnetic.z;
      magNorm = sqrtf(magX * magX + magY * magY + magZ * magZ);
    } else {
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
    if (rtdFault != 0) rtd.clearFault();
  }

  Serial.print(now);
  Serial.print(',');
  printFloatOrNan(magX, 3);
  Serial.print(',');
  printFloatOrNan(magY, 3);
  Serial.print(',');
  printFloatOrNan(magZ, 3);
  Serial.print(',');
  printFloatOrNan(magNorm, 3);
  Serial.print(',');
  printFloatOrNan(irAmbientC, 2);
  Serial.print(',');
  printFloatOrNan(irObjectC, 2);
  Serial.print(',');
  Serial.print(rtdRaw);
  Serial.print(',');
  printFloatOrNan(rtdOhm, 2);
  Serial.print(',');
  printFloatOrNan(rtdC, 2);
  Serial.print(',');
  Serial.print(magValid ? 1 : 0);
  Serial.print(',');
  Serial.print(irValid ? 1 : 0);
  Serial.print(',');
  Serial.print(rtdValid ? 1 : 0);
  Serial.print(",0x");
  printHexByte(rtdFault);
  Serial.println();
}
