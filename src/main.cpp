#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <Adafruit_LIS3MDL.h>
#include <Adafruit_MAX31865.h>
#include <Adafruit_MLX90614.h>
#include "mag_calibration.h"

namespace {
// Teensy 4.1 default hardware buses.
constexpr uint8_t PIN_MAX31865_CS = 10;
constexpr uint8_t BOARD_SPI_MOSI = 11;
constexpr uint8_t BOARD_SPI_MISO = 12;
constexpr uint8_t BOARD_SPI_SCK = 13;
constexpr uint8_t PIN_LIS3MDL_SDA = 18;
constexpr uint8_t PIN_LIS3MDL_SCL = 19;
constexpr uint8_t PIN_MLX90614_SDA = 17;
constexpr uint8_t PIN_MLX90614_SCL = 16;
constexpr uint32_t MAX31865_SPI_CLOCK_HZ = 1000000;
constexpr uint8_t MAX31865_SPI_MODE = SPI_MODE1;

constexpr uint8_t MLX90614_ADDRESS = 0x5A;
constexpr uint32_t I2C_CLOCK_HZ = 50000;
constexpr uint32_t MLX90614_I2C_CLOCK_HZ = 50000;
constexpr uint32_t POWER_STABILIZE_MS = 5000;
constexpr uint32_t BUS_STABILIZE_MS = 250;
constexpr uint32_t WIRE_STABILIZE_MS = 500;
constexpr uint32_t BEGIN_RETRY_DELAY_MS = 150;
constexpr uint8_t BEGIN_MAX_ATTEMPTS = 5;
constexpr uint32_t SAMPLE_INTERVAL_MS = 500;
constexpr uint32_t RETRY_INTERVAL_MS = 2000;
constexpr uint32_t MLX90614_COOLDOWN_MS = 10000;
constexpr uint32_t PERIODIC_REINIT_INTERVAL_MS = 5UL * 60UL * 1000UL;

constexpr float RTD_NOMINAL_OHM = 1000.0f;  // PT1000
constexpr float RTD_REFERENCE_OHM = 4344.0f;

Adafruit_LIS3MDL lisMagnetometer;
Adafruit_MLX90614 infrared;
// Adafruit_MAX31865 v1.6.2 configures this hardware SPI device for
// 1 MHz, MSB first, SPI_MODE1. Pass &SPI explicitly so Teensy uses SPI0.
Adafruit_MAX31865 rtd(PIN_MAX31865_CS, &SPI);

bool lisMagnetometerReady = false;
bool infraredReady = false;
bool rtdReady = false;
uint32_t lastSampleMs = 0;
uint32_t lastRetryMs = 0;
uint32_t mlxNextRetryMs = 0;
uint8_t mlxRuntimeFailures = 0;
uint32_t lastPeriodicReinitMs = 0;
bool nanRecoveryArmed = true;

void printHexByte(uint8_t value) {
  if (value < 0x10) Serial.print('0');
  Serial.print(value, HEX);
}

void printFloatOrNan(float value, uint8_t digits) {
  if (isfinite(value)) Serial.print(value, digits);
  else Serial.print("nan");
}

bool beginLisMagnetometer() {
  if (!lisMagnetometer.begin_I2C(0x1C, &Wire) &&
      !lisMagnetometer.begin_I2C(0x1E, &Wire)) {
    Wire.setClock(I2C_CLOCK_HZ);
    Serial.println("# LIS3MDL FAIL on Wire: no response at 0x1C or 0x1E");
    return false;
  }
  // Library begin() resets the bus clock; restore the chosen clock.
  Wire.setClock(I2C_CLOCK_HZ);
  lisMagnetometer.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lisMagnetometer.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lisMagnetometer.setDataRate(LIS3MDL_DATARATE_20_HZ);
  lisMagnetometer.setRange(LIS3MDL_RANGE_4_GAUSS);
  Serial.println("# LIS3MDL READY on Wire; 20Hz, +/-4G");
  return true;
}

bool beginInfrared() {
  // No scanner or address-only pre-probe. begin() performs library detection,
  // then real temperature registers are read to verify communication.
  if (!infrared.begin(MLX90614_ADDRESS, &Wire1)) {
    Wire1.setClock(MLX90614_I2C_CLOCK_HZ);
    Serial.println("# MLX90614 FAIL: library initialization failed");
    return false;
  }

  // Adafruit_I2CDevice::begin() calls Wire1.begin(), restoring 100 kHz.
  Wire1.setClock(MLX90614_I2C_CLOCK_HZ);
  const float ambient = infrared.readAmbientTempC();
  const float object = infrared.readObjectTempC();
  if (!isfinite(ambient) || !isfinite(object) || ambient <= -70.0f ||
      ambient >= 390.0f || object <= -70.0f || object >= 390.0f) {
    Serial.println("# MLX90614 FAIL: temperature register verification failed");
    return false;
  }
  Serial.println("# MLX90614 READY at 0x5A; registers verified");
  return true;
}

void recoverMlxBus() {
  Serial.print("# MLX90614 Wire1 recovery begin SDA=");
  Serial.print(digitalRead(PIN_MLX90614_SDA));
  Serial.print(" SCL=");
  Serial.println(digitalRead(PIN_MLX90614_SCL));

  Wire1.end();
  pinMode(PIN_MLX90614_SDA, INPUT_PULLUP);
  pinMode(PIN_MLX90614_SCL, INPUT_PULLUP);
  delayMicroseconds(10);

  uint8_t pulses = 0;
  while (digitalRead(PIN_MLX90614_SDA) == LOW && pulses < 9) {
    pinMode(PIN_MLX90614_SCL, OUTPUT);
    digitalWrite(PIN_MLX90614_SCL, LOW);
    delayMicroseconds(5);
    pinMode(PIN_MLX90614_SCL, INPUT_PULLUP);
    delayMicroseconds(5);
    ++pulses;
  }

  // STOP condition: SDA low, release SCL, then release SDA. Neither line is
  // ever driven HIGH, preserving open-drain operation.
  pinMode(PIN_MLX90614_SDA, OUTPUT);
  digitalWrite(PIN_MLX90614_SDA, LOW);
  delayMicroseconds(5);
  pinMode(PIN_MLX90614_SCL, INPUT_PULLUP);
  delayMicroseconds(5);
  pinMode(PIN_MLX90614_SDA, INPUT_PULLUP);
  delayMicroseconds(5);

  Wire1.setSDA(PIN_MLX90614_SDA);
  Wire1.setSCL(PIN_MLX90614_SCL);
  Wire1.begin();
  Wire1.setClock(MLX90614_I2C_CLOCK_HZ);
  delay(WIRE_STABILIZE_MS);

  Serial.print("# MLX90614 Wire1 recovery end pulses=");
  Serial.print(pulses);
  Serial.print(" SDA=");
  Serial.print(digitalRead(PIN_MLX90614_SDA));
  Serial.print(" SCL=");
  Serial.println(digitalRead(PIN_MLX90614_SCL));
}

bool beginRtd() {
  if (!rtd.begin(MAX31865_3WIRE)) {
    Serial.println("# MAX31865 FAIL: begin failed");
    return false;
  }

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

void printCsvHeader();

bool beginWithRetries(const char* name, bool (*beginSensor)()) {
  for (uint8_t attempt = 1; attempt <= BEGIN_MAX_ATTEMPTS; ++attempt) {
    Serial.print("# ");
    Serial.print(name);
    Serial.print(" begin attempt ");
    Serial.print(attempt);
    Serial.print('/');
    Serial.println(BEGIN_MAX_ATTEMPTS);
    if (beginSensor()) return true;
    if (attempt < BEGIN_MAX_ATTEMPTS) delay(BEGIN_RETRY_DELAY_MS);
  }
  Serial.print("# ");
  Serial.print(name);
  Serial.println(" unavailable after 5 attempts");
  return false;
}

void reinitializeSensors(const char* reason) {
  Serial.println();
  Serial.print("# SENSOR REINITIALIZATION BEGIN reason=");
  Serial.println(reason);

  pinMode(PIN_MAX31865_CS, OUTPUT);
  digitalWrite(PIN_MAX31865_CS, HIGH);
  SPI.begin();

  Wire.setSDA(PIN_LIS3MDL_SDA);
  Wire.setSCL(PIN_LIS3MDL_SCL);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  Wire1.setSDA(PIN_MLX90614_SDA);
  Wire1.setSCL(PIN_MLX90614_SCL);
  Wire1.begin();
  Wire1.setClock(MLX90614_I2C_CLOCK_HZ);
  delay(BUS_STABILIZE_MS);
  delay(WIRE_STABILIZE_MS - BUS_STABILIZE_MS);

  lisMagnetometerReady = beginWithRetries("LIS3MDL", beginLisMagnetometer);
  infraredReady = beginWithRetries("MLX90614", beginInfrared);
  rtdReady = beginWithRetries("MAX31865", beginRtd);

  mlxRuntimeFailures = 0;
  const uint32_t completedMs = millis();
  mlxNextRetryMs = infraredReady ? 0 : completedMs + MLX90614_COOLDOWN_MS;
  lastRetryMs = completedMs;
  lastSampleMs = completedMs;
  lastPeriodicReinitMs = completedMs;

  Serial.print("# SENSOR REINITIALIZATION END LIS3MDL=");
  Serial.print(lisMagnetometerReady ? 1 : 0);
  Serial.print(" MLX90614=");
  Serial.print(infraredReady ? 1 : 0);
  Serial.print(" MAX31865=");
  Serial.println(rtdReady ? 1 : 0);
  printCsvHeader();
}

void retryMissingSensors(uint32_t now) {
  if (now - lastRetryMs < RETRY_INTERVAL_MS) return;
  lastRetryMs = now;
  if (!lisMagnetometerReady)
    lisMagnetometerReady = beginWithRetries("LIS3MDL", beginLisMagnetometer);
  if (!rtdReady) rtdReady = beginWithRetries("MAX31865", beginRtd);
}

bool retryInfrared(uint32_t now) {
  if (infraredReady || static_cast<int32_t>(now - mlxNextRetryMs) < 0)
    return false;

  Serial.print("# MLX90614 runtime begin attempt ");
  Serial.print(mlxRuntimeFailures + 1);
  Serial.println("/5");
  infraredReady = beginInfrared();
  if (infraredReady) {
    mlxRuntimeFailures = 0;
    return true;
  }

  ++mlxRuntimeFailures;
  if (mlxRuntimeFailures >= BEGIN_MAX_ATTEMPTS) {
    recoverMlxBus();
    mlxRuntimeFailures = 0;
    mlxNextRetryMs = millis() + MLX90614_COOLDOWN_MS;
    Serial.println("# MLX90614 cooldown 10s after 5 failed attempts");
  } else {
    mlxNextRetryMs = now + RETRY_INTERVAL_MS;
  }
  return true;
}

void printCsvHeader() {
  Serial.println(
      "time_ms,mag_x_uT,mag_y_uT,mag_z_uT,mag_norm_uT,"
      "ir_ambient_C,ir_object_C,rtd_raw,rtd_ohm,rtd_C,"
      "mag_valid,ir_valid,rtd_valid,rtd_fault,"
      "mag_cal_x_uT,mag_cal_y_uT,mag_cal_z_uT,mag_cal_norm_uT,"
      "mag_cal_valid,mag_cal_science_ready");
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(POWER_STABILIZE_MS);

  pinMode(PIN_MAX31865_CS, OUTPUT);
  digitalWrite(PIN_MAX31865_CS, HIGH);
  SPI.begin();
  Wire.setSDA(PIN_LIS3MDL_SDA);
  Wire.setSCL(PIN_LIS3MDL_SCL);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  Wire1.setSDA(PIN_MLX90614_SDA);
  Wire1.setSCL(PIN_MLX90614_SCL);
  Wire1.begin();
  Wire1.setClock(MLX90614_I2C_CLOCK_HZ);
  delay(BUS_STABILIZE_MS);
  delay(WIRE_STABILIZE_MS - BUS_STABILIZE_MS);

  Serial.println();
  Serial.println("# TEENSY 4.1 THREE-SENSOR LOGGER");
  Serial.print("# MAG_CAL id=");
  Serial.print(magcal::kId);
  Serial.print(" enabled=");
  Serial.print(magcal::kEnabled ? 1 : 0);
  Serial.print(" science_ready=");
  Serial.println(magcal::kScienceReady ? 1 : 0);
  Serial.print("# SPI MAX31865: CS=");
  Serial.print(PIN_MAX31865_CS);
  Serial.print(" MOSI=");
  Serial.print(BOARD_SPI_MOSI);
  Serial.print(" MISO=");
  Serial.print(BOARD_SPI_MISO);
  Serial.print(" SCK=");
  Serial.println(BOARD_SPI_SCK);
  Serial.print("# MAX31865 SPI: clock=");
  Serial.print(MAX31865_SPI_CLOCK_HZ);
  Serial.print("Hz mode=");
  Serial.println(MAX31865_SPI_MODE);
  Serial.println("# CSV mag_* = LIS3MDL; magnetic field in uT");
  Serial.print("# I2C LIS3MDL Wire: SDA=");
  Serial.print(PIN_LIS3MDL_SDA);
  Serial.print(" SCL=");
  Serial.print(PIN_LIS3MDL_SCL);
  Serial.print(" clock=");
  Serial.println(I2C_CLOCK_HZ);
  Serial.print("# I2C MLX90614 Wire1: SDA=");
  Serial.print(PIN_MLX90614_SDA);
  Serial.print(" SCL=");
  Serial.print(PIN_MLX90614_SCL);
  Serial.print(" clock=");
  Serial.println(MLX90614_I2C_CLOCK_HZ);

  lisMagnetometerReady = beginWithRetries("LIS3MDL", beginLisMagnetometer);
  infraredReady = beginWithRetries("MLX90614", beginInfrared);
  rtdReady = beginWithRetries("MAX31865", beginRtd);

  printCsvHeader();
  lastRetryMs = millis();
  mlxNextRetryMs = infraredReady ? 0 : millis() + MLX90614_COOLDOWN_MS;
  lastPeriodicReinitMs = millis();
}

void loop() {
  const uint32_t now = millis();
  if (now - lastPeriodicReinitMs >= PERIODIC_REINIT_INTERVAL_MS) {
    reinitializeSensors("periodic_5min");
    return;
  }
  // Do not read other sensors while MLX90614 initialization is running.
  if (retryInfrared(now)) return;
  retryMissingSensors(now);
  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) return;
  lastSampleMs = now;

  float magX = NAN, magY = NAN, magZ = NAN, magNorm = NAN;
  float irAmbientC = NAN, irObjectC = NAN;
  uint16_t rtdRaw = 0;
  float rtdOhm = NAN, rtdC = NAN;
  uint8_t rtdFault = 0;
  bool magValid = false, irValid = false, rtdValid = false;
  float magCal[4] = {NAN, NAN, NAN, NAN};
  bool magCalValid = false;

  if (lisMagnetometerReady) {
    magValid = lisMagnetometer.readMagneticField(magX, magY, magZ) &&
               isfinite(magX) && isfinite(magY) && isfinite(magZ);
    if (magValid) {
      magNorm = sqrtf(magX * magX + magY * magY + magZ * magZ);
      magCalValid = magcal::apply(magX, magY, magZ, magCal);
    } else {
      magX = magY = magZ = NAN;
      lisMagnetometerReady = false;
      Serial.println("# LIS3MDL read failed; scheduling Wire retry");
    }
  }

  if (infraredReady) {
    irAmbientC = infrared.readAmbientTempC();
    irObjectC = infrared.readObjectTempC();
    irValid = isfinite(irAmbientC) && isfinite(irObjectC) &&
              irAmbientC > -70.0f && irAmbientC < 390.0f &&
              irObjectC > -70.0f && irObjectC < 390.0f;
    if (!irValid) {
      Serial.println("# MLX90614 read failed; scheduling isolated retry");
      infraredReady = false;
      mlxRuntimeFailures = 0;
      mlxNextRetryMs = now + RETRY_INTERVAL_MS;
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
  for (float value : magCal) {
    Serial.print(','); printFloatOrNan(value, 3);
  }
  Serial.print(','); Serial.print(magCalValid ? 1 : 0);
  Serial.print(','); Serial.print(magCalValid && magcal::kScienceReady ? 1 : 0);
  Serial.println();

  const bool sampleHasNan = !isfinite(magX) || !isfinite(magY) ||
                            !isfinite(magZ) || !isfinite(magNorm) ||
                            !isfinite(irAmbientC) || !isfinite(irObjectC) ||
                            !isfinite(rtdOhm) || !isfinite(rtdC);
  if (sampleHasNan && nanRecoveryArmed) {
    nanRecoveryArmed = false;
    reinitializeSensors("nan_detected");
  } else if (!sampleHasNan) {
    nanRecoveryArmed = true;
  }
}
