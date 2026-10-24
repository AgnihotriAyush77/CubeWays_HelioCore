#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <DHT.h>
#include <Wire.h>

#include "index_html.h"

// ===================== Wi-Fi =====================
// Configure these locally before uploading. Do not publish real credentials.
const char* wifiSSID = "YOUR_WIFI_SSID";
const char* wifiPassword = "YOUR_WIFI_PASSWORD";

// ===================== Pins ======================
#define DHT_PIN 26
#define DHT_TYPE DHT22

#define TRIG_PIN 33
#define ECHO_PIN 25

#define TILT_PIN 32

// Digital output from the flame sensor module (OUT -> GPIO27).
// Most LM393 flame modules are active LOW; set to 0 for active-HIGH modules.
#define FLAME_PIN 27
#define FLAME_ACTIVE_LOW 1

// MQ-2 analog output is read on ADC1 GPIO34 (input-only pin, Wi-Fi compatible).
// IMPORTANT: If the MQ-2 module is powered at 5 V, use a voltage divider before GPIO34.
#define MQ2_PIN 34
#define MQ2_WARNING_THRESHOLD 2200  // tune from warmed-up raw readings (0..4095)
#define MQ2_CLEAR_THRESHOLD   2100  // hysteresis: must fall below this to clear
#define MQ2_STARTUP_WARMUP_MS 0UL
#define MQ2_CONFIRM_SAMPLES   1

#define SDA_PIN 21
#define SCL_PIN 22

// Warning LEDs: GPIO12 minor, GPIO14 major, GPIO19 normal
#define LED_MINOR  12
#define LED_MAJOR  14
#define LED_NORMAL 19
const unsigned long MINOR_BLINK_INTERVAL = 500; // 0.5 s toggle interval
const unsigned long MAJOR_BLINK_INTERVAL = 300; // 0.3 s toggle interval
const unsigned long NORMAL_BLINK_INTERVAL = 1000; // 1 s toggle interval
unsigned long lastMinorToggle = 0, lastMajorToggle = 0, lastNormalToggle = 0;
bool minorLedState = false, majorLedState = false, normalLedState = false;

DHT dht(DHT_PIN, DHT_TYPE);
WebServer server(80);

// ================= Sensor values =================
float temperature = NAN;
float humidity = NAN;
float distanceCm = NAN;

float accelX = 0, accelY = 0, accelZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float mpuTemperature = 0;

bool tilted = false;
bool fireDetected = false;
bool mpuOK = false;

int mq2Raw = 0;
bool mq2Warning = false;
bool mq2SampleOK = false;
unsigned long tMq2 = 0;
unsigned long lastMq2Read = 0;
const unsigned long MQ2_SAMPLE_INTERVAL_MS = 100;
unsigned long mq2WarmupStarted = 0;
uint8_t mq2HighCount = 0;
uint8_t mq2ClearCount = 0;

// Sample the flame input continuously with a short two-sample debounce.
const unsigned long FLAME_SAMPLE_INTERVAL = 50;
const uint8_t FLAME_CONFIRM_SAMPLES = 2;
unsigned long lastFlameSample = 0;
bool flameCandidate = false;
uint8_t flameCandidateCount = 0;

unsigned long lastRead = 0;
const unsigned long READ_INTERVAL = 2000;

// A reading older than STALE_MS is reported as null instead of a stale number
const unsigned long STALE_MS = 3UL * READ_INTERVAL;
unsigned long tTemp = 0, tHum = 0, tDist = 0, tMpu = 0;  // millis() of last good reading
unsigned long lastSample = 0;                              // millis() when readSensors() last finished

// ---- Inertial sensor + network helpers ----
unsigned long lastMpuTry = 0;
bool netReported = false;

uint8_t mpuAddr = 0;                         // I2C address the MPU6500 answered on (0 = none)
float prevMpu[6] = {0, 0, 0, 0, 0, 0};       // previous sample, used to detect a frozen/dead data path
uint8_t mpuFrozen = 0;                       // consecutive identical samples
unsigned long mpuFaults = 0;                 // number of runtime faults since boot

const uint8_t MPU_FROZEN_LIMIT = 5;          // 5 bit-identical 6-axis samples in a row cannot happen with a live sensor

// Fast MPU6500 sampling and gyro calibration based on the supplied working sketch.
static const float MPU_ACCEL_LSB_PER_G = 16384.0f;
static const float MPU_GYRO_LSB_PER_DPS = 131.0f;
static const unsigned long MPU_SAMPLE_INTERVAL_MS = 10;  // 100 Hz update target
static const uint16_t GYRO_CALIBRATION_SAMPLES = 500;
static const uint32_t GYRO_CALIBRATION_SPACING_US = 4000;
unsigned long lastMpuSampleMs = 0;
uint32_t lastMpuSampleUs = 0;
float gyroBiasXDps = 0.0f, gyroBiasYDps = 0.0f, gyroBiasZDps = 0.0f;
float rollDeg = 0.0f, pitchDeg = 0.0f, yawDeg = 0.0f;
float orientationZeroRoll = 0.0f, orientationZeroPitch = 0.0f, orientationZeroYaw = 0.0f;
bool gyroCalibrated = false;
bool calibrationBusy = false;
bool calibrationRequested = false;
bool calibrationFailed = false;
uint16_t calibrationAttemptSamples = 0, calibrationValidSamples = 0;
int64_t calibrationSumX = 0, calibrationSumY = 0, calibrationSumZ = 0;
uint32_t lastCalibrationSampleUs = 0;
unsigned long calibrationStartedMs = 0;

// ---- I2C diagnostics ----
bool i2cProbe(uint8_t addr) {                // true if a device ACKs at this address
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

int readWhoAmI(uint8_t addr) {               // MPU WHO_AM_I register 0x75; -1 on bus error
  Wire.beginTransmission(addr);
  Wire.write(0x75);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((int)addr, 1) != 1) return -1;
  return Wire.read();
}

void i2cScan() {
  Serial.printf("[I2C] Scanning bus (SDA=GPIO%d, SCL=GPIO%d)...\n", SDA_PIN, SCL_PIN);
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    if (i2cProbe(addr)) {
      Serial.printf("[I2C]   device ACK at 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0) {
    Serial.println("[I2C] No devices found. Check SDA/SCL wiring, 3V3 + GND, and that the module is not damaged.");
  }
}

bool mpuWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpuAddr ? mpuAddr : 0x68);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool mpuReadRegisters(uint8_t reg, uint8_t *buffer, size_t count) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  size_t got = Wire.requestFrom((int)mpuAddr, (int)count);
  if (got != count) return false;
  for (size_t i = 0; i < count; i++) buffer[i] = Wire.read();
  return true;
}

int16_t combine16(uint8_t hi, uint8_t lo) {
  return (int16_t)((uint16_t)hi << 8 | lo);
}


bool readMpuRaw(int16_t &rawAx, int16_t &rawAy, int16_t &rawAz,
                int16_t &rawTemp, int16_t &rawGx, int16_t &rawGy, int16_t &rawGz) {
  uint8_t raw[14];
  if (!mpuReadRegisters(0x3B, raw, sizeof(raw))) return false;
  rawAx = combine16(raw[0], raw[1]);
  rawAy = combine16(raw[2], raw[3]);
  rawAz = combine16(raw[4], raw[5]);
  rawTemp = combine16(raw[6], raw[7]);
  rawGx = combine16(raw[8], raw[9]);
  rawGy = combine16(raw[10], raw[11]);
  rawGz = combine16(raw[12], raw[13]);
  return true;
}

float wrapDegrees(float value) {
  while (value > 180.0f) value -= 360.0f;
  while (value < -180.0f) value += 360.0f;
  return value;
}

// Establish the currently resting plane as the user-selected orientation zero.
bool seedOrientationReference() {
  int16_t rax, ray, raz, rt, rgx, rgy, rgz;
  if (!readMpuRaw(rax, ray, raz, rt, rgx, rgy, rgz)) return false;
  const float axG = rax / MPU_ACCEL_LSB_PER_G;
  const float ayG = ray / MPU_ACCEL_LSB_PER_G;
  const float azG = raz / MPU_ACCEL_LSB_PER_G;
  const float radToDeg = 57.2957795f;
  rollDeg = atan2f(ayG, azG) * radToDeg;
  pitchDeg = atan2f(-axG, sqrtf(ayG * ayG + azG * azG)) * radToDeg;
  yawDeg = 0.0f; // no magnetometer; yaw reference is relative only
  orientationZeroRoll = rollDeg;
  orientationZeroPitch = pitchDeg;
  orientationZeroYaw = yawDeg;
  accelX = axG * 9.80665f;
  accelY = ayG * 9.80665f;
  accelZ = azG * 9.80665f;
  gyroX = ((rgx / MPU_GYRO_LSB_PER_DPS) - gyroBiasXDps) * DEG_TO_RAD;
  gyroY = ((rgy / MPU_GYRO_LSB_PER_DPS) - gyroBiasYDps) * DEG_TO_RAD;
  gyroZ = ((rgz / MPU_GYRO_LSB_PER_DPS) - gyroBiasZDps) * DEG_TO_RAD;
  mpuTemperature = (rt / 333.87f) + 21.0f;
  lastMpuSampleUs = micros();
  lastMpuSampleMs = millis();
  tMpu = millis();
  return true;
}

// Gyroscope bias calibration: collect stationary raw samples exactly like the supplied sketch.
void runStartupGyroCalibration() {
  Serial.println("[MPU6500] Gyro bias calibration: keep the board completely still for about 2 seconds.");
  int64_t sx = 0, sy = 0, sz = 0;
  uint16_t valid = 0;
  for (uint16_t i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
    int16_t rax, ray, raz, rt, rgx, rgy, rgz;
    if (readMpuRaw(rax, ray, raz, rt, rgx, rgy, rgz)) {
      sx += rgx; sy += rgy; sz += rgz; valid++;
    }
    delay(4);
  }
  if (valid >= 400) {
    gyroBiasXDps = ((float)sx / valid) / MPU_GYRO_LSB_PER_DPS;
    gyroBiasYDps = ((float)sy / valid) / MPU_GYRO_LSB_PER_DPS;
    gyroBiasZDps = ((float)sz / valid) / MPU_GYRO_LSB_PER_DPS;
    gyroCalibrated = true;
    calibrationFailed = false;
    seedOrientationReference();
    Serial.printf("[MPU6500] Gyro bias dps: X=%.3f Y=%.3f Z=%.3f; current plane set as zero.\n", gyroBiasXDps, gyroBiasYDps, gyroBiasZDps);
  } else {
    gyroCalibrated = false;
    calibrationFailed = true;
    Serial.printf("[MPU6500] Startup calibration failed: only %u valid samples.\n", valid);
  }
}

void beginGyroCalibration() {
  calibrationRequested = false;
  calibrationBusy = true;
  calibrationFailed = false;
  calibrationAttemptSamples = 0;
  calibrationValidSamples = 0;
  calibrationSumX = calibrationSumY = calibrationSumZ = 0;
  lastCalibrationSampleUs = 0;
  calibrationStartedMs = millis();
  Serial.println("[MPU6500] Recalibration requested: hold sensor still; current resting plane will become zero.");
}

void finishGyroCalibration() {
  if (calibrationValidSamples >= 400) {
    gyroBiasXDps = ((float)calibrationSumX / calibrationValidSamples) / MPU_GYRO_LSB_PER_DPS;
    gyroBiasYDps = ((float)calibrationSumY / calibrationValidSamples) / MPU_GYRO_LSB_PER_DPS;
    gyroBiasZDps = ((float)calibrationSumZ / calibrationValidSamples) / MPU_GYRO_LSB_PER_DPS;
    gyroCalibrated = true;
    if (!seedOrientationReference()) calibrationFailed = true;
    else calibrationFailed = false;
    Serial.printf("[MPU6500] Recalibrated. Bias dps: X=%.3f Y=%.3f Z=%.3f. New plane set as zero.\n", gyroBiasXDps, gyroBiasYDps, gyroBiasZDps);
  } else {
    gyroCalibrated = false;
    calibrationFailed = true;
    Serial.printf("[MPU6500] Calibration failed: only %u valid samples.\n", calibrationValidSamples);
  }
  calibrationBusy = false;
}

void serviceGyroCalibration() {
  if (calibrationRequested && !calibrationBusy) beginGyroCalibration();
  if (!calibrationBusy) return;
  const uint32_t nowUs = micros();
  if (lastCalibrationSampleUs != 0 && (uint32_t)(nowUs - lastCalibrationSampleUs) < GYRO_CALIBRATION_SPACING_US) return;
  lastCalibrationSampleUs = nowUs;

  int16_t rax, ray, raz, rt, rgx, rgy, rgz;
  if (readMpuRaw(rax, ray, raz, rt, rgx, rgy, rgz)) {
    calibrationSumX += rgx; calibrationSumY += rgy; calibrationSumZ += rgz;
    calibrationValidSamples++;
  }
  calibrationAttemptSamples++;
  if (calibrationAttemptSamples >= GYRO_CALIBRATION_SAMPLES || millis() - calibrationStartedMs > 4500UL) {
    finishGyroCalibration();
  }
}

// Fast 100 Hz sensor sample; complementary roll/pitch filter + integrated yaw from supplied code.
void sampleMpuFast() {
  const unsigned long nowMs = millis();
  if (!mpuOK || calibrationBusy || nowMs - lastMpuSampleMs < MPU_SAMPLE_INTERVAL_MS) return;
  lastMpuSampleMs = nowMs;

  int16_t rax, ray, raz, rt, rgx, rgy, rgz;
  if (!readMpuRaw(rax, ray, raz, rt, rgx, rgy, rgz)) {
    mpuOK = false;
    mpuFaults++;
    lastMpuTry = millis();
    Serial.printf("[MPU6500] Read failure at 0x%02X; retry will be attempted.\n", mpuAddr);
    return;
  }

  const float axG = rax / MPU_ACCEL_LSB_PER_G;
  const float ayG = ray / MPU_ACCEL_LSB_PER_G;
  const float azG = raz / MPU_ACCEL_LSB_PER_G;
  const float gxDps = (rgx / MPU_GYRO_LSB_PER_DPS) - gyroBiasXDps;
  const float gyDps = (rgy / MPU_GYRO_LSB_PER_DPS) - gyroBiasYDps;
  const float gzDps = (rgz / MPU_GYRO_LSB_PER_DPS) - gyroBiasZDps;

  accelX = axG * 9.80665f;
  accelY = ayG * 9.80665f;
  accelZ = azG * 9.80665f;
  gyroX = gxDps * DEG_TO_RAD; // dashboard expects rad/s
  gyroY = gyDps * DEG_TO_RAD;
  gyroZ = gzDps * DEG_TO_RAD;
  mpuTemperature = (rt / 333.87f) + 21.0f;

  const float radToDeg = 57.2957795f;
  const float accelRoll = atan2f(ayG, azG) * radToDeg;
  const float accelPitch = atan2f(-axG, sqrtf(ayG * ayG + azG * azG)) * radToDeg;
  const uint32_t nowUs = micros();
  if (lastMpuSampleUs == 0) {
    rollDeg = accelRoll;
    pitchDeg = accelPitch;
    yawDeg = 0.0f;
    orientationZeroRoll = rollDeg;
    orientationZeroPitch = pitchDeg;
    orientationZeroYaw = yawDeg;
  } else {
    float dt = (nowUs - lastMpuSampleUs) / 1000000.0f;
    if (dt <= 0.0f || dt > 0.15f) dt = 0.01f;
    const float alpha = 0.98f;
    rollDeg = alpha * (rollDeg + gxDps * dt) + (1.0f - alpha) * accelRoll;
    pitchDeg = alpha * (pitchDeg + gyDps * dt) + (1.0f - alpha) * accelPitch;
    yawDeg += gzDps * dt;
    yawDeg = wrapDegrees(yawDeg);
  }
  lastMpuSampleUs = nowUs;
  tMpu = millis();
}

// MPU6500 register setup follows the supplied working sketch (±2 g accelerometer, ±250 degrees/s gyro).
bool initMpu() {
  const uint8_t candidates[2] = {0x68, 0x69};
  for (uint8_t i = 0; i < 2; i++) {
    uint8_t addr = candidates[i];
    if (!i2cProbe(addr)) {
      Serial.printf("[MPU6500] No ACK at 0x%02X\n", addr);
      continue;
    }

    mpuAddr = addr;
    int who = readWhoAmI(addr);
    Serial.printf("[MPU6500] Device at 0x%02X, WHO_AM_I=0x%02X\n", addr, who < 0 ? 0xFF : who);

    // A genuine MPU6500 normally reports WHO_AM_I = 0x70.
    // Reject unexpected IDs rather than treating any I2C device as an MPU6500.
    if (who != 0x70) {
      Serial.printf("[MPU6500] Unexpected WHO_AM_I value at 0x%02X; expected 0x70.\n", addr);
      mpuAddr = 0;
      continue;
    }

    // Wake device, configure filter and full-scale ranges.
    if (!mpuWriteRegister(0x6B, 0x01) ||  // wake using gyro PLL, like supplied working sketch
        !mpuWriteRegister(0x1A, 0x03) ||  // DLPF
        !mpuWriteRegister(0x19, 0x04) ||  // sample divider: 200 Hz nominal rate
        !mpuWriteRegister(0x1B, 0x00) ||  // gyro ±250 deg/s (131 LSB/deg/s)
        !mpuWriteRegister(0x1C, 0x00)) {  // accel ±2 g (16384 LSB/g)
      Serial.println("[MPU6500] Register configuration failed.");
      mpuAddr = 0;
      continue;
    }
    delay(100);
    uint8_t verify[1];
    if (!mpuReadRegisters(0x75, verify, 1)) {
      Serial.println("[MPU6500] Could not read WHO_AM_I after setup.");
      mpuAddr = 0;
      continue;
    }
    mpuFrozen = 0;
    for (uint8_t k = 0; k < 6; k++) prevMpu[k] = 0;
    Serial.printf("[MPU6500] Initialised at 0x%02X\n", mpuAddr);
    return true;
  }
  mpuAddr = 0;
  return false;
}

void announceNetwork() {                         // runs once, as soon as Wi-Fi is connected (at boot or later)
  if (netReported || WiFi.status() != WL_CONNECTED) return;
  netReported = true;
  Serial.print("Wi-Fi connected. Dashboard IP: ");
  Serial.println(WiFi.localIP());
  if (MDNS.begin("heliocore")) {
    Serial.println("Dashboard: http://heliocore.local");
  }
}

// ================= Sensor reading =================
float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (duration == 0) return NAN;

  return duration * 0.0343f / 2.0f;
}

void updateFireDetection() {
  const unsigned long now = millis();
  if (now - lastFlameSample < FLAME_SAMPLE_INTERVAL) return;
  lastFlameSample = now;

  const int activeLevel = FLAME_ACTIVE_LOW ? LOW : HIGH;
  const bool candidate = digitalRead(FLAME_PIN) == activeLevel;
  if (candidate == flameCandidate) {
    if (flameCandidateCount < FLAME_CONFIRM_SAMPLES) flameCandidateCount++;
  } else {
    flameCandidate = candidate;
    flameCandidateCount = 1;
  }

  if (flameCandidateCount >= FLAME_CONFIRM_SAMPLES && fireDetected != flameCandidate) {
    fireDetected = flameCandidate;
    Serial.println(fireDetected
      ? "[FIRE] CRITICAL: onboard flame sensor activated."
      : "[FIRE] Flame input cleared; onboard fire alert inactive.");
  }
}

int readMq2Raw() {
  // Average several ADC samples to reduce short-term noise.
  uint32_t total = 0;
  const uint8_t sampleCount = 8;
  for (uint8_t i = 0; i < sampleCount; i++) {
    total += analogRead(MQ2_PIN);
    delay(2);
  }
  return (int)(total / sampleCount);
}

void updateMq2Warning(int raw) {
  // Trigger immediately when the averaged ADC reading reaches 2300.
  // Clear once the reading falls below the clear threshold.
  if (!mq2Warning && raw >= MQ2_WARNING_THRESHOLD) {
    mq2Warning = true;
    mq2HighCount = 1;
    mq2ClearCount = 0;
    Serial.println("[ATMOSPHERE] WARNING: threshold reached.");
  } else if (mq2Warning && raw < MQ2_CLEAR_THRESHOLD) {
    mq2Warning = false;
    mq2ClearCount = 1;
    mq2HighCount = 0;
    Serial.println("[ATMOSPHERE] Warning cleared: reading below clear threshold.");
  }
}

void readSensors() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();

  if (!isnan(t)) { temperature = t; tTemp = millis(); }
  if (!isnan(h)) { humidity = h; tHum = millis(); }

  float d = readDistance();
  if (!isnan(d)) { distanceCm = d; tDist = millis(); }


  tilted = (digitalRead(TILT_PIN) == HIGH);

  if (!mpuOK && millis() - lastMpuTry >= 5000) {   // retry after a sensor/I2C fault
    lastMpuTry = millis();
    Serial.println("[MPU6500] Not available - retrying initialisation...");
    if (initMpu()) {
      mpuOK = true;
      calibrationRequested = true; // recover with fresh bias calibration and a new zero reference
    }
  }

  Serial.println("---- HelioCore Sensor Data ----");
  Serial.printf("DHT Temperature: %.1f C\n", temperature);
  Serial.printf("DHT Humidity: %.1f %%\n", humidity);
  Serial.printf("Distance: %.1f cm\n", distanceCm);
  Serial.printf("Atmosphere warning: %s\n", mq2Warning ? "ACTIVE" : "CLEAR");
  Serial.printf("Onboard fire hazard: %s (flame sensor GPIO%d)\n", fireDetected ? "DETECTED" : "CLEAR", FLAME_PIN);
  Serial.printf("Orientation: %s (tilt GPIO%d raw %s)\n", tilted ? "DEVIATION" : "NORMAL", TILT_PIN, tilted ? "HIGH" : "LOW");
  if (mpuOK) {
    float mag = sqrtf(accelX * accelX + accelY * accelY + accelZ * accelZ);
    Serial.printf("MPU6500 @0x%02X: OK (runtime faults so far: %lu)\n", mpuAddr, mpuFaults);
    Serial.printf("Accel: X %.2f | Y %.2f | Z %.2f m/s2 (|a| %.2f, ~9.8 when still)\n", accelX, accelY, accelZ, mag);
    Serial.printf("Gyro:  X %.2f | Y %.2f | Z %.2f rad/s\n", gyroX, gyroY, gyroZ);
    Serial.printf("MPU Temp: %.1f C\n", mpuTemperature);
  } else {
    Serial.printf("MPU6500: UNAVAILABLE (runtime faults so far: %lu) - dashboard shows no motion data\n", mpuFaults);
  }

  lastSample = millis();
}

// ===================== HTTP handlers =================
void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleHealth() { 
  server.sendHeader("Access-Control-Allow-Origin", "*"); 
  server.send(200, "application/json", "{\"ok\":true,\"device\":\"HelioCore-CubeWays\"}"); 
}

// ---- JSON helpers: never emit nan/inf (invalid JSON); stale or missing values become null ----
bool fresh(float v, unsigned long t) {
  return !isnan(v) && !isinf(v) && (millis() - t) <= STALE_MS;
}

void addNum(String &s, const char* key, float v, bool ok, int decimals) {
  s += '"';
  s += key;
  s += "\":";
  if (ok && !isnan(v) && !isinf(v)) s += String(v, decimals);
  else s += "null";
  s += ',';
}

String buildDataJson() {
  bool tOk = fresh(temperature, tTemp);
  bool hOk = fresh(humidity, tHum);
  bool dOk = fresh(distanceCm, tDist);
  bool mq2Ok = mq2SampleOK && (millis() - tMq2) <= STALE_MS;
  bool mOk = mpuOK && (millis() - tMpu) <= STALE_MS;

  String s;
  s.reserve(600);
  s = "{";
  addNum(s, "temperature", temperature, tOk, 1);
  addNum(s, "humidity", humidity, hOk, 1);
  addNum(s, "distance", distanceCm, dOk, 1);
  addNum(s, "mq2Raw", (float)mq2Raw, mq2Ok, 0);
  s += "\"mq2Warning\":";
  s += mq2Warning ? "true," : "false,";
  s += "\"mq2Ready\":";
  s += (millis() - mq2WarmupStarted >= MQ2_STARTUP_WARMUP_MS) ? "true," : "false,";
  s += "\"fireDetected\":";
  s += fireDetected ? "true," : "false,";
  s += "\"tilt\":";
  s += tilted ? "true," : "false,";
  addNum(s, "accelX", accelX, mOk, 2);
  addNum(s, "accelY", accelY, mOk, 2);
  addNum(s, "accelZ", accelZ, mOk, 2);
  addNum(s, "gyroX", gyroX, mOk, 2);
  addNum(s, "gyroY", gyroY, mOk, 2);
  addNum(s, "gyroZ", gyroZ, mOk, 2);
  addNum(s, "mpuTemperature", mpuTemperature, mOk, 1);
  addNum(s, "roll", wrapDegrees(rollDeg - orientationZeroRoll), mOk, 2);
  addNum(s, "pitch", wrapDegrees(pitchDeg - orientationZeroPitch), mOk, 2);
  addNum(s, "yaw", wrapDegrees(yawDeg - orientationZeroYaw), mOk, 2);
  s += "\"gyroCalibrated\":";
  s += gyroCalibrated ? "true," : "false,";
  s += "\"calibrationBusy\":";
  s += calibrationBusy ? "true," : "false,";
  s += "\"calibrationFailed\":";
  s += calibrationFailed ? "true," : "false,";
  s += "\"mpuOk\":";
  s += mOk ? "true" : "false";
  s += ",\"sampleAgeMs\":";
  s += lastSample ? String((unsigned long)(millis() - lastSample)) : String(-1);
  s += ",\"uptimeMs\":";
  s += String((unsigned long)millis());
  s += '}';
  return s;
}

void handleData() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buildDataJson());
}

void handleGyroCalibrationRequest() {
  if (!mpuOK) {
    server.send(503, "application/json", "{\"ok\":false,\"message\":\"MPU6500 is unavailable\"}");
    return;
  }
  if (calibrationBusy || calibrationRequested) {
    server.send(409, "application/json", "{\"ok\":false,\"message\":\"Calibration is already in progress\"}");
    return;
  }
  calibrationRequested = true;
  calibrationFailed = false;
  server.send(202, "application/json", "{\"ok\":true,\"message\":\"Calibration queued; keep the MPU6500 still\"}");
}

// Ultrasonic thresholds: <=15 cm major, >15 and <=50 cm minor.
// Tilt remains a major warning. GPIO12 blinks at 0.5 s; GPIO14 at 0.3 s;
// GPIO19 blinks at 1 s only when no warning is active.
void updateWarningLeds() {
  const bool validDistance = !isnan(distanceCm) && !isinf(distanceCm) && distanceCm >= 0;
  const bool majorWarning = fireDetected || tilted || (validDistance && distanceCm <= 15.0f);
  const bool minorWarning = !majorWarning && (mq2Warning || (validDistance && distanceCm <= 50.0f));
  const unsigned long now = millis();

  if (majorWarning) {
    if (now - lastMajorToggle >= MAJOR_BLINK_INTERVAL) {
      lastMajorToggle = now; majorLedState = !majorLedState;
    }
    digitalWrite(LED_MAJOR, majorLedState ? HIGH : LOW);
  } else {
    majorLedState = false;
    digitalWrite(LED_MAJOR, LOW);
  }

  if (minorWarning) {
    if (now - lastMinorToggle >= MINOR_BLINK_INTERVAL) {
      lastMinorToggle = now; minorLedState = !minorLedState;
    }
    digitalWrite(LED_MINOR, minorLedState ? HIGH : LOW);
  } else {
    minorLedState = false;
    digitalWrite(LED_MINOR, LOW);
  }

  if (!majorWarning && !minorWarning) {
    if (now - lastNormalToggle >= NORMAL_BLINK_INTERVAL) {
      lastNormalToggle = now; normalLedState = !normalLedState;
    }
    digitalWrite(LED_NORMAL, normalLedState ? HIGH : LOW);
  } else {
    normalLedState = false;
    digitalWrite(LED_NORMAL, LOW);
  }
}

// ===================== Setup ========================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n=== HelioCore SatHealth ===");
  Serial.printf("Pins: DHT=%d TRIG=%d ECHO=%d TILT=%d FLAME=%d MQ2_AO=%d SDA=%d SCL=%d\n",
                DHT_PIN, TRIG_PIN, ECHO_PIN, TILT_PIN, FLAME_PIN, MQ2_PIN, SDA_PIN, SCL_PIN);

  dht.begin();

  pinMode(LED_MINOR, OUTPUT);
  pinMode(LED_MAJOR, OUTPUT);
  pinMode(LED_NORMAL, OUTPUT);
  digitalWrite(LED_MINOR, LOW);
  digitalWrite(LED_MAJOR, LOW);
  digitalWrite(LED_NORMAL, LOW);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(TILT_PIN, INPUT_PULLUP);
  pinMode(FLAME_PIN, FLAME_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
  pinMode(MQ2_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(MQ2_PIN, ADC_11db);
  mq2WarmupStarted = millis();
  Serial.printf("[MQ-2] ADC threshold=%d, clear=%d; initial warm-up=%lu ms. Tune thresholds after sensor stabilization.\n",
                MQ2_WARNING_THRESHOLD, MQ2_CLEAR_THRESHOLD, MQ2_STARTUP_WARMUP_MS);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);  // conservative I2C speed for reliable detection
  delay(100);
  i2cScan();

  if (initMpu()) {
    mpuOK = true;
    runStartupGyroCalibration();
  } else {
    Serial.println("WARNING: MPU6500 not found (retrying every 5 s). Dashboard will still run.");
  }

  // ================= Station-only Wi-Fi =================
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(wifiSSID, wifiPassword);

  Serial.print("Connecting to configured Wi-Fi");
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  announceNetwork();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi not connected yet. Check the local SSID/password in firmware.");
    Serial.println("No hotspot is started; the device keeps retrying in the background.");
  }

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/calibrate", HTTP_POST, handleGyroCalibrationRequest);
  server.on("/health", handleHealth);
  server.begin();

  readSensors();
}

unsigned long lastWifiTry = 0;

void loop() {
  server.handleClient();

  serviceGyroCalibration();
  if (!calibrationBusy && !calibrationRequested) sampleMpuFast();

  updateFireDetection();
  // Sample the gas channel independently of the 2-second dashboard telemetry cycle.
  if (millis() - lastMq2Read >= MQ2_SAMPLE_INTERVAL_MS) {
    lastMq2Read = millis();
    mq2Raw = readMq2Raw();
    mq2SampleOK = true;
    tMq2 = millis();
    updateMq2Warning(mq2Raw);
  }
  updateWarningLeds();

  if (millis() - lastRead >= READ_INTERVAL) {
    lastRead = millis();
    readSensors();
    updateWarningLeds();

    announceNetwork();                           // mDNS/IP report if Wi-Fi came up after boot
    if (WiFi.status() != WL_CONNECTED && millis() - lastWifiTry >= 10000) {
      lastWifiTry = millis();                    // do not rely on the driver alone to rejoin the network
      WiFi.reconnect();
    }
  }
}