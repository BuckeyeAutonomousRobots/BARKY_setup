// Stage 1: IMU + serial validation ONLY. NO PCA9685 or servo output code.
#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>
#include <stdio.h>
#include "protocol.h"

constexpr uint32_t LINK_TIMEOUT_MS = 300;
char rx[nova::MAX_LINE + 1];
size_t rxLength = 0;
bool receiving = false;
char tx[nova::MAX_LINE + 2];
size_t txLength = 0, txOffset = 0;
uint32_t rxErrors = 0, txDrops = 0;
bool imuOK = false, haveHeartbeat = false, haveCommand = false;
uint32_t lastHeartbeat = 0, lastCommand = 0, lastStatus = 0, lastIMUSend = 0;
uint32_t accelTime = 0, gyroTime = 0;
bool freshAccel = false, freshGyro = false;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

bool linkAlive(uint32_t now) {
  return haveHeartbeat && static_cast<uint32_t>(now - lastHeartbeat) < LINK_TIMEOUT_MS;
}

void queueFrame(const char* payload) {
  if (!Serial || txOffset < txLength) { ++txDrops; return; }
  int length = snprintf(tx, sizeof(tx), "$%s*%04X\n", payload,
                        static_cast<unsigned>(nova::crc16(payload, strlen(payload))));
  if (length <= 0 || static_cast<size_t>(length) >= sizeof(tx)) { ++txDrops; return; }
  txOffset = 0;
  txLength = static_cast<size_t>(length);
}

void flushOutput() {
  if (!Serial) {
    txOffset = 0;
    txLength = 0;
    return;
  }

  if (txOffset >= txLength) {
    return;
  }

  size_t remaining = txLength - txOffset;

  size_t written = Serial.write(
    reinterpret_cast<const uint8_t*>(tx + txOffset),
    remaining
  );

  txOffset += written;
}

void receiveInput(uint32_t now) {
  // Bound work per loop so noisy input cannot starve the IMU/watchdog.
  for (int budget = 0; budget < 256 && Serial.available(); ++budget) {
    char c = static_cast<char>(Serial.read());
    if (c == '$') { rxLength = 0; receiving = true; rx[rxLength++] = c; }
    else if (receiving && c == '\n') {
      rx[rxLength] = '\0';
      receiving = false;
      nova::Command command;
      if (!nova::parse(rx, command)) { ++rxErrors; continue; }
      if (command.kind == 'H') {
        haveHeartbeat = true;
        lastHeartbeat = now;
      } else {
        // Receipt acknowledgement only: angles are NEVER applied to hardware.
        haveCommand = true;
        lastCommand = command.sequence;
        char payload[48];
        snprintf(payload, sizeof(payload), "A,%lu,DRY_RUN", static_cast<unsigned long>(lastCommand));
        queueFrame(payload);
      }
    } else if (receiving) {
      if (c < 32 || c > 126 || rxLength == nova::MAX_LINE) {
        receiving = false; ++rxErrors;
      } else rx[rxLength++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  // Do not wait for Serial: watchdog and sensor work must not depend on a host.
  imuOK = IMU.begin();
}

void loop() {
  uint32_t now = millis();
  flushOutput();
  receiveInput(now);
  if (imuOK && IMU.accelerationAvailable() && IMU.readAcceleration(ax, ay, az)) {
    accelTime = now; freshAccel = true;
  }
  if (imuOK && IMU.gyroscopeAvailable() && IMU.readGyroscope(gx, gy, gz)) {
    gyroTime = now; freshGyro = true;
  }
  // Prioritize 5 Hz status. Values are fixed-width uint32 on the wire.
  if (static_cast<uint32_t>(now - lastStatus) >= 200) {
    lastStatus = now;
    char payload[160];
    snprintf(payload, sizeof(payload), "S,1,%lu,%u,%u,%u,%lu,%lu,%lu",
      static_cast<unsigned long>(now), static_cast<unsigned>(imuOK),
      static_cast<unsigned>(linkAlive(now)), static_cast<unsigned>(haveCommand),
      static_cast<unsigned long>(lastCommand), static_cast<unsigned long>(rxErrors),
      static_cast<unsigned long>(txDrops));
    queueFrame(payload);
  }
  flushOutput(); 
  now = millis();
  if (freshAccel && freshGyro && static_cast<uint32_t>(now - lastIMUSend) >= 20 &&
      static_cast<uint32_t>(now - accelTime) < 50 && static_cast<uint32_t>(now - gyroTime) < 50) {
    const uint32_t elapsed = now - lastIMUSend;
lastIMUSend += (elapsed / 20U) * 20U;
    freshAccel = freshGyro = false;
    // Arduino library returns g and degrees/second. Convert to SI here once.
    constexpr float G = 9.80665f, DEG_TO_RAD_SI = 0.0174532925199433f;
    if (isfinite(ax) && isfinite(ay) && isfinite(az) &&
        isfinite(gx) && isfinite(gy) && isfinite(gz)) {
      char payload[220];
      snprintf(payload, sizeof(payload), "I,%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
        static_cast<unsigned long>(now), ax*G, ay*G, az*G,
        gx*DEG_TO_RAD_SI, gy*DEG_TO_RAD_SI, gz*DEG_TO_RAD_SI);
      queueFrame(payload);
    }
  }
  flushOutput();
}
