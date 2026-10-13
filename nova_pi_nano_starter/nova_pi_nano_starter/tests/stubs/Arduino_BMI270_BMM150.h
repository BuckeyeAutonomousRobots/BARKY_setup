#pragma once
class FakeIMU {
 public:
  bool begin() { return true; }
  bool accelerationAvailable() { return true; }
  bool gyroscopeAvailable() { return true; }
  int readAcceleration(float& x, float& y, float& z) { x = 0; y = 0; z = 1; return 1; }
  int readGyroscope(float& x, float& y, float& z) { x = 90; y = 0; z = 0; return 1; }
};
extern FakeIMU IMU;
