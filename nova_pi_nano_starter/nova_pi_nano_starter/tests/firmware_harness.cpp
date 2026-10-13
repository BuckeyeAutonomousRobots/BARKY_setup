#include <iostream>
#include <iterator>
#include "Arduino.h"
#include "Arduino_BMI270_BMM150.h"
uint32_t fakeMillis = 0;
FakeSerial Serial;
FakeIMU IMU;
#include "../firmware/src/main.cpp"
int main() {
  if (nova::crc16("123456789", 9) != 0x29B1) return 2;
  Serial.input = std::string(std::istreambuf_iterator<char>(std::cin), {});
  setup();
  for (fakeMillis = 0; fakeMillis < 650; ++fakeMillis) loop();
  std::cout << Serial.output;
}
