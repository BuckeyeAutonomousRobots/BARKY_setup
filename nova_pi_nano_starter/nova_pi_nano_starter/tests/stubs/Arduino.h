#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string>
extern uint32_t fakeMillis;
inline uint32_t millis() { return fakeMillis; }
class FakeSerial {
 public:
  std::string input, output;
  size_t offset = 0;
  void begin(int) {}
  explicit operator bool() const { return true; }
  int availableForWrite() { return 64; }
  size_t write(const uint8_t* data, size_t count) {
    output.append(reinterpret_cast<const char*>(data), count); return count;
  }
  int available() { return static_cast<int>(input.size() - offset); }
  int read() { return input[offset++]; }
};
extern FakeSerial Serial;
