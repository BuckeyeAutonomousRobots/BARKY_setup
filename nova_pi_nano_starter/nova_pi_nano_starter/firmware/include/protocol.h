#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>

namespace nova {
constexpr size_t MAX_LINE = 255;
inline uint16_t crc16(const char* text, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(static_cast<uint8_t>(text[i])) << 8;
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}
inline bool uint32(const char* text, uint32_t& output) {
  if (!*text) return false;
  uint64_t number = 0;
  for (const char* p = text; *p; ++p) {
    if (*p < '0' || *p > '9') return false;
    number = number * 10 + (*p - '0');
    if (number > 0xFFFFFFFFULL) return false;
  }
  output = static_cast<uint32_t>(number);
  return true;
}
struct Command {
  char kind;
  uint32_t sequence;
  float targets[12];
};
// Destructively parse one complete line, WITHOUT its newline.
inline bool parse(char* line, Command& output) {
  const size_t length = strlen(line);
  if (length > MAX_LINE || length < 7 || line[0] != '$') return false;
  char* star = strchr(line, '*');
  if (!star || strlen(star + 1) != 4) return false;
  for (const char* p = line + 1; p < star; ++p)
    if (*p < 32 || *p > 126 || *p == '$') return false;
  unsigned checksum = 0;
  for (char* p = star + 1; *p; ++p) {
    unsigned digit;
    if (*p >= '0' && *p <= '9') digit = *p - '0';
    else if (*p >= 'A' && *p <= 'F') digit = *p - 'A' + 10;
    else if (*p >= 'a' && *p <= 'f') digit = *p - 'a' + 10;
    else return false;
    checksum = checksum * 16 + digit;
  }
  if (crc16(line + 1, star - line - 1) != checksum) return false;
  *star = '\0';
  char* fields[14];
  size_t count = 0;
  char* start = line + 1;
  while (true) {
    if (count == 14 || !*start) return false;
    fields[count++] = start;
    char* comma = strchr(start, ',');
    if (!comma) break;
    *comma = '\0';
    if (!*start) return false;
    start = comma + 1;
  }
  Command parsed = {};
  if (strlen(fields[0]) != 1 || count < 2 || !uint32(fields[1], parsed.sequence)) return false;
  parsed.kind = fields[0][0];
  if (parsed.kind == 'H') {
    if (count != 2) return false;
  } else if (parsed.kind == 'C') {
    if (count != 14) return false;
    for (int i = 0; i < 12; ++i) {
      char* end;
      errno = 0;
      float value = strtof(fields[i + 2], &end);
      if (end == fields[i + 2] || *end || errno || !isfinite(value) || fabsf(value) > 3.0f)
        return false;
      parsed.targets[i] = value;
    }
  } else return false;
  output = parsed;
  return true;
}
}
