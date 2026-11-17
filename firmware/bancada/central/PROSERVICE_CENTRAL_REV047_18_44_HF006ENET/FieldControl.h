#pragma once
#include <stdint.h>

namespace FieldControl {
constexpr bool counterAfter(uint32_t value, uint32_t previous) {
  return (int32_t)(value - previous) > 0;
}
constexpr bool pulseOn(uint32_t now, uint32_t cycle, uint32_t onTime) {
  return cycle > 0 && onTime > 0 && (onTime >= cycle || now % cycle < onTime);
}
constexpr bool cannonReady(bool water, bool pressure, uint32_t detected,
                           uint32_t now, uint32_t wait) {
  return water && pressure && detected != 0 && (uint32_t)(now - detected) >= wait;
}

struct DirectionGuard {
  static constexpr uint32_t DEAD_TIME_MS = 1000;
  int current = 0;
  int last = 0;
  uint32_t offSince = 0;
  constexpr int request(uint32_t now, int desired) {
    if (desired != -1 && desired != 1) desired = 0;
    if (current != 0 && desired != current) {
      last = current;
      current = 0;
      offSince = now;
    }
    if (desired == 0) return current;
    if (current == desired) return current;
    if (last != 0 && desired != last && (uint32_t)(now - offSince) < DEAD_TIME_MS) return 0;
    current = desired;
    last = desired;
    return current;
  }
};
}
