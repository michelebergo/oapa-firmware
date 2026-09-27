// Host model of the TMC2209 driver: records the last values written.
#pragma once
#include <cstdint>
class Stream;

class TMC2209Stepper {
 public:
  TMC2209Stepper(Stream *, float, uint8_t) {}
  void begin() {}
  void toff(uint8_t) {}
  void pwm_autoscale(bool) {}
  void rms_current(uint16_t mA, float hold) { lastRunMa = mA; lastHold = hold; }
  void microsteps(uint16_t m) { lastMicrosteps = m; }
  uint16_t lastRunMa = 0;
  float lastHold = -1;
  uint16_t lastMicrosteps = 0;
};
