// Snapshot of everything the web page shows, copied by loop() on core 1 and
// serialized on core 0. Arduino-free for host tests.
#pragma once
#include <cstddef>
#include <cstdint>

struct StatusSnapshot {
  long x = 0;
  long y = 0;
  bool running = false;
  bool homing = false;
  bool ninaActive = false;
  bool stationConnected = false;
  int rssi = 0;
  uint32_t uptimeS = 0;
  uint32_t bootCount = 0;
  const char *resetReason = "UNKNOWN";
  const char *fwVersion = "";
};

// Returns the body length, or -1 when buf is too small.
int formatStatusJson(const StatusSnapshot &s, char *buf, size_t len);
