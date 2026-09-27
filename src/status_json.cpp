#include "status_json.h"

#include <cstdio>

int formatStatusJson(const StatusSnapshot &s, char *buf, size_t len) {
  // Same precedence as handleStatusQuery() in the protocol region.
  const char *state = s.homing ? "Home" : (s.running ? "Run" : "Idle");
  int n = std::snprintf(buf, len,
      "{\"x\":%ld,\"y\":%ld,\"state\":\"%s\",\"ninaActive\":%s,"
      "\"network\":\"%s\",\"rssi\":%d,\"uptime\":%lu,\"boots\":%lu,"
      "\"reset\":\"%s\",\"fw\":\"%s\"}",
      s.x, s.y, state, s.ninaActive ? "true" : "false",
      s.stationConnected ? "station" : "setup", s.rssi,
      (unsigned long)s.uptimeS, (unsigned long)s.bootCount,
      s.resetReason, s.fwVersion);
  return (n < 0 || (size_t)n >= len) ? -1 : n;
}
