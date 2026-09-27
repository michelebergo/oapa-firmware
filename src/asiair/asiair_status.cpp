#include "asiair_status.h"

#include <cstdio>

#include "../json_util.h"

int formatAsiairJson(const AsiairSnapshot &s, char *buf, size_t len) {
  char host[40], phase[40], reason[112];
  if (!jsonEscape(s.host, host, sizeof host) || !jsonEscape(s.phase, phase, sizeof phase) ||
      !jsonEscape(s.endReason, reason, sizeof reason)) {
    return -1;
  }
  int n = std::snprintf(buf, len,
                        "{\"connected\":%s,\"host\":\"%s\",\"phase\":\"%s\",\"adjusting\":%s,\"ended\":%s,"
                        "\"endReason\":\"%s\",\"reading\":%s,\"ageMs\":%lu,\"az\":%.2f,\"alt\":%.2f,"
                        "\"lines\":%lu,\"badLines\":%lu,\"rejected\":%lu}",
                        s.connected ? "true" : "false", host, phase, s.adjusting ? "true" : "false",
                        s.ended ? "true" : "false", reason, s.hasReading ? "true" : "false",
                        static_cast<unsigned long>(s.readingAgeMs), s.azArcmin, s.altArcmin,
                        static_cast<unsigned long>(s.lines), static_cast<unsigned long>(s.badLines),
                        static_cast<unsigned long>(s.rejected));
  return (n < 0 || static_cast<size_t>(n) >= len) ? -1 : n;
}
