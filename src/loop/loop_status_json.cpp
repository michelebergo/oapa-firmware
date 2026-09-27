#include "loop_status_json.h"

#include <cstdio>

#include "../json_util.h"

int formatLoopJson(const LoopStatusSnapshot &s, char *buf, size_t len) {
  char reason[512];
  if (!jsonEscape(s.reason, reason, sizeof reason)) return -1;
  int n = std::snprintf(buf, len,
                        "{\"phase\":\"%s\",\"outcome\":\"%s\",\"reason\":\"%s\",\"simulation\":%s,"
                        "\"source\":\"%s\",\"sourceReady\":%s,\"reading\":%s,"
                        "\"az\":%.2f,\"alt\":%.2f,\"total\":%.2f,\"moves\":%d,"
                        "\"plan\":{\"x\":%.3f,\"y\":%.3f,\"probe\":%s},\"cap\":%.2f,\"passMaxUs\":%lu}",
                        s.phase, s.outcome, reason, s.simulation ? "true" : "false", s.source,
                        s.sourceReady ? "true" : "false", s.hasReading ? "true" : "false",
                        s.azArcmin, s.altArcmin, s.totalArcmin, s.moves, s.planX, s.planY,
                        s.planProbe ? "true" : "false", s.capArcmin, static_cast<unsigned long>(s.passMaxUs));
  return (n < 0 || static_cast<size_t>(n) >= len) ? -1 : n;
}
