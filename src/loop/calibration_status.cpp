#include "calibration_status.h"

#include <cstdio>

#include "../json_util.h"

int formatCalibrationJson(const CalibrationSnapshot &s, char *buf, size_t len) {
  char reason[200];
  if (!jsonEscape(s.reason, reason, sizeof reason)) return -1;
  int n = std::snprintf(buf, len,
                        "{\"state\":\"%s\",\"axis\":\"%c\",\"probe\":%d,\"steps\":%ld,\"response\":%.2f,"
                        "\"reason\":\"%s\",\"x\":{\"valid\":%s,\"factor\":%.2f,\"sign\":%d},"
                        "\"y\":{\"valid\":%s,\"factor\":%.2f,\"sign\":%d},\"offset\":{\"x\":%ld,\"y\":%ld}}",
                        s.state, s.axis, s.probe, s.steps, s.responseArcmin, reason, s.xValid ? "true" : "false",
                        s.xFactor, s.xSign, s.yValid ? "true" : "false", s.yFactor, s.ySign, s.offsetX, s.offsetY);
  return (n < 0 || static_cast<size_t>(n) >= len) ? -1 : n;
}
