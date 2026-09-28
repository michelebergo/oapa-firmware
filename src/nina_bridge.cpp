#include "nina_bridge.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace ninabridge {

namespace {

// Two finite numbers separated by a comma and nothing else.
bool parsePair(const char *text, double &a, double &b) {
  char *end = nullptr;
  a = std::strtod(text, &end);
  if (end == text || *end != ',') return false;
  const char *second = end + 1;
  b = std::strtod(second, &end);
  if (end == second) return false;
  while (*end == ' ' || *end == '\r' || *end == '\n') ++end;
  return *end == 0 && std::isfinite(a) && std::isfinite(b);
}

}  // namespace

Command parse(const char *line) {
  Command c;
  if (line == nullptr || line[0] != '$' || line[1] == 0 || line[2] == 0) return c;
  char letter = line[1];
  const char *rest = line + 2;

  if (letter == 'L' && std::strcmp(rest, "?") == 0) {
    c.kind = Kind::StatusQuery;
    return c;
  }
  if (letter == 'K' && std::strcmp(rest, "?") == 0) {
    c.kind = Kind::CalibrationQuery;
    return c;
  }
  if (*rest != '=' || (letter != 'E' && letter != 'F' && letter != 'A' && letter != 'B' && letter != 'T' && letter != 'C' && letter != 'M')) return c;
  const char *value = rest + 1;

  if (letter == 'T') {
    char *end = nullptr;
    c.a = std::strtod(value, &end);
    bool valid = end != value && *end == 0 && std::isfinite(c.a) && c.a > 0 && c.a <= 60;
    c.kind = valid ? Kind::Tolerance : Kind::Invalid;
    return c;
  }
  if (letter == 'M') {
    char *end = nullptr;
    c.a = std::strtod(value, &end);
    bool valid = end != value && *end == 0 && std::isfinite(c.a) && c.a >= 1 && c.a <= 120;
    c.kind = valid ? Kind::MoveCap : Kind::Invalid;
    return c;
  }
  if (letter == 'B') {
    // X,U,14.78,10.21
    c.kind = Kind::Invalid;
    if ((value[0] != 'X' && value[0] != 'Y') || value[1] != ',') return c;
    char mode = value[2];
    if ((mode != 'O' && mode != 'S' && mode != 'F' && mode != 'U') || value[3] != ',') return c;
    if (!parsePair(value + 4, c.a, c.b) || c.a < 0 || c.b < 0) return c;
    c.axis = value[0];
    c.mode = mode;
    c.kind = Kind::Backlash;
    return c;
  }

  if (letter == 'C') {
    if (std::strcmp(value, "1") == 0) c.kind = Kind::CalibrateStart;
    else if (std::strcmp(value, "2") == 0) c.kind = Kind::CalibrateOnly;
    else if (std::strcmp(value, "0") == 0) c.kind = Kind::CalibrateStop;
    else c.kind = Kind::Invalid;
    return c;
  }
  if (letter == 'A') {
    if (std::strcmp(value, "1") == 0) c.kind = Kind::Start;
    else if (std::strcmp(value, "0") == 0) c.kind = Kind::Stop;
    else c.kind = Kind::Invalid;
    return c;
  }
  if (!parsePair(value, c.a, c.b)) {
    c.kind = Kind::Invalid;
    return c;
  }
  if (letter == 'F' && (c.a == 0 || c.b == 0)) {
    c.kind = Kind::Invalid;  // a zero factor would turn every move into no steps at all
    return c;
  }
  c.kind = letter == 'E' ? Kind::Reading : Kind::Factors;
  return c;
}

size_t formatStatus(const Status &status, char *out, size_t len) {
  if (out == nullptr || len == 0) return 0;
  char reason[kReasonMax];
  std::snprintf(reason, sizeof reason, "%s", status.reason ? status.reason : "");
  for (char *p = reason; *p; ++p) {
    if (*p == '|') *p = '/';
  }
  int n;
  if (status.hasReading) {
    n = std::snprintf(out, len, "<L|phase:%s|outcome:%s|source:%s|moves:%d|az:%.2f|alt:%.2f|plan:%.2f,%.2f|reason:%s|>",
                      status.phase, status.outcome, status.source, status.moves, status.azArcmin, status.altArcmin,
                      status.planX, status.planY, reason);
  } else {
    n = std::snprintf(out, len, "<L|phase:%s|outcome:%s|source:%s|moves:%d|az:-|alt:-|plan:%.2f,%.2f|reason:%s|>",
                      status.phase, status.outcome, status.source, status.moves, status.planX, status.planY, reason);
  }
  if (n < 0) return 0;
  return static_cast<size_t>(n) < len ? static_cast<size_t>(n) : len - 1;
}

size_t formatStatusFrame(const char *state, long x, long y, const char *version, char *out, size_t len) {
  if (out == nullptr || len == 0) return 0;
  int n = std::snprintf(out, len, "<%s|MPos:%.2f,%.2f,0.00|V:%s|>", state, static_cast<double>(static_cast<float>(x)),
                        static_cast<double>(static_cast<float>(y)), version);
  if (n < 0) return 0;
  return static_cast<size_t>(n) < len ? static_cast<size_t>(n) : len - 1;
}

size_t formatCalibration(const CalibrationStatus &status, char *out, size_t len) {
  if (out == nullptr || len == 0) return 0;
  char reason[kReasonMax];
  std::snprintf(reason, sizeof reason, "%s", status.reason ? status.reason : "");
  for (char *p = reason; *p; ++p) {
    if (*p == '|') *p = '/';
  }
  char x[32] = "-", y[32] = "-", xp[32] = "-", yp[32] = "-";
  if (status.xValid) std::snprintf(x, sizeof x, "%.2f,%+d", status.xFactor, status.xSign);
  if (status.yValid) std::snprintf(y, sizeof y, "%.2f,%+d", status.yFactor, status.ySign);
  if (status.xPlayValid) {
    std::snprintf(xp, sizeof xp, "%.2f,%.2f,%c", status.xPlayPositive, status.xPlayNegative, status.xMode);
  }
  if (status.yPlayValid) {
    std::snprintf(yp, sizeof yp, "%.2f,%.2f,%c", status.yPlayPositive, status.yPlayNegative, status.yMode);
  }
  int n = std::snprintf(out, len, "<K|state:%s|x:%s|y:%s|xplay:%s|yplay:%s|reason:%s|>", status.state, x, y, xp, yp,
                        reason);
  if (n < 0) return 0;
  return static_cast<size_t>(n) < len ? static_cast<size_t>(n) : len - 1;
}

bool NinaSource::onReading(double azArcmin, double altArcmin, uint32_t nowMs, bool runActive) {
  bool newStream = !streaming(nowMs);
  latest_.azArcmin = azArcmin;
  latest_.altArcmin = altArcmin;
  latest_.receivedMs = nowMs;
  hasReading_ = true;
  return newStream && !runActive;
}

}  // namespace ninabridge
