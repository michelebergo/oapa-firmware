#include "device_config.h"

#include <cmath>
#include <cstdio>

#include "web_bridge.h"

namespace devset {

const char *validateLoop(const LoopConfig &c) {
  if (!(c.factorX > 0 && c.factorX <= 100000) || !(c.factorY > 0 && c.factorY <= 100000)) {
    return "factors must be > 0 and <= 100000 steps per arcminute";
  }
  if (!(c.toleranceArcmin >= 0.1 && c.toleranceArcmin <= 10)) return "tolerance must be 0.1-10 arcminutes";
  if (!(c.userCapArcmin >= 1 && c.userCapArcmin <= 120)) return "move cap must be 1-120 arcminutes";
  if (c.settleMs > 30000) return "settle must be 0-30000 ms";
  if (c.feed < 50 || c.feed > 3000) return "feed must be 50-3000 steps/s";
  if (!(c.calThresholdArcmin >= 1 && c.calThresholdArcmin <= 20)) return "calibration threshold must be 1-20 arcminutes";
  return nullptr;
}

const char *validateHost(const char *host) {
  static const char *kMessage = "address must be empty (automatic) or like 192.168.1.53";
  if (!host) return kMessage;
  if (!*host) return nullptr;
  int parts = 0;
  const char *p = host;
  for (;;) {
    int digits = 0, value = 0;
    while (*p >= '0' && *p <= '9') {
      if (++digits > 3) return kMessage;
      value = value * 10 + (*p - '0');
      p++;
    }
    if (digits == 0 || value > 255) return kMessage;
    parts++;
    if (*p == 0) break;
    if (*p != '.' || parts == 4) return kMessage;
    p++;
  }
  return parts == 4 ? nullptr : kMessage;
}

const char *validateDriver(const DriverConfig &c) {
  if (c.runMa < 100 || c.runMa > 2000) return "run current must be 100-2000 mA";
  if (c.holdPct < 0 || c.holdPct > 100) return "hold must be 0-100 %";
  switch (c.microsteps) {
    case 1: case 2: case 4: case 8: case 16: case 32: case 64: case 128: case 256: return nullptr;
    default: return "microsteps must be 1, 2, 4, 8, 16, 32, 64, 128 or 256";
  }
}

bool driverCommands(char axis, const DriverConfig &c, char out[3][12]) {
  if ((axis != 'X' && axis != 'Y') || validateDriver(c) != nullptr) return false;
  std::snprintf(out[0], 12, "S%c%d", axis, c.microsteps);
  std::snprintf(out[1], 12, "C%c%d", axis, c.runMa);
  std::snprintf(out[2], 12, "H%c%d", axis, c.holdPct);
  return true;
}

bool moveCommand(char axis, double arcmin, double factor, int feed, char *out, size_t len) {
  if (!(factor > 0) || !std::isfinite(arcmin)) return false;
  double steps = arcmin * factor;
  if (std::fabs(steps) > 1e7) return false;
  return web::buildJogCommand(axis, std::lround(steps), feed, out, len);
}

}  // namespace devset
