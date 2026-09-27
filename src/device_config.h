// Settings the page can change and the commands built from them. Pure, host-tested.
#pragma once
#include <cstddef>
#include <cstdint>

namespace devset {

struct LoopConfig {
  double factorX = 1;  // steps per azimuth arcminute
  double factorY = 1;  // steps per altitude arcminute
  double toleranceArcmin = 1.0;
  double userCapArcmin = 30;
  uint32_t settleMs = 2000;
  int feed = 1000;
  double calThresholdArcmin = 2.0;  // calibration response threshold
};

struct DriverConfig {
  int runMa = 600;
  int holdPct = 25;
  int microsteps = 16;
};

// nullptr when valid, otherwise a message for the user.
const char *validateLoop(const LoopConfig &config);
const char *validateDriver(const DriverConfig &config);
const char *validateHost(const char *host);  // nullptr for "" (automatic) or a dotted IPv4

// "S<axis><n>", "C<axis><mA>", "H<axis><pct>": the plugin's type-first grammar,
// microsteps first as in its startup batch.
bool driverCommands(char axis, const DriverConfig &config, char out[3][12]);

// Relative jog of arcmin on axis using its factor; false when it rounds to 0 steps.
bool moveCommand(char axis, double arcmin, double factor, int feed, char *out, size_t len);

}  // namespace devset
