// Deterministic OAPA platform seen through a polar-alignment error readout.
// Motor steps -> axis arcminutes (true factor and sign) -> backlash dead band
// -> coupling matrix -> error, plus seeded Gaussian noise on each readout.
// Used by host scenarios and by the board's simulation mode.
#pragma once
#include <cstdint>

#include "observation.h"

namespace paloop {

struct SimAxis {
  double trueStepsPerArcmin = 60;
  int sign = 1;
  double backlashArcmin = 0;
};

struct SimParams {
  SimAxis x;  // azimuth
  SimAxis y;  // altitude
  double coupling[2][2] = {{1, 0}, {0, 1}};  // [az, alt] per [X, Y] arcmin
  double noiseSigmaArcmin = 0.05;
  uint32_t refreshMs = 4000;
  double initialAzArcmin = 30;
  double initialAltArcmin = -20;
  uint32_t seed = 1;
  double driftAzArcminPerMin = 0;  // drift of the readout from device time 0, as seen at night
  double driftAltArcminPerMin = 0;
};

class PlatformSim {
 public:
  explicit PlatformSim(const SimParams &params);

  void track(long stepsX, long stepsY);
  bool due(uint32_t nowMs) const;
  Observation read(uint32_t nowMs);
  void trueError(double &azArcmin, double &altArcmin) const;
  const SimParams &params() const { return params_; }

 private:
  double gaussian();

  SimParams params_;
  bool tracked_ = false;
  double baseX_ = 0, baseY_ = 0;  // axis arcmin at the first track()
  double effX_ = 0, effY_ = 0;    // axis arcmin after the dead band
  bool hasRead_ = false;
  uint32_t lastReadMs_ = 0;
  uint32_t rng_;
};

}  // namespace paloop
