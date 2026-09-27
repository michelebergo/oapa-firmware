#include "platform_sim.h"

#include <cmath>

namespace paloop {
namespace {

// Output follows the input only when the input pushes past half the band.
double followBand(double input, double output, double band) {
  double half = band / 2.0;
  if (input - output > half) return input - half;
  if (output - input > half) return input + half;
  return output;
}

}  // namespace

PlatformSim::PlatformSim(const SimParams &params) : params_(params), rng_(params.seed ? params.seed : 1) {}

void PlatformSim::track(long stepsX, long stepsY) {
  double ax = params_.x.sign * static_cast<double>(stepsX) / params_.x.trueStepsPerArcmin;
  double ay = params_.y.sign * static_cast<double>(stepsY) / params_.y.trueStepsPerArcmin;
  if (!tracked_) {
    baseX_ = effX_ = ax;
    baseY_ = effY_ = ay;
    tracked_ = true;
    return;
  }
  effX_ = followBand(ax, effX_, params_.x.backlashArcmin);
  effY_ = followBand(ay, effY_, params_.y.backlashArcmin);
}

bool PlatformSim::due(uint32_t nowMs) const {
  return !hasRead_ || static_cast<uint32_t>(nowMs - lastReadMs_) >= params_.refreshMs;
}

void PlatformSim::trueError(double &azArcmin, double &altArcmin) const {
  double dx = effX_ - baseX_;
  double dy = effY_ - baseY_;
  azArcmin = params_.initialAzArcmin + params_.coupling[0][0] * dx + params_.coupling[0][1] * dy;
  altArcmin = params_.initialAltArcmin + params_.coupling[1][0] * dx + params_.coupling[1][1] * dy;
}

Observation PlatformSim::read(uint32_t nowMs) {
  Observation o;
  trueError(o.azArcmin, o.altArcmin);
  o.azArcmin += params_.noiseSigmaArcmin * gaussian();
  o.altArcmin += params_.noiseSigmaArcmin * gaussian();
  o.azArcmin += params_.driftAzArcminPerMin * nowMs / 60000.0;
  o.altArcmin += params_.driftAltArcminPerMin * nowMs / 60000.0;
  o.receivedMs = nowMs;
  hasRead_ = true;
  lastReadMs_ = nowMs;
  return o;
}

double PlatformSim::gaussian() {
  // xorshift32 + Box-Muller: reproducible on MSVC and GCC alike.
  auto next = [this]() {
    rng_ ^= rng_ << 13;
    rng_ ^= rng_ >> 17;
    rng_ ^= rng_ << 5;
    return (rng_ + 1.0) / 4294967297.0;  // (0, 1)
  };
  double u1 = next();
  double u2 = next();
  return std::sqrt(-2.0 * std::log(u1)) * std::cos(6.283185307179586 * u2);
}

}  // namespace paloop
