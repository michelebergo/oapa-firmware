#include "backlash_planner.h"

#include <algorithm>
#include <cmath>

namespace motion {

namespace {

int signOf(float value) { return value > 0 ? 1 : value < 0 ? -1 : 0; }

BacklashPlan one(float leg) {
  BacklashPlan p;
  p.count = 1;
  p.legs[0] = leg;
  return p;
}

BacklashPlan two(float first, float second) {
  BacklashPlan p;
  p.count = 2;
  p.legs[0] = first;
  p.legs[1] = second;
  return p;
}

}  // namespace

BacklashPlan BacklashPlanner::plan(BacklashMode mode, float move, float backlashArcmin, Direction last) {
  return plan(mode, move, backlashArcmin, backlashArcmin, last);
}

BacklashPlan BacklashPlanner::plan(BacklashMode mode, float move, float backlashEnteringPositive,
                                   float backlashEnteringNegative, Direction last, float minimumReversalArcmin) {
  int sign = signOf(move);
  int lastSign = last == Direction::Positive ? 1 : -1;

  if (sign == 0 || mode == BacklashMode::Off) return one(move);

  // A reversal finer than the compensation is known to cannot be honoured.
  bool hasPlay = std::max(backlashEnteringPositive, backlashEnteringNegative) > 0.0f;
  bool reverses = mode == BacklashMode::Unidirectional ? (sign < 0 || lastSign < 0) : sign != lastSign;
  if (hasPlay && reverses && minimumReversalArcmin > 0.0f && std::fabs(move) < minimumReversalArcmin) {
    return BacklashPlan{};
  }

  if (mode == BacklashMode::Unidirectional) {
    // Every move arrives travelling positive, so the axis always rests on the same flank.
    if (backlashEnteringPositive <= 0.0f && backlashEnteringNegative <= 0.0f) return one(move);
    if (sign > 0) {
      if (lastSign > 0) return one(move);
      return backlashEnteringPositive > 0.0f ? one(move + backlashEnteringPositive) : one(move);
    }
    // Negative move: overshoot below the target, then arrive from underneath.
    float overshoot = kOvershootFractionOfBacklash *
                          std::max(std::max(backlashEnteringPositive, backlashEnteringNegative), 0.0f) +
                      kOvershootFloorArcmin;
    float reversalPlay = lastSign > 0 ? std::max(0.0f, backlashEnteringNegative) : 0.0f;
    return two(move - reversalPlay - overshoot, std::max(0.0f, backlashEnteringPositive) + overshoot);
  }

  // Soft and Full compensate only actual reversals, in the move's own direction.
  float outward = sign > 0 ? backlashEnteringPositive : backlashEnteringNegative;
  if (sign == lastSign || outward <= 0.0f) return one(move);

  switch (mode) {
    case BacklashMode::Soft:
      return one(move + sign * kSoftFraction * outward);
    case BacklashMode::Full:
      return one(move + sign * outward);
    default:
      return one(move);
  }
}

BacklashMode BacklashPlanner::recommend(float backlashArcmin, float noiseSigmaArcmin) {
  float measurable = std::max(kMeasurableSigmaFactor * noiseSigmaArcmin, kMeasurableFloorArcmin);
  if (backlashArcmin < measurable) return BacklashMode::Off;
  return backlashArcmin <= kLargeBacklashArcmin ? BacklashMode::Full : BacklashMode::Unidirectional;
}

BacklashMode BacklashPlanner::recommend(float backlashEnteringPositive, float backlashEnteringNegative,
                                        float noiseSigmaArcmin) {
  return recommend(std::max(backlashEnteringPositive, backlashEnteringNegative), noiseSigmaArcmin);
}

}  // namespace motion
