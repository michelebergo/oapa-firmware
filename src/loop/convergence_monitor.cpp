#include "convergence_monitor.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>

namespace paloop {
namespace {

std::string format(const char *fmt, ...) {
  char buf[320];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buf, sizeof buf, fmt, args);
  va_end(args);
  return buf;
}

}  // namespace

ConvergenceMonitor::ConvergenceMonitor(double toleranceArcmin, JitterProvider estimateJitterArcmin)
    : toleranceArcmin_(toleranceArcmin), estimateJitterArcmin_(std::move(estimateJitterArcmin)) {}

ConvergenceDecision ConvergenceMonitor::observe(double totalErrorArcmin, double lastCommandedMagnitudeArcmin,
                                                bool movedSinceLastObservation, bool isFirstObservation) {
  if (isFirstObservation) {
    previousErrorArcmin_ = totalErrorArcmin;
    if (totalErrorArcmin <= toleranceArcmin_) {
      consecutiveBelowTolerance_++;
      if (!minimumAchievedArcmin_ || totalErrorArcmin < *minimumAchievedArcmin_) minimumAchievedArcmin_ = totalErrorArcmin;
      return {ConvergenceAction::AwaitConfirmation,
              format("Below tolerance (%.2f') on the first observation, awaiting confirmation solve (%d/%d).",
                     totalErrorArcmin, consecutiveBelowTolerance_, kRequiredConsecutiveBelowTolerance)};
    }
    return {ConvergenceAction::Continue, "First observation."};
  }

  // (a) A change larger than the noise floor while nothing moved cannot be a real polar-error change.
  if (previousErrorArcmin_ && !movedSinceLastObservation &&
      std::fabs(totalErrorArcmin - *previousErrorArcmin_) > kStationaryDriftArcmin) {
    estimateDegraded_ = true;
  }

  if (movedSinceLastObservation) {
    recentLargestMoveArcmin_ = std::max(recentLargestMoveArcmin_ * 0.5, std::fabs(lastCommandedMagnitudeArcmin));
  }

  ConvergenceDecision decision = classify(totalErrorArcmin);
  previousErrorArcmin_ = totalErrorArcmin;
  return decision;
}

std::optional<double> ConvergenceMonitor::improvementFloor() const {
  if (!estimateJitterArcmin_) return std::nullopt;
  std::optional<double> jitter = estimateJitterArcmin_();
  if (jitter && *jitter > 0) return kImprovementFloorJitterFactor * *jitter;
  return std::nullopt;
}

ConvergenceDecision ConvergenceMonitor::classify(double totalErrorArcmin) {
  if (totalErrorArcmin <= toleranceArcmin_) {
    consecutiveBelowTolerance_++;
    consecutiveWorsenings_ = 0;
    oscillationsSinceMinimum_ = 0;
    if (!minimumAchievedArcmin_ || totalErrorArcmin < *minimumAchievedArcmin_) minimumAchievedArcmin_ = totalErrorArcmin;

    if (consecutiveBelowTolerance_ >= kRequiredConsecutiveBelowTolerance) {
      return {ConvergenceAction::Finish,
              format("Total error %.2f' below tolerance for %d consecutive solves.", totalErrorArcmin, consecutiveBelowTolerance_)};
    }

    std::optional<double> floor = improvementFloor();
    if (floor && totalErrorArcmin > *floor && !estimateDegraded_) {
      return {ConvergenceAction::Continue,
              format("Below tolerance (%.2f') but %.2f' of it is larger than the estimate's own noise; "
                     "correcting once more before the confirmation solve.",
                     totalErrorArcmin, *floor)};
    }

    return {ConvergenceAction::AwaitConfirmation,
            format("Below tolerance (%.2f'), awaiting confirmation solve (%d/%d).", totalErrorArcmin,
                   consecutiveBelowTolerance_, kRequiredConsecutiveBelowTolerance)};
  }

  if (consecutiveBelowTolerance_ > 0 && totalErrorArcmin <= toleranceArcmin_ + kConfirmationMarginArcmin) {
    return {ConvergenceAction::AwaitConfirmation,
            format("Reading %.2f' is within the noise margin above tolerance; holding for another confirmation solve.",
                   totalErrorArcmin)};
  }

  consecutiveBelowTolerance_ = 0;

  if (minimumAchievedArcmin_) {
    oscillationsSinceMinimum_++;
    if (oscillationsSinceMinimum_ >= kBestEffortOscillations || estimateDegraded_) {
      std::string why = estimateDegraded_ ? std::string("stationary drift detected")
                                          : format("%d oscillations", oscillationsSinceMinimum_);
      return {ConvergenceAction::FinishBestEffort,
              format("Best-effort finish at previously achieved %.2f': the estimate no longer improves (%s).",
                     *minimumAchievedArcmin_, why.c_str())};
    }
  }

  if (previousErrorArcmin_ && totalErrorArcmin > toleranceArcmin_ + kConfirmationMarginArcmin &&
      totalErrorArcmin > *previousErrorArcmin_ + kWorseningNoiseArcmin) {
    consecutiveWorsenings_++;
  } else {
    consecutiveWorsenings_ = 0;
  }

  if (consecutiveWorsenings_ >= kMaxConsecutiveWorsenings) {
    if (recentLargestMoveArcmin_ >= kCalibrationSuspectMoveArcmin && !estimateDegraded_) {
      return {ConvergenceAction::HaltCalibrationSuspect,
              format("Error increased for %d consecutive measurements under large corrections; calibration factors "
                     "or backlash compensation are likely wrong.",
                     consecutiveWorsenings_)};
    }
    return {ConvergenceAction::HaltEstimateDrift,
            format("Error increased for %d consecutive measurements while corrections were small; the error estimate "
                   "appears to have drifted. This is not a calibration problem - re-run the alignment to re-measure.",
                   consecutiveWorsenings_)};
  }

  return {ConvergenceAction::Continue, "Continuing corrections."};
}

}  // namespace paloop
