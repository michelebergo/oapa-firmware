#include "adjust_controller.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

namespace paloop {
namespace {

bool trySolveLeastSquaresCommand(double azPerX, double azPerY, double altPerX, double altPerY, double az, double alt,
                                 double &x, double &y) {
  // Damped least squares min ||e + A u||^2 via (A^T A + lambda I) u = -A^T e.
  const double damping = AdjustController::kNormalEquationDamping;
  double m00 = azPerX * azPerX + altPerX * altPerX + damping;
  double m01 = azPerX * azPerY + altPerX * altPerY;
  double m11 = azPerY * azPerY + altPerY * altPerY + damping;
  double rhs0 = -(azPerX * az + altPerX * alt);
  double rhs1 = -(azPerY * az + altPerY * alt);
  double determinant = m00 * m11 - m01 * m01;
  // Relative to the scale of the system: the responses are degrees per commanded arcminute
  // (about 1/60), so the determinant is of order (1/60)^4 = 8e-8 and an absolute 1e-6
  // refused every well-posed system, leaving only the single-axis fallback at half the error.
  if (std::fabs(determinant) <= damping * m00 * m11) {
    x = 0;
    y = 0;
    return false;
  }
  x = (rhs0 * m11 - rhs1 * m01) / determinant;
  y = (m00 * rhs1 - m01 * rhs0) / determinant;
  return true;
}

}  // namespace

double AdjustController::ErrorReading::total() const { return std::sqrt(az * az + alt * alt); }

void AdjustController::setMaximumMoveMagnitude(double value) {
  maximumMoveMagnitude_ = std::max(kMinimumConfigurableMoveMagnitude, std::min(kMaximumConfigurableMoveMagnitude, value));
}

bool AdjustController::hasResponseModel() const {
  ResponseModel model;
  return tryBuildResponseModel(model);
}

void AdjustController::reset() {
  samples_.clear();
  current_ = ErrorReading();
  hasPendingPlan_ = false;
  hasObservation_ = false;
  consecutiveWorsenings_ = 0;
  streakLargestObservedResponseDegrees_ = 0;
  runawayDetected_ = false;
  runawayLikelyEstimateDrift_ = false;
}

void AdjustController::updateObservation(double azimuthErrorDegrees, double altitudeErrorDegrees) {
  ErrorReading latest{azimuthErrorDegrees, altitudeErrorDegrees};

  if (hasPendingPlan_) {
    double deltaAz = latest.az - pendingBefore_.az;
    double deltaAlt = latest.alt - pendingBefore_.alt;
    addSample({pendingPlan_.x, pendingPlan_.y, deltaAz, deltaAlt});

    if (!pendingPlan_.isProbe) {
      if (latest.total() > pendingBefore_.total() + kWorseningNoiseMarginDegrees) {
        consecutiveWorsenings_++;
        double observedResponse = std::sqrt(deltaAz * deltaAz + deltaAlt * deltaAlt);
        streakLargestObservedResponseDegrees_ = std::max(streakLargestObservedResponseDegrees_, observedResponse);
        if (consecutiveWorsenings_ >= kMaxConsecutiveWorsenings) {
          runawayDetected_ = true;
          runawayLikelyEstimateDrift_ = streakLargestObservedResponseDegrees_ < kCalibrationSuspectResponseDegrees;
        }
      } else {
        consecutiveWorsenings_ = 0;
        streakLargestObservedResponseDegrees_ = 0;
      }

      if (latest.total() > pendingBefore_.total() * kModelResetWorseningFactor) samples_.clear();
    }

    hasPendingPlan_ = false;
  }

  current_ = latest;
  hasObservation_ = true;
}

AdjustmentPlan AdjustController::createPlan() {
  if (!hasObservation_) return AdjustmentPlan::skip("No continuous error measurement is available yet.");

  if (runawayDetected_) {
    char buf[320];
    if (runawayLikelyEstimateDrift_) {
      std::snprintf(buf, sizeof buf,
                    "Automated adjustments halted: the error increased for %d consecutive corrective moves, but the "
                    "mount's measured response to them was negligible. The error estimate has likely drifted; re-run "
                    "the alignment to obtain a fresh measurement.",
                    kMaxConsecutiveWorsenings);
    } else {
      std::snprintf(buf, sizeof buf,
                    "Automated adjustments halted: the error increased for %d consecutive corrective moves. "
                    "Calibration factors or backlash compensation are likely wrong.",
                    kMaxConsecutiveWorsenings);
    }
    return AdjustmentPlan::skip(buf);
  }

  ResponseModel model;
  if (tryBuildResponseModel(model)) {
    AdjustmentPlan corrective = createCorrectivePlan(model, current_);
    if (corrective.hasMovement()) return corrective;
  }
  return createProbePlan();
}

void AdjustController::noteSuccessfulExecution(const AdjustmentPlan &plan) {
  consecutiveFailedExecutions_ = 0;
  if (!hasObservation_ || !plan.hasMovement()) return;
  pendingPlan_ = plan;
  pendingBefore_ = current_;
  hasPendingPlan_ = true;
}

void AdjustController::noteFailedExecution() {
  hasPendingPlan_ = false;
  consecutiveFailedExecutions_++;
}

void AdjustController::addSample(const ResponseSample &sample) {
  samples_.push_back(sample);
  while (samples_.size() > kMaxSamples) samples_.pop_front();
}

AdjustmentPlan AdjustController::createProbePlan() const {
  double xExcitation = 0, yExcitation = 0;
  for (const ResponseSample &sample : samples_) {
    xExcitation += std::fabs(sample.x);
    yExcitation += std::fabs(sample.y);
  }

  double probeMagnitude = kDefaultProbeMagnitude;
  if (aggressiveCorrections) {
    double errorMagnitude = current_.total() * 60.0;
    probeMagnitude = std::max(kDefaultProbeMagnitude, std::min(errorMagnitude * kProbeErrorFraction, maximumMoveMagnitude_ / 2.0));
  }

  if (xExcitation <= yExcitation) return AdjustmentPlan{probeMagnitude, 0, true, "Probing azimuth response"};
  return AdjustmentPlan{0, probeMagnitude, true, "Probing altitude response"};
}

AdjustmentPlan AdjustController::createCorrectivePlan(const ResponseModel &model, const ErrorReading &reading) const {
  double currentNorm = reading.total();
  std::vector<AdjustmentPlan> candidates;

  double rawX = 0, rawY = 0;
  if (trySolveLeastSquaresCommand(model.azPerX, model.azPerY, model.altPerX, model.altPerY, reading.az, reading.alt, rawX,
                                  rawY)) {
    if (aggressiveCorrections) candidates.push_back(createScaledPlan(rawX, rawY, 0.75, "Adaptive two-axis correction"));
    candidates.push_back(createScaledPlan(rawX, rawY, 0.5, "Adaptive two-axis correction"));
    candidates.push_back(createScaledPlan(rawX, rawY, 0.25, "Adaptive two-axis correction"));
    candidates.push_back(createScaledPlan(rawX, rawY, 0.125, "Adaptive two-axis correction"));
  }

  AdjustmentPlan single;
  if (tryCreateSingleAxisPlan(model.azPerX, model.altPerX, reading, true, single)) candidates.push_back(single);
  if (tryCreateSingleAxisPlan(model.azPerY, model.altPerY, reading, false, single)) candidates.push_back(single);

  const AdjustmentPlan *best = nullptr;
  double bestPredictedNorm = currentNorm;
  for (const AdjustmentPlan &candidate : candidates) {
    if (!candidate.hasMovement()) continue;
    double az = reading.az + model.azPerX * candidate.x + model.azPerY * candidate.y;
    double alt = reading.alt + model.altPerX * candidate.x + model.altPerY * candidate.y;
    double predictedNorm = std::sqrt(az * az + alt * alt);
    if (predictedNorm < bestPredictedNorm * kMinimumExpectedImprovementFactor) {
      bestPredictedNorm = predictedNorm;
      best = &candidate;
    }
  }

  return best ? *best : AdjustmentPlan::skip("The learned automation model does not yet predict a safe improvement.");
}

AdjustmentPlan AdjustController::createScaledPlan(double x, double y, double scale, const char *why) const {
  return AdjustmentPlan{normalizeMagnitude(x * scale), normalizeMagnitude(y * scale), false, why};
}

bool AdjustController::tryCreateSingleAxisPlan(double azPerUnit, double altPerUnit, const ErrorReading &reading,
                                               bool xAxis, AdjustmentPlan &plan) const {
  double leverage = azPerUnit * azPerUnit + altPerUnit * altPerUnit;
  if (leverage <= kNormalEquationDamping) return false;

  double command = -((azPerUnit * reading.az) + (altPerUnit * reading.alt)) / leverage;
  command = normalizeMagnitude(command * 0.5);
  if (std::fabs(command) < kMinimumMoveMagnitude) return false;

  plan = xAxis ? AdjustmentPlan{command, 0, false, "Adaptive azimuth correction"}
               : AdjustmentPlan{0, command, false, "Adaptive altitude correction"};
  return true;
}

bool AdjustController::tryBuildResponseModel(ResponseModel &model) const {
  if (samples_.size() < 2) return false;

  double s00 = 0, s01 = 0, s11 = 0, azB0 = 0, azB1 = 0, altB0 = 0, altB1 = 0;
  for (const ResponseSample &sample : samples_) {
    s00 += sample.x * sample.x;
    s01 += sample.x * sample.y;
    s11 += sample.y * sample.y;
    azB0 += sample.x * sample.dAz;
    azB1 += sample.y * sample.dAz;
    altB0 += sample.x * sample.dAlt;
    altB1 += sample.y * sample.dAlt;
  }

  double determinant = s00 * s11 - s01 * s01;
  if (determinant <= kNormalEquationDamping) return false;

  // Reject nearly singular sample sets: the hardware has not been probed in enough independent directions.
  double trace = s00 + s11;
  double discriminant = std::sqrt(std::max(0.0, trace * trace - 4 * determinant));
  double largest = (trace + discriminant) / 2.0;
  double smallest = (trace - discriminant) / 2.0;
  if (smallest <= kNormalEquationDamping || largest / smallest > 1e6) return false;

  double inv00 = s11 / determinant, inv01 = -s01 / determinant, inv11 = s00 / determinant;
  model.azPerX = inv00 * azB0 + inv01 * azB1;
  model.azPerY = inv01 * azB0 + inv11 * azB1;
  model.altPerX = inv00 * altB0 + inv01 * altB1;
  model.altPerY = inv01 * altB0 + inv11 * altB1;
  return true;
}

double AdjustController::normalizeMagnitude(double magnitude) const {
  if (std::fabs(magnitude) < kMinimumMoveMagnitude) return 0;
  if (magnitude > maximumMoveMagnitude_) return maximumMoveMagnitude_;
  if (magnitude < -maximumMoveMagnitude_) return -maximumMoveMagnitude_;
  return magnitude;
}

}  // namespace paloop
