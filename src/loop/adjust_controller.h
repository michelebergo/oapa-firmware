// Port of AutomatedAdjustmentController.cs (TPPA plugin): learns a local linear
// actuator model delta_error ~= A * command from observed error changes and uses
// it to choose bounded correction moves. Errors are [azimuth, altitude] in
// degrees; commands are [X, Y] in the system's nudge units (axis arcminutes for
// OAPA). Constants and decision order follow the C# original.
#pragma once
#include <cstddef>
#include <deque>
#include <string>

namespace paloop {

struct AdjustmentPlan {
  double x = 0;
  double y = 0;
  bool isProbe = false;
  std::string reason;

  bool hasMovement() const { return x != 0 || y != 0; }
  static AdjustmentPlan skip(std::string why) {
    AdjustmentPlan plan;
    plan.reason = std::move(why);
    return plan;
  }
};

class AdjustController {
 public:
  static constexpr double kDefaultProbeMagnitude = 1.0;
  static constexpr double kProbeErrorFraction = 0.15;
  static constexpr double kMinimumMoveMagnitude = 0.05;
  static constexpr double kDefaultMaximumMoveMagnitude = 5.0;
  static constexpr double kMinimumConfigurableMoveMagnitude = 1.0;
  static constexpr double kMaximumConfigurableMoveMagnitude = 60.0;
  static constexpr double kNormalEquationDamping = 1e-6;
  static constexpr double kMinimumExpectedImprovementFactor = 0.99;
  static constexpr double kModelResetWorseningFactor = 1.05;
  static constexpr size_t kMaxSamples = 12;
  static constexpr int kMaxConsecutiveWorsenings = 3;
  static constexpr double kWorseningNoiseMarginDegrees = 0.05 / 60.0;
  static constexpr double kCalibrationSuspectResponseDegrees = 1.0 / 60.0;
  static constexpr int kMaxConsecutiveFailedExecutions = 3;

  // Opt-in OAPA profile: probes scale with the error, adds a 75% candidate.
  bool aggressiveCorrections = false;

  size_t sampleCount() const { return samples_.size(); }
  bool runawayDetected() const { return runawayDetected_; }
  bool runawayLikelyEstimateDrift() const { return runawayLikelyEstimateDrift_; }
  double maximumMoveMagnitude() const { return maximumMoveMagnitude_; }
  void setMaximumMoveMagnitude(double value);
  bool hasResponseModel() const;

  void reset();
  void updateObservation(double azimuthErrorDegrees, double altitudeErrorDegrees);
  AdjustmentPlan createPlan();
  void noteSuccessfulExecution(const AdjustmentPlan &plan);
  int consecutiveFailedExecutions() const { return consecutiveFailedExecutions_; }
  bool executionUnresponsive() const { return consecutiveFailedExecutions_ >= kMaxConsecutiveFailedExecutions; }
  void resetExecutionFailureStreak() { consecutiveFailedExecutions_ = 0; }
  void noteFailedExecution();

 private:
  struct ErrorReading {
    double az = 0;
    double alt = 0;
    double total() const;
  };
  struct ResponseSample { double x, y, dAz, dAlt; };
  struct ResponseModel { double azPerX, azPerY, altPerX, altPerY; };

  void addSample(const ResponseSample &sample);
  AdjustmentPlan createProbePlan() const;
  AdjustmentPlan createCorrectivePlan(const ResponseModel &model, const ErrorReading &reading) const;
  AdjustmentPlan createScaledPlan(double x, double y, double scale, const char *why) const;
  bool tryCreateSingleAxisPlan(double azPerUnit, double altPerUnit, const ErrorReading &reading, bool xAxis,
                               AdjustmentPlan &plan) const;
  bool tryBuildResponseModel(ResponseModel &model) const;
  double normalizeMagnitude(double magnitude) const;

  std::deque<ResponseSample> samples_;
  ErrorReading current_;
  bool hasObservation_ = false;
  bool hasPendingPlan_ = false;
  AdjustmentPlan pendingPlan_;
  ErrorReading pendingBefore_;
  int consecutiveWorsenings_ = 0;
  double streakLargestObservedResponseDegrees_ = 0;
  double maximumMoveMagnitude_ = kDefaultMaximumMoveMagnitude;
  bool runawayDetected_ = false;
  bool runawayLikelyEstimateDrift_ = false;
  int consecutiveFailedExecutions_ = 0;
};

}  // namespace paloop
