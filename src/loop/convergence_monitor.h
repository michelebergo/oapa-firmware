// Port of ConvergenceMonitor.cs (TPPA plugin): pure decision state machine for
// the fine phase of automated polar alignment. Constants, decision order and
// reason strings follow the C# original; see its comments for the field cases.
#pragma once
#include <functional>
#include <optional>
#include <string>

namespace paloop {

enum class ConvergenceAction { Continue, AwaitConfirmation, Finish, FinishBestEffort, HaltCalibrationSuspect, HaltEstimateDrift };

struct ConvergenceDecision {
  ConvergenceAction action;
  std::string reason;
};

class ConvergenceMonitor {
 public:
  static constexpr double kConfirmationMarginArcmin = 0.1;
  static constexpr int kRequiredConsecutiveBelowTolerance = 2;
  static constexpr int kMaxConsecutiveWorsenings = 3;
  static constexpr double kWorseningNoiseArcmin = 0.05;
  static constexpr double kStationaryDriftArcmin = 0.25;
  static constexpr double kCalibrationSuspectMoveArcmin = 1.0;
  static constexpr int kBestEffortOscillations = 4;
  static constexpr double kImprovementFloorJitterFactor = 3.0;

  using JitterProvider = std::function<std::optional<double>()>;

  explicit ConvergenceMonitor(double toleranceArcmin, JitterProvider estimateJitterArcmin = nullptr);

  bool estimateDegraded() const { return estimateDegraded_; }
  std::optional<double> minimumAchievedArcmin() const { return minimumAchievedArcmin_; }

  ConvergenceDecision observe(double totalErrorArcmin, double lastCommandedMagnitudeArcmin,
                              bool movedSinceLastObservation, bool isFirstObservation = false);

 private:
  std::optional<double> improvementFloor() const;
  ConvergenceDecision classify(double totalErrorArcmin);

  double toleranceArcmin_;
  JitterProvider estimateJitterArcmin_;
  std::optional<double> previousErrorArcmin_;
  int consecutiveBelowTolerance_ = 0;
  int consecutiveWorsenings_ = 0;
  int oscillationsSinceMinimum_ = 0;
  double recentLargestMoveArcmin_ = 0;
  bool estimateDegraded_ = false;
  std::optional<double> minimumAchievedArcmin_;
};

}  // namespace paloop
