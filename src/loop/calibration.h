// Automatic calibration on the PA error readout, per axis, in four legs. Each
// leg probes with doubling steps until the axis's own error component moves by
// the threshold (every reading is the mean of two):
//   take-up  forward until the error answers: the play is taken up;
//   factor   forward again by the same travel, engaged: steps per arcmin;
//   reverse  backward until the error answers: the travel it did not show is
//            the play entering the negative direction;
//   return   forward until the error answers: the play entering the positive one;
// then back to the start. Pure state machine on the alignment loop's tick types.
#pragma once
#include <cstdint>
#include <string>

#include "alignment_loop.h"

namespace paloop {

struct CalibrationSettings {
  double thresholdArcmin = 2.0;
  uint32_t settleMs = 2000;
  int feed = 1000;
  long firstProbeSteps = 200;  // also the backlash take-up move
  long maxProbeSteps = 12800;
  int readingsPerMean = 2;
  uint32_t sourceTimeoutMs = 60000;
  // Readings to discard after each move, on top of settleMs: TPPA exposes
  // continuously and can deliver one from an image taken mid-move.
  int readingsToSkipAfterMotion = 0;
};

enum class CalState { Idle, Preloading, Baseline, Probing, Measuring, Restoring, Done, Failed, Stopped };

struct AxisCalibration {
  bool valid = false;
  double stepsPerArcmin = 0;  // magnitude: the controller learns the direction itself
  int sign = 0;               // +1 when positive steps raise the error component
  double responseArcmin = 0;
  long steps = 0;              // the clean (engaged) move the factor was measured on
  bool backlashValid = false;  // both reversals answered within the probe budget
  double playEnteringNegative = 0;  // arcmin lost reversing from positive to negative
  double playEnteringPositive = 0;  // arcmin lost reversing back to positive
};

const char *calStateName(CalState state);

class Calibration {
 public:
  static constexpr uint32_t kCompletionGraceMs = 200;

  bool start(const CalibrationSettings &settings, uint32_t nowMs, bool ninaActive);
  void stopByUser();
  void stopBySource(const std::string &why);
  LoopAction tick(const TickInput &in);

  bool running() const;
  CalState state() const { return state_; }
  AxisId axis() const { return axis_; }
  int probe() const { return probe_; }
  long probeSteps() const { return legTravel_; }
  double responseArcmin() const { return response_; }
  const AxisCalibration &result(AxisId axis) const { return results_[static_cast<int>(axis)]; }
  const std::string &reason() const { return reason_; }

 private:
  LoopAction onReading(const TickInput &in);
  LoopAction onMoving(const TickInput &in);
  LoopAction move(CalState next, long steps, uint32_t nowMs);
  void beginAxis(AxisId axis, uint32_t nowMs);
  void finish(CalState state, const std::string &why);
  // What the messages call the axis: its role first, the motor letter after.
  const char *axisName() const { return axis_ == AxisId::X ? "AZ (X)" : "ALT (Y)"; }

  CalibrationSettings settings_;
  CalState state_ = CalState::Idle;
  AxisId axis_ = AxisId::X;
  AxisCalibration results_[2];
  std::string reason_;
  bool pendingStop_ = false;
  enum class Leg { TakeUp, Factor, Reverse, Return };
  LoopAction probe(long steps, uint32_t nowMs);
  LoopAction nextProbe(uint32_t nowMs);
  LoopAction afterLeg(double mean, uint32_t nowMs);
  LoopAction restore(uint32_t nowMs);
  int legDirection() const { return leg_ == Leg::Reverse ? -1 : 1; }
  const char *legName() const;

  Leg leg_ = Leg::TakeUp;
  bool axisFailed_ = false;
  bool sawRunning_ = false;
  int probe_ = 0;
  int count_ = 0;
  long lastProbe_ = 0;   // magnitude of the last probe of the leg
  long legTravel_ = 0;   // magnitude travelled by the probes of the leg
  long net_ = 0;         // signed steps moved on the axis since it began
  double sum_ = 0;
  double baseline_ = 0;
  double response_ = 0;
  uint32_t freshAfterMs_ = 0;
  int skipRemaining_ = 0;
  uint32_t lastReadingMs_ = 0;
  uint32_t moveStartMs_ = 0;
  uint32_t moveDeadlineMs_ = 0;
};

}  // namespace paloop
