// One automated alignment run as a pure state machine (spec 3.2). It mirrors
// the plugin's correction loop: observe -> controller -> convergence monitor ->
// move X then Y -> settle -> next fresh observation. tick() returns at most one
// action and holds no hardware, so every rule is testable on a fake clock.
#pragma once
#include <cstdint>
#include <string>

#include "../motion/backlash_planner.h"
#include "adjust_controller.h"
#include "convergence_monitor.h"
#include "observation.h"

namespace paloop {

enum class AxisId { X = 0, Y = 1 };

// How one axis handles its play in the loop's own moves (BacklashPlanner modes,
// arcminutes entering the positive and the negative direction).
struct AxisBacklash {
  motion::BacklashMode mode = motion::BacklashMode::Off;
  float plusArcmin = 0;
  float minusArcmin = 0;
};

struct LoopSettings {
  double toleranceArcmin = 1.0;
  double userCapArcmin = 30.0;
  uint32_t settleMs = 2000;
  int feed = 1000;
  double factorX = 1.0;  // steps per azimuth arcminute
  double factorY = 1.0;  // steps per altitude arcminute
  uint32_t sourceTimeoutMs = 300000;
  // Readings to discard after each move, on top of settleMs: a source that
  // exposes continuously (TPPA) can deliver one from an image taken mid-move.
  int readingsToSkipAfterMotion = 0;
  AxisBacklash backlashX;
  AxisBacklash backlashY;
};

enum class Phase { Idle, WaitingObservation, MovingX, MovingY, Ended };

enum class Outcome {
  None,
  Finished,
  FinishedBestEffort,
  HaltedCalibrationSuspect,
  HaltedEstimateDrift,
  HaltedRunaway,
  HaltedUnresponsive,
  StoppedByUser,
  StoppedByNina,
  StoppedNoSource,
  StoppedSourceEnded
};

enum class ActionKind { None, Move, Stop };

struct LoopAction {
  ActionKind kind = ActionKind::None;
  AxisId axis = AxisId::X;
  long steps = 0;
  int feed = 0;
};

struct TickInput {
  uint32_t nowMs = 0;
  const Observation *observation = nullptr;  // latest reading, may repeat across ticks
  bool axisIdle[2] = {true, true};
  bool ninaActive = false;
};

const char *outcomeName(Outcome outcome);
const char *phaseName(Phase phase);

class AlignmentLoop {
 public:
  static constexpr uint32_t kCompletionGraceMs = 200;
  static constexpr double kMaxMoveFractionOfError = 0.8;

  explicit AlignmentLoop(const LoopSettings &settings = LoopSettings());

  bool configure(const LoopSettings &settings);  // refused while running
  // axesJustMoved: the run begins right after other moves (a calibration), so the first
  // reading is treated as after one of its own moves: settle, then discard.
  bool start(uint32_t nowMs, bool ninaActive, bool axesJustMoved = false);
  void stopByUser();
  void stopBySource(const std::string &why);  // the error source ended (PA stopped on ASIAIR)
  LoopAction tick(const TickInput &in);

  bool running() const { return phase_ != Phase::Idle && phase_ != Phase::Ended; }
  Phase phase() const { return phase_; }
  Outcome outcome() const { return outcome_; }
  const std::string &reason() const { return reason_; }
  const AdjustmentPlan &lastPlan() const { return plan_; }
  int movesCommanded() const { return movesCommanded_; }
  double lastTotalErrorArcmin() const { return lastTotalErrorArcmin_; }
  double lastCapArcmin() const { return lastCapArcmin_; }
  const LoopSettings &settings() const { return settings_; }

  // The side of its play each axis rests on. It belongs to the mechanism, not to a
  // run: kept across runs, updated by the loop's own legs and, through the service,
  // by any other motion (a jog from N.I.N.A. or the page).
  void setEngagement(AxisId axis, motion::Direction direction) { engaged_[static_cast<int>(axis)] = direction; }
  motion::Direction engagement(AxisId axis) const { return engaged_[static_cast<int>(axis)]; }

 private:
  LoopAction onWaiting(const TickInput &in);
  LoopAction onMoving(const TickInput &in);
  LoopAction beginAxis(AxisId axis, uint32_t nowMs);
  LoopAction nextLeg(AxisId axis, uint32_t nowMs);
  LoopAction finishExecution(uint32_t nowMs);
  LoopAction end(Outcome outcome, const std::string &why, bool stopMotors);

  LoopSettings settings_;
  AdjustController controller_;
  ConvergenceMonitor monitor_{1.0};
  Phase phase_ = Phase::Idle;
  Outcome outcome_ = Outcome::None;
  std::string reason_;
  AdjustmentPlan plan_;
  bool pendingStop_ = false;
  bool firstObservation_ = true;
  bool movedSinceLastObservation_ = false;
  double lastCommandedMagnitude_ = 0;
  double lastTotalErrorArcmin_ = 0;
  double lastCapArcmin_ = 0;
  double executedX_ = 0;
  double executedY_ = 0;
  int movesCommanded_ = 0;
  uint32_t freshAfterMs_ = 0;
  int skipRemaining_ = 0;
  uint32_t lastObservationMs_ = 0;
  uint32_t moveStartMs_ = 0;
  uint32_t moveDeadlineMs_ = 0;
  bool sawRunning_ = false;
  motion::Direction engaged_[2] = {motion::Direction::Positive, motion::Direction::Positive};
  double legs_[2] = {0, 0};
  int legCount_ = 0;
  int legIndex_ = 0;
  bool axisMoved_ = false;
};

}  // namespace paloop
