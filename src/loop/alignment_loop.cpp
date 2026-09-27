#include "alignment_loop.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace paloop {
namespace {

// Unsigned half-range comparison: true when t is at or after ref, across millis() wrap.
bool atOrAfter(uint32_t t, uint32_t ref) { return static_cast<uint32_t>(t - ref) < 0x80000000u; }

}  // namespace

const char *outcomeName(Outcome outcome) {
  switch (outcome) {
    case Outcome::None: return "none";
    case Outcome::Finished: return "finished";
    case Outcome::FinishedBestEffort: return "finished_best_effort";
    case Outcome::HaltedCalibrationSuspect: return "halted_calibration_suspect";
    case Outcome::HaltedEstimateDrift: return "halted_estimate_drift";
    case Outcome::HaltedRunaway: return "halted_runaway";
    case Outcome::HaltedUnresponsive: return "halted_unresponsive";
    case Outcome::StoppedByUser: return "stopped_by_user";
    case Outcome::StoppedByNina: return "stopped_by_nina";
    case Outcome::StoppedNoSource: return "stopped_no_source";
    case Outcome::StoppedSourceEnded: return "stopped_source_ended";
  }
  return "none";
}

const char *phaseName(Phase phase) {
  switch (phase) {
    case Phase::Idle: return "idle";
    case Phase::WaitingObservation: return "waiting";
    case Phase::MovingX: return "moving_x";
    case Phase::MovingY: return "moving_y";
    case Phase::Ended: return "ended";
  }
  return "idle";
}

AlignmentLoop::AlignmentLoop(const LoopSettings &settings) : settings_(settings) {}

bool AlignmentLoop::configure(const LoopSettings &settings) {
  if (running()) return false;
  settings_ = settings;
  return true;
}

bool AlignmentLoop::start(uint32_t nowMs, bool ninaActive, bool axesJustMoved) {
  if (running() || ninaActive) return false;
  controller_.reset();
  controller_.resetExecutionFailureStreak();
  monitor_ = ConvergenceMonitor(settings_.toleranceArcmin);
  phase_ = Phase::WaitingObservation;
  outcome_ = Outcome::None;
  reason_ = "Waiting for an error measurement";
  plan_ = AdjustmentPlan::skip("");
  pendingStop_ = false;
  firstObservation_ = true;
  movedSinceLastObservation_ = false;
  lastCommandedMagnitude_ = 0;
  lastTotalErrorArcmin_ = 0;
  lastCapArcmin_ = 0;
  movesCommanded_ = 0;
  freshAfterMs_ = axesJustMoved ? nowMs + settings_.settleMs : nowMs;
  skipRemaining_ = axesJustMoved ? settings_.readingsToSkipAfterMotion : 0;
  lastObservationMs_ = nowMs;
  return true;
}

void AlignmentLoop::stopByUser() {
  if (running()) {
    phase_ = Phase::Ended;
    outcome_ = Outcome::StoppedByUser;
    reason_ = "Stopped by the user";
  }
  pendingStop_ = true;
}

void AlignmentLoop::stopBySource(const std::string &why) {
  if (running()) {
    phase_ = Phase::Ended;
    outcome_ = Outcome::StoppedSourceEnded;
    reason_ = why;
  }
  pendingStop_ = true;
}

LoopAction AlignmentLoop::tick(const TickInput &in) {
  if (pendingStop_) {
    pendingStop_ = false;
    LoopAction stop;
    stop.kind = ActionKind::Stop;
    return stop;
  }
  if (!running()) return LoopAction();
  if (in.ninaActive) return end(Outcome::StoppedByNina, "A PC took control of OAPA", true);
  if (phase_ == Phase::WaitingObservation) return onWaiting(in);
  return onMoving(in);
}

LoopAction AlignmentLoop::end(Outcome outcome, const std::string &why, bool stopMotors) {
  phase_ = Phase::Ended;
  outcome_ = outcome;
  reason_ = why;
  LoopAction action;
  if (stopMotors) action.kind = ActionKind::Stop;
  return action;
}

LoopAction AlignmentLoop::onWaiting(const TickInput &in) {
  const Observation *obs = in.observation;
  if (obs == nullptr || !atOrAfter(obs->receivedMs, freshAfterMs_)) {
    if (static_cast<uint32_t>(in.nowMs - lastObservationMs_) >= settings_.sourceTimeoutMs) {
      return end(Outcome::StoppedNoSource, "No error measurement received", false);
    }
    return LoopAction();
  }

  lastObservationMs_ = in.nowMs;
  freshAfterMs_ = obs->receivedMs + 1;  // never consume the same reading twice
  if (skipRemaining_ > 0) {
    skipRemaining_--;
    reason_ = "Discarding a reading that may predate the last move";
    return LoopAction();
  }

  controller_.updateObservation(obs->azArcmin / 60.0, obs->altArcmin / 60.0);
  double total = std::hypot(obs->azArcmin, obs->altArcmin);
  lastTotalErrorArcmin_ = total;

  ConvergenceDecision decision =
      monitor_.observe(total, lastCommandedMagnitude_, movedSinceLastObservation_, firstObservation_);
  firstObservation_ = false;
  movedSinceLastObservation_ = false;
  reason_ = decision.reason;

  switch (decision.action) {
    case ConvergenceAction::Finish: return end(Outcome::Finished, decision.reason, false);
    case ConvergenceAction::FinishBestEffort: return end(Outcome::FinishedBestEffort, decision.reason, false);
    case ConvergenceAction::HaltCalibrationSuspect: return end(Outcome::HaltedCalibrationSuspect, decision.reason, false);
    case ConvergenceAction::HaltEstimateDrift: return end(Outcome::HaltedEstimateDrift, decision.reason, false);
    case ConvergenceAction::AwaitConfirmation: return LoopAction();  // motors stay still
    case ConvergenceAction::Continue: break;
  }

  lastCapArcmin_ = std::min(std::max(AdjustController::kDefaultMaximumMoveMagnitude, total * kMaxMoveFractionOfError),
                            settings_.userCapArcmin);
  controller_.setMaximumMoveMagnitude(lastCapArcmin_);
  controller_.aggressiveCorrections = true;
  plan_ = controller_.createPlan();

  if (!plan_.hasMovement()) {
    reason_ = plan_.reason;
    if (controller_.runawayDetected()) return end(Outcome::HaltedRunaway, plan_.reason, false);
    return LoopAction();
  }

  lastCommandedMagnitude_ = std::max(std::fabs(plan_.x), std::fabs(plan_.y));
  movedSinceLastObservation_ = true;
  executedX_ = 0;
  executedY_ = 0;
  movesCommanded_++;
  return beginAxis(AxisId::X, in.nowMs);
}

LoopAction AlignmentLoop::beginAxis(AxisId axis, uint32_t nowMs) {
  double arcmin = axis == AxisId::X ? plan_.x : plan_.y;
  const AxisBacklash &play = axis == AxisId::X ? settings_.backlashX : settings_.backlashY;
  legIndex_ = 0;
  axisMoved_ = false;
  if (play.mode == motion::BacklashMode::Off || arcmin == 0) {
    legs_[0] = arcmin;  // exactly the original single move
    legCount_ = 1;
  } else {
    motion::BacklashPlan plan = motion::BacklashPlanner::plan(play.mode, static_cast<float>(arcmin), play.plusArcmin,
                                                              play.minusArcmin, engaged_[static_cast<int>(axis)]);
    legCount_ = plan.count;
    for (int i = 0; i < plan.count; ++i) legs_[i] = plan.legs[i];
  }
  return nextLeg(axis, nowMs);
}

// Emits the next leg of the axis in progress that rounds to at least one step;
// once none is left the axis is done and the next axis (or the settle) follows.
LoopAction AlignmentLoop::nextLeg(AxisId axis, uint32_t nowMs) {
  double factor = axis == AxisId::X ? settings_.factorX : settings_.factorY;
  while (legIndex_ < legCount_) {
    double leg = legs_[legIndex_];
    long steps = std::lround(leg * factor);
    if (steps == 0) {
      legIndex_++;
      continue;
    }
    phase_ = axis == AxisId::X ? Phase::MovingX : Phase::MovingY;
    moveStartMs_ = nowMs;
    moveDeadlineMs_ = nowMs + static_cast<uint32_t>(2000.0 * std::labs(steps) / settings_.feed) + 5000u;
    sawRunning_ = false;
    axisMoved_ = true;
    engaged_[static_cast<int>(axis)] = leg > 0 ? motion::Direction::Positive : motion::Direction::Negative;
    LoopAction move;
    move.kind = ActionKind::Move;
    move.axis = axis;
    move.steps = steps;
    move.feed = settings_.feed;
    return move;
  }
  if (axis == AxisId::X) {
    if (axisMoved_) executedX_ = plan_.x;
    return beginAxis(AxisId::Y, nowMs);
  }
  if (axisMoved_) executedY_ = plan_.y;
  return finishExecution(nowMs);
}

LoopAction AlignmentLoop::onMoving(const TickInput &in) {
  int index = phase_ == Phase::MovingX ? 0 : 1;
  if (!in.axisIdle[index]) sawRunning_ = true;

  bool completed = in.axisIdle[index] &&
                   (sawRunning_ || static_cast<uint32_t>(in.nowMs - moveStartMs_) >= kCompletionGraceMs);
  if (completed) {
    legIndex_++;
    return nextLeg(phase_ == Phase::MovingX ? AxisId::X : AxisId::Y, in.nowMs);
  }

  if (!atOrAfter(in.nowMs, moveDeadlineMs_)) return LoopAction();

  LoopAction stop;
  stop.kind = ActionKind::Stop;
  phase_ = Phase::WaitingObservation;
  freshAfterMs_ = in.nowMs + settings_.settleMs;
  skipRemaining_ = settings_.readingsToSkipAfterMotion;

  if (index == 1 && executedX_ != 0) {
    controller_.noteSuccessfulExecution(AdjustmentPlan{executedX_, 0, plan_.isProbe, plan_.reason + " (partial AZ (X) move)"});
    reason_ = "ALT (Y) move timed out after a completed AZ (X) move";
    return stop;
  }

  controller_.noteFailedExecution();
  if (controller_.executionUnresponsive()) {
    end(Outcome::HaltedUnresponsive, "OAPA did not complete 3 consecutive moves", false);
    return stop;
  }
  reason_ = "Move timed out";
  return stop;
}

LoopAction AlignmentLoop::finishExecution(uint32_t nowMs) {
  controller_.noteSuccessfulExecution(AdjustmentPlan{executedX_, executedY_, plan_.isProbe, plan_.reason});
  phase_ = Phase::WaitingObservation;
  freshAfterMs_ = nowMs + settings_.settleMs;
  skipRemaining_ = settings_.readingsToSkipAfterMotion;
  return LoopAction();
}

}  // namespace paloop
