#include "calibration.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace paloop {
namespace {

// Unsigned half-range comparison: true when t is at or after ref, across millis() wrap.
bool atOrAfter(uint32_t t, uint32_t ref) { return static_cast<uint32_t>(t - ref) < 0x80000000u; }

LoopAction stopAction() {
  LoopAction a;
  a.kind = ActionKind::Stop;
  return a;
}

}  // namespace

const char *calStateName(CalState state) {
  switch (state) {
    case CalState::Idle: return "idle";
    case CalState::Preloading: return "preloading";
    case CalState::Baseline: return "baseline";
    case CalState::Probing: return "probing";
    case CalState::Measuring: return "measuring";
    case CalState::Restoring: return "restoring";
    case CalState::Done: return "done";
    case CalState::Failed: return "failed";
    case CalState::Stopped: return "stopped";
  }
  return "idle";
}

bool Calibration::running() const {
  return state_ == CalState::Preloading || state_ == CalState::Baseline || state_ == CalState::Probing ||
         state_ == CalState::Measuring || state_ == CalState::Restoring;
}

bool Calibration::start(const CalibrationSettings &settings, uint32_t nowMs, bool ninaActive) {
  if (running() || ninaActive) return false;
  settings_ = settings;
  results_[0] = AxisCalibration();
  results_[1] = AxisCalibration();
  pendingStop_ = false;
  freshAfterMs_ = nowMs;
  skipRemaining_ = 0;
  beginAxis(AxisId::X, nowMs);
  return true;
}

const char *Calibration::legName() const {
  switch (leg_) {
    case Leg::TakeUp: return "taking up the play";
    case Leg::Factor: return "measuring the factor";
    case Leg::Reverse: return "reversing";
    case Leg::Return: return "coming back";
  }
  return "";
}

void Calibration::beginAxis(AxisId axis, uint32_t nowMs) {
  axis_ = axis;
  leg_ = Leg::TakeUp;
  state_ = CalState::Baseline;
  axisFailed_ = false;
  probe_ = 0;
  count_ = 0;
  sum_ = 0;
  lastProbe_ = 0;
  legTravel_ = 0;
  net_ = 0;
  response_ = 0;
  lastReadingMs_ = nowMs;
  char text[48];
  std::snprintf(text, sizeof text, "%s: reading where it starts", axisName());
  reason_ = text;
}

void Calibration::finish(CalState state, const std::string &why) {
  state_ = state;
  reason_ = why;
}

void Calibration::stopByUser() {
  if (!running()) return;
  finish(CalState::Stopped, "Stopped by the user");
  pendingStop_ = true;
}

void Calibration::stopBySource(const std::string &why) {
  if (!running()) return;
  finish(CalState::Stopped, why);
  pendingStop_ = true;
}

LoopAction Calibration::tick(const TickInput &in) {
  if (pendingStop_) {
    pendingStop_ = false;
    return stopAction();
  }
  if (!running()) return LoopAction();
  if (in.ninaActive) {
    finish(CalState::Stopped, "A PC took control of OAPA");
    return stopAction();
  }
  if (state_ == CalState::Baseline || state_ == CalState::Measuring) return onReading(in);
  return onMoving(in);
}

LoopAction Calibration::move(CalState next, long steps, uint32_t nowMs) {
  state_ = next;
  net_ += steps;
  moveStartMs_ = nowMs;
  moveDeadlineMs_ = nowMs + static_cast<uint32_t>(2000.0 * std::labs(steps) / settings_.feed) + 5000u;
  sawRunning_ = false;
  LoopAction a;
  a.kind = ActionKind::Move;
  a.axis = axis_;
  a.steps = steps;
  a.feed = settings_.feed;
  return a;
}

// One probe of the current leg, in the leg's direction.
LoopAction Calibration::probe(long steps, uint32_t nowMs) {
  lastProbe_ = steps;
  legTravel_ += steps;
  probe_++;
  char text[96];
  std::snprintf(text, sizeof text, "%s: %s, %ld steps", axisName(), legName(), legTravel_);
  reason_ = text;
  return move(CalState::Probing, legDirection() * steps, nowMs);
}

// The error has not answered yet: double the probe, within the leg's budget.
LoopAction Calibration::nextProbe(uint32_t nowMs) {
  long next = lastProbe_ * 2;
  if (next <= settings_.maxProbeSteps) return probe(next, nowMs);
  char text[96];
  if (leg_ == Leg::TakeUp || leg_ == Leg::Factor) {
    axisFailed_ = true;
    std::snprintf(text, sizeof text, "axis %s does not respond (%.2f' after %ld steps)", axisName(), response_, legTravel_);
  } else {
    // The factor stands; only the play of this axis stays unmeasured.
    std::snprintf(text, sizeof text, "%s: no answer %s within %ld steps, play not measured", axisName(), legName(),
                  legTravel_);
  }
  reason_ = text;
  return restore(nowMs);
}

LoopAction Calibration::restore(uint32_t nowMs) { return move(CalState::Restoring, -net_, nowMs); }

// The error answered by the threshold: close the leg and open the next one.
LoopAction Calibration::afterLeg(double mean, uint32_t nowMs) {
  AxisCalibration &r = results_[static_cast<int>(axis_)];
  double moved = std::fabs(response_);
  char text[96];
  switch (leg_) {
    case Leg::TakeUp: {
      // Engaged now: the same travel again, where no play is left to eat it.
      long travel = legTravel_;
      leg_ = Leg::Factor;
      baseline_ = mean;
      legTravel_ = 0;
      return probe(travel, nowMs);
    }
    case Leg::Factor:
      r.valid = true;
      r.stepsPerArcmin = legTravel_ / moved;
      r.sign = response_ > 0 ? 1 : -1;
      r.responseArcmin = response_;
      r.steps = legTravel_;
      leg_ = Leg::Reverse;
      break;
    case Leg::Reverse:
      r.playEnteringNegative = std::max(0.0, legTravel_ / r.stepsPerArcmin - moved);
      leg_ = Leg::Return;
      break;
    case Leg::Return:
      r.playEnteringPositive = std::max(0.0, legTravel_ / r.stepsPerArcmin - moved);
      r.backlashValid = true;
      std::snprintf(text, sizeof text, "%s: %.2f steps per arcmin, play %.2f' / %.2f', moving back", axisName(),
                    r.stepsPerArcmin, r.playEnteringPositive, r.playEnteringNegative);
      reason_ = text;
      return restore(nowMs);
  }
  baseline_ = mean;
  legTravel_ = 0;
  lastProbe_ = 0;
  return probe(settings_.firstProbeSteps, nowMs);
}

LoopAction Calibration::onReading(const TickInput &in) {
  const Observation *obs = in.observation;
  if (obs == nullptr || !atOrAfter(obs->receivedMs, freshAfterMs_)) {
    if (static_cast<uint32_t>(in.nowMs - lastReadingMs_) >= settings_.sourceTimeoutMs) {
      finish(CalState::Failed, "No error measurement received");
    }
    return LoopAction();
  }
  lastReadingMs_ = in.nowMs;
  freshAfterMs_ = obs->receivedMs + 1;  // never average the same reading twice
  if (skipRemaining_ > 0) {
    skipRemaining_--;
    return LoopAction();
  }
  sum_ += axis_ == AxisId::X ? obs->azArcmin : obs->altArcmin;
  if (++count_ < settings_.readingsPerMean) return LoopAction();
  double mean = sum_ / count_;
  sum_ = 0;
  count_ = 0;

  if (state_ == CalState::Baseline) {
    baseline_ = mean;
    return probe(settings_.firstProbeSteps, in.nowMs);
  }
  response_ = mean - baseline_;
  if (std::fabs(response_) >= settings_.thresholdArcmin) return afterLeg(mean, in.nowMs);
  return nextProbe(in.nowMs);
}

LoopAction Calibration::onMoving(const TickInput &in) {
  int index = static_cast<int>(axis_);
  if (!in.axisIdle[index]) sawRunning_ = true;
  bool completed = in.axisIdle[index] &&
                   (sawRunning_ || static_cast<uint32_t>(in.nowMs - moveStartMs_) >= kCompletionGraceMs);
  if (completed) {
    freshAfterMs_ = in.nowMs + settings_.settleMs;
    skipRemaining_ = settings_.readingsToSkipAfterMotion;
    lastReadingMs_ = in.nowMs;
    count_ = 0;
    sum_ = 0;
    if (state_ == CalState::Probing) {
      state_ = CalState::Measuring;
      return LoopAction();
    }
    // Restoring done.
    if (axisFailed_) {
      finish(CalState::Failed, reason_);
      return LoopAction();
    }
    if (axis_ == AxisId::X) {
      beginAxis(AxisId::Y, in.nowMs);
      return LoopAction();
    }
    finish(CalState::Done, "Calibration complete");
    return LoopAction();
  }
  if (!atOrAfter(in.nowMs, moveDeadlineMs_)) return LoopAction();
  finish(CalState::Failed, "OAPA did not complete a calibration move");
  return stopAction();
}

}  // namespace paloop
