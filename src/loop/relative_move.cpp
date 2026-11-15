#include "relative_move.h"

#include <cmath>
#include <cstdlib>

namespace paloop {

namespace {

bool atOrAfter(uint32_t now, uint32_t deadline) { return static_cast<int32_t>(now - deadline) >= 0; }

const char *axisName(AxisId axis) { return axis == AxisId::X ? "AZ (X)" : "ALT (Y)"; }

}  // namespace

const char *relStateName(RelState state) {
  switch (state) {
    case RelState::Moving: return "moving";
    case RelState::Done: return "done";
    case RelState::Failed: return "failed";
    default: return "idle";
  }
}

bool RelativeMove::start(const RelativeSettings &settings, double azArcmin, double altArcmin, uint32_t nowMs,
                         motion::Direction engagedX, motion::Direction engagedY) {
  if (state_ == RelState::Moving) return false;
  settings_ = settings;
  requested_[0] = azArcmin;
  requested_[1] = altArcmin;
  commanded_[0] = commanded_[1] = 0;
  stepsCommanded_[0] = stepsCommanded_[1] = 0;
  engaged_[0] = engagedX;
  engaged_[1] = engagedY;
  reason_.clear();
  state_ = RelState::Moving;
  awaitingAxis_ = false;
  (void)nowMs;
  return true;
}

void RelativeMove::stopByUser() {
  if (state_ != RelState::Moving) return;
  state_ = RelState::Failed;
  reason_ = "stopped";
  awaitingAxis_ = false;
}

LoopAction RelativeMove::tick(const TickInput &in) {
  if (state_ != RelState::Moving) return LoopAction();
  if (!awaitingAxis_) {
    // Fresh start: the first tick plans azimuth.
    awaitingAxis_ = true;
    return beginAxis(AxisId::X, in.nowMs);
  }
  int index = static_cast<int>(axis_);
  if (!in.axisIdle[index]) sawRunning_ = true;
  bool completed = in.axisIdle[index] &&
                   (sawRunning_ || static_cast<uint32_t>(in.nowMs - moveStartMs_) >= kCompletionGraceMs);
  if (completed) {
    legIndex_++;
    return nextLeg(axis_, in.nowMs);
  }
  if (!atOrAfter(in.nowMs, moveDeadlineMs_)) return LoopAction();
  std::string why = std::string("timeout ") + axisName(axis_);
  return fail(why.c_str());
}

LoopAction RelativeMove::beginAxis(AxisId axis, uint32_t nowMs) {
  int index = static_cast<int>(axis);
  double arcmin = requested_[index];
  const AxisBacklash &play = axis == AxisId::X ? settings_.backlashX : settings_.backlashY;
  axis_ = axis;
  legIndex_ = 0;
  if (play.mode == motion::BacklashMode::Off || arcmin == 0) {
    legs_[0] = arcmin;
    legCount_ = 1;
  } else {
    motion::BacklashPlan plan = motion::BacklashPlanner::plan(play.mode, static_cast<float>(arcmin), play.plusArcmin,
                                                              play.minusArcmin, engaged_[index]);
    legCount_ = plan.count;
    for (int i = 0; i < plan.count; ++i) legs_[i] = plan.legs[i];
  }
  return nextLeg(axis, nowMs);
}

// Emits the next leg of the axis in progress that rounds to at least one step; once
// none is left the next axis follows, and after altitude the move is done.
LoopAction RelativeMove::nextLeg(AxisId axis, uint32_t nowMs) {
  int index = static_cast<int>(axis);
  double factor = axis == AxisId::X ? settings_.factorX : settings_.factorY;
  while (legIndex_ < legCount_) {
    double leg = legs_[legIndex_];
    long steps = std::lround(leg * factor);
    if (steps == 0) {
      legIndex_++;
      continue;
    }
    stepsCommanded_[index] += steps;
    commanded_[index] = stepsCommanded_[index] / factor;  // reported by $R? while the move runs, too
    moveStartMs_ = nowMs;
    moveDeadlineMs_ = nowMs + static_cast<uint32_t>(2000.0 * std::labs(steps) / settings_.feed) + 5000u;
    sawRunning_ = false;
    engaged_[index] = steps > 0 ? motion::Direction::Positive : motion::Direction::Negative;
    LoopAction move;
    move.kind = ActionKind::Move;
    move.axis = axis;
    move.steps = steps;
    move.feed = settings_.feed;
    move.arcmin = leg;
    return move;
  }
  commanded_[index] = factor > 0 ? stepsCommanded_[index] / factor : 0;
  if (axis == AxisId::X) return beginAxis(AxisId::Y, nowMs);
  state_ = RelState::Done;
  awaitingAxis_ = false;
  return LoopAction();
}

LoopAction RelativeMove::fail(const char *why) {
  state_ = RelState::Failed;
  reason_ = why;
  awaitingAxis_ = false;
  LoopAction stop;
  stop.kind = ActionKind::Stop;
  return stop;
}

}  // namespace paloop
