// A relative move the host asks for in arcminutes ($R=), carried out by the board the
// way the alignment carries out its own corrections: azimuth then altitude, each in
// the backlash legs the planner gives for the side of its play the axis rests on. A
// pure state machine like AlignmentLoop: tick() returns at most one action and holds
// no hardware, so every rule is testable on a fake clock.
#pragma once
#include <cstdint>
#include <string>

#include "../motion/backlash_planner.h"
#include "alignment_loop.h"

namespace paloop {

struct RelativeSettings {
  double factorX = 1.0;  // steps per azimuth arcminute (magnitude)
  double factorY = 1.0;  // steps per altitude arcminute (magnitude)
  int feed = 1000;
  AxisBacklash backlashX;
  AxisBacklash backlashY;
};

enum class RelState { Idle, Moving, Done, Failed };

const char *relStateName(RelState state);

class RelativeMove {
 public:
  static constexpr uint32_t kCompletionGraceMs = AlignmentLoop::kCompletionGraceMs;

  // Starts a move; refused (false) while one is in progress. The amounts are rounded
  // to whole steps per axis; both zero is a valid move that is done at once.
  bool start(const RelativeSettings &settings, double azArcmin, double altArcmin, uint32_t nowMs,
             motion::Direction engagedX, motion::Direction engagedY);
  LoopAction tick(const TickInput &in);
  // A stop from the host or the page: the move ends as failed, "stopped". The caller
  // sends the stop to the motors itself.
  void stopByUser();

  RelState state() const { return state_; }
  bool moving() const { return state_ == RelState::Moving; }
  double commandedAz() const { return commanded_[0]; }
  double commandedAlt() const { return commanded_[1]; }
  const char *reason() const { return reason_.c_str(); }
  motion::Direction engagement(AxisId axis) const { return engaged_[static_cast<int>(axis)]; }

 private:
  LoopAction beginAxis(AxisId axis, uint32_t nowMs);
  LoopAction nextLeg(AxisId axis, uint32_t nowMs);
  LoopAction fail(const char *why);

  RelativeSettings settings_;
  RelState state_ = RelState::Idle;
  std::string reason_;
  double requested_[2] = {0, 0};
  double commanded_[2] = {0, 0};
  motion::Direction engaged_[2] = {motion::Direction::Positive, motion::Direction::Positive};
  AxisId axis_ = AxisId::X;
  double legs_[2] = {0, 0};
  int legCount_ = 0;
  int legIndex_ = 0;
  long stepsCommanded_[2] = {0, 0};
  uint32_t moveStartMs_ = 0;
  uint32_t moveDeadlineMs_ = 0;
  bool sawRunning_ = false;
  bool awaitingAxis_ = false;
};

}  // namespace paloop
