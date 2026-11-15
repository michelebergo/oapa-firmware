// $R=: a relative move in arcminutes on both axes, carried out by the board with the
// same backlash legs as the alignment's corrections, reported through $R?.
#pragma once
#include "../../src/loop/relative_move.h"

using paloop::ActionKind;
using paloop::AxisId;
using paloop::RelState;
using paloop::RelativeMove;
using paloop::RelativeSettings;
using paloop::TickInput;

static RelativeSettings relBase() {
  RelativeSettings s;
  s.factorX = 60;
  s.factorY = 40;
  s.feed = 1000;
  return s;
}

static TickInput relTick(uint32_t nowMs, bool idleX, bool idleY) {
  TickInput in;
  in.nowMs = nowMs;
  in.axisIdle[0] = idleX;
  in.axisIdle[1] = idleY;
  return in;
}

TEST(REL_MovesAzimuthThenAltitude_InStepsFromTheFactors) {
  RelativeMove m;
  CHECK(m.start(relBase(), 2.0, -1.5, 1000, motion::Direction::Positive, motion::Direction::Positive));
  CHECK(m.state() == RelState::Moving);
  auto a = m.tick(relTick(1000, true, true));
  CHECK(a.kind == ActionKind::Move && a.axis == AxisId::X && a.steps == 120 && a.feed == 1000);
  CHECK(m.tick(relTick(1100, false, true)).kind == ActionKind::None);  // X running
  auto b = m.tick(relTick(2000, true, true));                            // X done: Y starts
  CHECK(b.kind == ActionKind::Move && b.axis == AxisId::Y && b.steps == -60);
  CHECK(m.tick(relTick(2100, true, false)).kind == ActionKind::None);
  CHECK(m.tick(relTick(3000, true, true)).kind == ActionKind::None);
  CHECK(m.state() == RelState::Done);
  CHECK_NEAR(m.commandedAz(), 2.0, 1e-9);
  CHECK_NEAR(m.commandedAlt(), -1.5, 1e-9);
}

TEST(REL_TheCommandedAmountIsWhatRoundsToWholeSteps) {
  RelativeMove m;
  m.start(relBase(), 0.0125, 0, 0, motion::Direction::Positive, motion::Direction::Positive);  // 0.75 step -> 1 step
  auto a = m.tick(relTick(0, true, true));
  CHECK(a.kind == ActionKind::Move && a.steps == 1);
  CHECK_NEAR(m.commandedAz(), 1.0 / 60, 1e-9);
  m.tick(relTick(10, false, true));
  m.tick(relTick(500, true, true));
  CHECK(m.state() == RelState::Done);
  CHECK_NEAR(m.commandedAlt(), 0, 1e-9);
}

TEST(REL_AZeroMoveCompletesAtOnce_WithoutAnAction) {
  RelativeMove m;
  CHECK(m.start(relBase(), 0, 0, 0, motion::Direction::Positive, motion::Direction::Positive));
  CHECK(m.tick(relTick(0, true, true)).kind == ActionKind::None);
  CHECK(m.state() == RelState::Done);
}

TEST(REL_AReversalInFullMode_IsOneLongerLeg_LikeTheLoopsOwnCorrections) {
  RelativeSettings s = relBase();
  s.backlashX.mode = motion::BacklashMode::Full;
  s.backlashX.plusArcmin = 1.0;
  s.backlashX.minusArcmin = 1.0;
  RelativeMove m;
  // Resting on the positive side, asked to go -3': the planner adds the play once, -4'.
  m.start(s, -3.0, 0, 0, motion::Direction::Positive, motion::Direction::Positive);
  auto leg = m.tick(relTick(0, true, true));
  CHECK(leg.kind == ActionKind::Move && leg.axis == AxisId::X && leg.steps == -240);
  m.tick(relTick(10, false, true));
  CHECK(m.tick(relTick(1000, true, true)).kind == ActionKind::None);  // Y has nothing to do
  CHECK(m.state() == RelState::Done);
  CHECK_NEAR(m.commandedAz(), -4.0, 1e-9);
  CHECK(m.engagement(AxisId::X) == motion::Direction::Negative);
}

TEST(REL_AReversalInUnidirectionalMode_OvershootsAndComesBack_NetTheRequest) {
  RelativeSettings s = relBase();
  s.backlashX.mode = motion::BacklashMode::Unidirectional;
  s.backlashX.plusArcmin = 1.0;
  s.backlashX.minusArcmin = 1.0;
  RelativeMove m;
  m.start(s, -3.0, 0, 0, motion::Direction::Positive, motion::Direction::Positive);
  auto first = m.tick(relTick(0, true, true));
  CHECK(first.kind == ActionKind::Move && first.axis == AxisId::X && first.steps == -285);  // -3 - 1 - 0.75
  m.tick(relTick(10, false, true));
  auto second = m.tick(relTick(1000, true, true));
  CHECK(second.kind == ActionKind::Move && second.axis == AxisId::X && second.steps == 105);  // 1 + 0.75
  CHECK(first.steps + second.steps == -180);
  m.tick(relTick(1010, false, true));
  m.tick(relTick(2000, true, true));
  CHECK(m.state() == RelState::Done);
  CHECK_NEAR(m.commandedAz(), -3.0, 1e-9);
  CHECK(m.engagement(AxisId::X) == motion::Direction::Positive);  // arrived travelling positive
}

TEST(REL_AnAxisThatNeverFinishes_FailsWithATimeout_AndStopsTheMotors) {
  RelativeMove m;
  m.start(relBase(), 2.0, 0, 0, motion::Direction::Positive, motion::Direction::Positive);
  m.tick(relTick(0, true, true));
  m.tick(relTick(100, false, true));
  // 120 steps at 1000 steps/s: deadline = 2000*120/1000 + 5000 = 5240 ms after the start
  CHECK(m.tick(relTick(5000, false, true)).kind == ActionKind::None);
  auto a = m.tick(relTick(5300, false, true));
  CHECK(a.kind == ActionKind::Stop);
  CHECK(m.state() == RelState::Failed);
  CHECK(std::strcmp(m.reason(), "timeout AZ (X)") == 0);
}

TEST(REL_StopByUser_WhileMoving_IsAFailedMove_NamedStopped) {
  RelativeMove m;
  m.start(relBase(), 2.0, 2.0, 0, motion::Direction::Positive, motion::Direction::Positive);
  m.tick(relTick(0, true, true));
  m.stopByUser();
  CHECK(m.state() == RelState::Failed);
  CHECK(std::strcmp(m.reason(), "stopped") == 0);
  CHECK(m.tick(relTick(100, true, true)).kind == ActionKind::None);
}

TEST(REL_StartWhileMoving_IsRefused) {
  RelativeMove m;
  m.start(relBase(), 2.0, 0, 0, motion::Direction::Positive, motion::Direction::Positive);
  m.tick(relTick(0, true, true));
  CHECK(!m.start(relBase(), 1.0, 0, 10, motion::Direction::Positive, motion::Direction::Positive));
  CHECK(m.state() == RelState::Moving);
}

TEST(REL_IdleBeforeAnyMove) {
  RelativeMove m;
  CHECK(m.state() == RelState::Idle);
  CHECK(std::strcmp(paloop::relStateName(RelState::Idle), "idle") == 0);
  CHECK(std::strcmp(paloop::relStateName(RelState::Moving), "moving") == 0);
  CHECK(std::strcmp(paloop::relStateName(RelState::Done), "done") == 0);
  CHECK(std::strcmp(paloop::relStateName(RelState::Failed), "failed") == 0);
}
