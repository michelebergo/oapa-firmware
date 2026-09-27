// The orchestrator on hand-fed inputs: every rule of spec 3.2 without a simulator.
#pragma once
#include <cmath>
#include <cstdlib>
using paloop::ActionKind;
using paloop::AlignmentLoop;
using paloop::AxisId;
using paloop::LoopAction;
using paloop::LoopSettings;
using paloop::Observation;
using paloop::Outcome;
using paloop::Phase;
using paloop::TickInput;

static LoopSettings lpSettings() {
  LoopSettings s;
  s.factorX = 60;
  s.factorY = 60;
  return s;
}

static LoopAction lpTick(AlignmentLoop &l, uint32_t now, const Observation *obs = nullptr, bool idleX = true,
                         bool idleY = true, bool nina = false) {
  TickInput in;
  in.nowMs = now;
  in.observation = obs;
  in.axisIdle[0] = idleX;
  in.axisIdle[1] = idleY;
  in.ninaActive = nina;
  return l.tick(in);
}

TEST(LP_StartIsRefusedWhileNinaIsActive) {
  AlignmentLoop l(lpSettings());
  CHECK(!l.start(0, true));
  CHECK(!l.running());
  CHECK(l.start(0, false));
  CHECK(l.phase() == Phase::WaitingObservation);
}

TEST(LP_ObservationsOlderThanTheStartAreIgnored) {
  AlignmentLoop l(lpSettings());
  l.start(1000, false);
  Observation old{30, -20, 999};
  CHECK(lpTick(l, 1000, &old).kind == ActionKind::None);
  CHECK(l.movesCommanded() == 0);
}

TEST(LP_FirstObservationAboveTolerance_ProbesAzimuthWithTheOapaProfile) {
  // total 36.06', cap min(max(5, 28.84), 30) = 28.84', probe max(1, min(5.408, 14.42)) = 5.408'
  // -> 5.408 * 60 = 324.4996 -> 324 steps on X.
  AlignmentLoop l(lpSettings());
  l.start(0, false);
  Observation o{30, -20, 100};
  LoopAction a = lpTick(l, 100, &o);
  CHECK(a.kind == ActionKind::Move && a.axis == AxisId::X);
  CHECK(a.steps == 324 && a.feed == 1000);
  CHECK(l.phase() == Phase::MovingX);
  CHECK(l.movesCommanded() == 1);
  CHECK_NEAR(l.lastCapArcmin(), 0.8 * std::sqrt(1300.0), 1e-9);
}

TEST(LP_AnIdleAxisCompletesOnlyAfterTheGrace_AndTheSettleGatesTheNextReading) {
  AlignmentLoop l(lpSettings());
  l.start(0, false);
  Observation o1{30, -20, 100};
  lpTick(l, 100, &o1);                                      // move X at t=100
  CHECK(lpTick(l, 150).kind == ActionKind::None);           // idle but 50 ms < grace
  CHECK(l.phase() == Phase::MovingX);
  CHECK(lpTick(l, 320).kind == ActionKind::None);           // 220 ms: complete; Y has 0 steps
  CHECK(l.phase() == Phase::WaitingObservation);
  Observation early{24.6, -20, 2000};                        // before 320 + 2000
  CHECK(lpTick(l, 2000, &early).kind == ActionKind::None);
  CHECK(l.movesCommanded() == 1);
  Observation fresh{24.6, -20, 2320};
  LoopAction a = lpTick(l, 2320, &fresh);
  CHECK(a.kind == ActionKind::Move && a.axis == AxisId::Y);  // second probe goes to the unexcited axis
}

TEST(LP_AxisSeenRunningCompletesAsSoonAsItIsIdle) {
  AlignmentLoop l(lpSettings());
  l.start(0, false);
  Observation o{30, -20, 100};
  lpTick(l, 100, &o);
  lpTick(l, 120, nullptr, false);
  CHECK(lpTick(l, 140, nullptr, true).kind == ActionKind::None);
  CHECK(l.phase() == Phase::WaitingObservation);
}

TEST(LP_MoveTimeout_StopsAndThreeInARowHaltUnresponsive) {
  AlignmentLoop l(lpSettings());
  l.start(0, false);
  uint32_t now = 100;
  for (int attempt = 1; attempt <= 3; attempt++) {
    Observation o{30, -20, now};
    LoopAction move = lpTick(l, now, &o);
    CHECK(move.kind == ActionKind::Move && move.steps == 324);
    uint32_t deadline = now + 2000u * 324u / 1000u + 5000u;  // 2 x steps/feed + 5 s
    CHECK(lpTick(l, deadline - 1, nullptr, false).kind == ActionKind::None);
    CHECK(lpTick(l, deadline, nullptr, false).kind == ActionKind::Stop);
    now = deadline + 2000;  // settle
  }
  CHECK(l.outcome() == Outcome::HaltedUnresponsive);
  CHECK(!l.running());
}

TEST(LP_NinaTakingOverMidMove_StopsTheMotorsAndEndsTheRun) {
  AlignmentLoop l(lpSettings());
  l.start(0, false);
  Observation o{30, -20, 100};
  lpTick(l, 100, &o);
  CHECK(lpTick(l, 200, nullptr, false, true, true).kind == ActionKind::Stop);
  CHECK(l.outcome() == Outcome::StoppedByNina);
  CHECK(lpTick(l, 300).kind == ActionKind::None);
}

TEST(LP_StopByUser_EmitsOneStop) {
  AlignmentLoop l(lpSettings());
  l.start(0, false);
  l.stopByUser();
  CHECK(l.outcome() == Outcome::StoppedByUser);
  CHECK(lpTick(l, 10).kind == ActionKind::Stop);
  CHECK(lpTick(l, 20).kind == ActionKind::None);
}

TEST(LP_StopBySource_EndsWithTheSourceReasonAndStopsOnce) {
  AlignmentLoop l(lpSettings());
  CHECK(l.start(0, false));
  l.stopBySource("PA stopped on ASIAIR");
  CHECK(l.outcome() == Outcome::StoppedSourceEnded);
  CHECK_EQ_STR(l.reason(), "PA stopped on ASIAIR");
  CHECK_EQ_STR(paloop::outcomeName(l.outcome()), "stopped_source_ended");
  CHECK(lpTick(l, 50).kind == ActionKind::Stop);
  CHECK(lpTick(l, 100).kind == ActionKind::None);
}

TEST(LP_NoReadingForTheSourceTimeout_EndsTheRun) {
  AlignmentLoop l(lpSettings());
  l.start(0, false);
  CHECK(lpTick(l, 299999).kind == ActionKind::None);
  CHECK(l.running());
  lpTick(l, 300000);
  CHECK(l.outcome() == Outcome::StoppedNoSource);
}

TEST(LP_ConfirmationHoldsTheMotorsStill_ThenFinishes) {
  AlignmentLoop l(lpSettings());
  l.start(0, false);
  Observation o1{0.3, 0.2, 100};
  CHECK(lpTick(l, 100, &o1).kind == ActionKind::None);
  CHECK(l.running());
  Observation o2{0.3, 0.2, 4100};
  CHECK(lpTick(l, 4100, &o2).kind == ActionKind::None);
  CHECK(l.outcome() == Outcome::Finished);
  CHECK(l.movesCommanded() == 0);
}

TEST(LP_TheSameReadingIsNeverConsumedTwice) {
  AlignmentLoop l(lpSettings());
  l.start(0, false);
  Observation o{0.3, 0.2, 100};
  lpTick(l, 100, &o);
  lpTick(l, 5000, &o);  // same receivedMs again
  CHECK(l.running());   // a second confirmation would have finished the run
}

TEST(LP_YTimeoutAfterACompletedX_KeepsTheRunGoing) {
  // Two probes teach the response, then a two-axis correction is planned. The
  // controller's two-axis least-squares step is gated by a 1e-6 determinant
  // floor in degree-per-unit terms: a 1'/1' response gives (1/60)^4 ~ 7.7e-8 and
  // never takes it (single-axis corrections only, as in the plugin). A 3'/1'
  // response (factors configured 3x too small) gives ~6.3e-6 and takes it.
  AlignmentLoop l(lpSettings());
  l.start(0, false);
  Observation o1{30, -20, 100};
  LoopAction p1 = lpTick(l, 100, &o1);
  CHECK(p1.kind == ActionKind::Move && p1.axis == AxisId::X);
  lpTick(l, 400);  // X done (grace), Y has 0 steps
  double probeX = static_cast<double>(p1.steps) / 60.0;
  Observation o2{30 - 3 * probeX, -20, 2400};
  LoopAction p2 = lpTick(l, 2400, &o2);
  CHECK(p2.kind == ActionKind::Move && p2.axis == AxisId::Y);
  lpTick(l, 2700);  // Y done
  double probeY = static_cast<double>(p2.steps) / 60.0;
  Observation o3{30 - 3 * probeX, -20 - 3 * probeY, 4700};
  LoopAction c = lpTick(l, 4700, &o3);
  CHECK(c.kind == ActionKind::Move && c.axis == AxisId::X);
  CHECK(l.lastPlan().x != 0 && l.lastPlan().y != 0);
  LoopAction cy = lpTick(l, 5000);  // X done -> Y move
  CHECK(cy.kind == ActionKind::Move && cy.axis == AxisId::Y);
  uint32_t deadline = 5000u + static_cast<uint32_t>(2000.0 * std::labs(cy.steps) / 1000.0) + 5000u;
  CHECK(lpTick(l, deadline, nullptr, true, false).kind == ActionKind::Stop);
  CHECK(l.running());
  CHECK_EQ_STR(l.reason(), "ALT (Y) move timed out after a completed AZ (X) move");
}
