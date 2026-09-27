// Backlash in the loop's own moves: each axis move is planned by the ported
// BacklashPlanner and executed leg by leg; the axis counts as done only after
// its last leg. Off keeps the original single move exactly.
using motion::BacklashMode;
using motion::BacklashPlanner;
using motion::Direction;

static LoopSettings lbSettings(BacklashMode mode, float b) {
  LoopSettings s = lpSettings();
  s.settleMs = 0;
  s.backlashX = {mode, b, b};
  s.backlashY = {mode, b, b};
  return s;
}

// Completes the move in progress on the given axis: running for one tick, then idle.
static LoopAction lbComplete(AlignmentLoop &l, uint32_t &now, AxisId axis) {
  bool x = axis == AxisId::X;
  lpTick(l, now += 50, nullptr, !x, x);
  return lpTick(l, now += 250, nullptr, true, true);
}

// probe X (+), probe Y (+), then the first correction, which reverses X.
static LoopAction lbRunToFirstCorrection(AlignmentLoop &l, uint32_t &now) {
  l.start(0, false);
  Observation o1{30, -20, now = 100};
  lpTick(l, now, &o1);                 // X probe +5.408'
  lbComplete(l, now, AxisId::X);       // Y rounds to zero: waiting
  Observation o2{35.408, -20, now += 100};
  lpTick(l, now, &o2);                 // Y probe
  lbComplete(l, now, AxisId::Y);
  Observation o3{35.408, -14.6, now += 100};
  return lpTick(l, now, &o3);
}

TEST(LB_Off_TheLoopMovesExactlyAsBefore) {
  AlignmentLoop l(lbSettings(BacklashMode::Off, 4));
  l.start(0, false);
  Observation o{30, -20, 100};
  LoopAction a = lpTick(l, 100, &o);
  CHECK(a.kind == ActionKind::Move && a.axis == AxisId::X && a.steps == 324);
}

TEST(LB_Full_AReversalCarriesTheBacklashInTheSameMove) {
  AlignmentLoop l(lbSettings(BacklashMode::Full, 4));
  l.setEngagement(AxisId::X, Direction::Negative);
  l.start(0, false);
  Observation o{30, -20, 100};
  LoopAction a = lpTick(l, 100, &o);
  // +5.408' from a negative engagement: one move of 5.408 + 4 = 9.408' -> 564 steps.
  CHECK(a.kind == ActionKind::Move && a.axis == AxisId::X);
  CHECK(a.steps == std::lround((0.15 * std::sqrt(1300.0) + 4.0) * 60));
  CHECK(l.engagement(AxisId::X) == Direction::Positive);
}

TEST(LB_Full_AMoveInTheEngagedDirectionPaysNothing) {
  AlignmentLoop l(lbSettings(BacklashMode::Full, 4));
  l.start(0, false);
  Observation o{30, -20, 100};
  CHECK(lpTick(l, 100, &o).steps == 324);
}

TEST(LB_Unidirectional_ANegativeCorrection_IsTwoLegs_AndTheAxisWaitsForTheSecond) {
  AlignmentLoop l(lbSettings(BacklashMode::Unidirectional, 4));
  uint32_t now = 0;
  LoopAction first = lbRunToFirstCorrection(l, now);
  double x = l.lastPlan().x;
  CHECK(x < 0);
  auto legs = BacklashPlanner::plan(BacklashMode::Unidirectional, static_cast<float>(x), 4.0f, 4.0f, Direction::Positive);
  CHECK(legs.count == 2);
  CHECK(first.kind == ActionKind::Move && first.axis == AxisId::X);
  CHECK(first.steps == std::lround(legs.legs[0] * 60.0));
  CHECK(l.phase() == Phase::MovingX);

  LoopAction second = lbComplete(l, now, AxisId::X);
  CHECK(second.kind == ActionKind::Move && second.axis == AxisId::X);
  CHECK(second.steps == std::lround(legs.legs[1] * 60.0));
  CHECK(l.phase() == Phase::MovingX);
  CHECK(l.engagement(AxisId::X) == Direction::Positive);

  LoopAction afterX = lbComplete(l, now, AxisId::X);
  CHECK(afterX.kind == ActionKind::Move ? afterX.axis == AxisId::Y : l.phase() == Phase::WaitingObservation);
}

TEST(LB_EngagementSetFromOutside_IsWhatTheNextPlanUses) {
  // A jog from N.I.N.A. or the page moves the axis behind the loop's back; the
  // service reports the direction the axis last moved and the loop plans from it.
  AlignmentLoop l(lbSettings(BacklashMode::Full, 4));
  l.setEngagement(AxisId::X, Direction::Negative);
  CHECK(l.engagement(AxisId::X) == Direction::Negative);
  l.setEngagement(AxisId::X, Direction::Positive);
  l.start(0, false);
  Observation o{30, -20, 100};
  CHECK(lpTick(l, 100, &o).steps == 324);
}
