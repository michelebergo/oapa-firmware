// Spec 5.2: each scenario lists the outcomes its mechanism allows; anything else fails.
#pragma once
#include <cstdio>
#include <initializer_list>

static ScenarioOptions scBase() {
  ScenarioOptions o;
  o.settings.factorX = 60;
  o.settings.factorY = 60;
  return o;
}

static bool scExpect(const ScenarioResult &r, std::initializer_list<paloop::Outcome> allowed) {
  for (paloop::Outcome o : allowed) {
    if (r.outcome == o) return true;
  }
  std::printf("  scenario ended %s after %u ms, %d moves: %s\n", paloop::outcomeName(r.outcome), r.elapsedMs, r.moves,
              r.reason.c_str());
  return false;
}

// The monitor judges the measured error; the true error may exceed tolerance by the readout noise.
static bool scNearTolerance(const ScenarioResult &r, const ScenarioOptions &o) {
  bool ok = r.finalTrueErrorArcmin <= o.settings.toleranceArcmin + 3 * o.sim.noiseSigmaArcmin;
  if (!ok) std::printf("  final true error %.3f'\n", r.finalTrueErrorArcmin);
  return ok;
}

TEST(SC_DefaultPlatform_Finishes) {
  auto o = scBase();
  auto r = runScenario(o);
  CHECK(scExpect(r, {Outcome::Finished}));
  CHECK(scNearTolerance(r, o));
  CHECK(r.capViolations == 0);
}

TEST(SC_AzimuthMotorWiredReversed_Finishes) {
  auto o = scBase();
  o.sim.x.sign = -1;
  auto r = runScenario(o);
  CHECK(scExpect(r, {Outcome::Finished}));
  CHECK(scNearTolerance(r, o));
  CHECK(r.capViolations == 0);
}

TEST(SC_TrueFactorsThreeTimesTheConfiguredOnes_Finishes) {
  auto o = scBase();
  o.sim.x.trueStepsPerArcmin = 180;
  o.sim.y.trueStepsPerArcmin = 180;
  auto r = runScenario(o);
  CHECK(scExpect(r, {Outcome::Finished}));
  CHECK(scNearTolerance(r, o));
  CHECK(r.capViolations == 0);
}

TEST(SC_StrongAxisCoupling_Finishes) {
  auto o = scBase();
  o.sim.coupling[0][1] = 0.3;
  o.sim.coupling[1][0] = 0.3;
  auto r = runScenario(o);
  CHECK(scExpect(r, {Outcome::Finished}));
  CHECK(scNearTolerance(r, o));
  CHECK(r.capViolations == 0);
}

TEST(SC_TwoArcminBacklash_FinishesOrFinishesBestEffort) {
  auto o = scBase();
  o.sim.x.backlashArcmin = 2.0;
  o.sim.y.backlashArcmin = 2.0;
  auto r = runScenario(o);
  CHECK(scExpect(r, {Outcome::Finished, Outcome::FinishedBestEffort}));
  CHECK(r.capViolations == 0);
}

TEST(SC_OneReadingEveryTwoMinutes_Finishes) {
  auto o = scBase();
  o.sim.refreshMs = 120000;
  o.maxMs = 6u * 60u * 60u * 1000u;
  auto r = runScenario(o);
  CHECK(scExpect(r, {Outcome::Finished}));
  CHECK(scNearTolerance(r, o));
}

TEST(SC_MotorsThatNeverMove_HaltUnresponsive) {
  auto o = scBase();
  o.motorsDead = true;
  auto r = runScenario(o);
  CHECK(scExpect(r, {Outcome::HaltedUnresponsive}));
}

TEST(SC_NinaTakingOverMidRun_StopsTheMotors) {
  // The default run finishes in about 52 s, so N.I.N.A. arrives at 10 s: well inside it.
  auto o = scBase();
  o.ninaActiveFromMs = 10000;
  auto r = runScenario(o);
  CHECK(scExpect(r, {Outcome::StoppedByNina}));
  CHECK(r.lastActionKind == paloop::ActionKind::Stop);
  CHECK(r.moves > 0);             // it really was mid-run
  CHECK(r.elapsedMs == 10000);    // and it stopped on the first tick N.I.N.A. was active
}

TEST(SC_SourceThatNeverDelivers_EndsAfterTheTimeout) {
  auto o = scBase();
  o.noReadout = true;
  auto r = runScenario(o);
  CHECK(scExpect(r, {Outcome::StoppedNoSource}));
  CHECK(r.elapsedMs == 300000);
}

// N.I.N.A. as the source: TPPA delivers a reading every 5 s, each from an image
// taken 8 s earlier, the moves are slow (200 steps/s) and the firmware adds no
// settle of its own. Skipping one reading after every move is what keeps the
// loop off images taken mid-move: with 0 this run halts calibration-suspect
// after 4 moves with the true error grown to 71.9'.
TEST(SC_TppaStream_ReadingsFromImagesTakenBeforeTheyArrive_Finishes) {
  auto o = scBase();
  o.sim.refreshMs = 5000;
  o.readoutLagMs = 8000;
  o.settings.feed = 200;
  o.settings.settleMs = 0;
  o.settings.readingsToSkipAfterMotion = 1;
  auto r = runScenario(o);
  CHECK(scExpect(r, {Outcome::Finished}));
  CHECK(scNearTolerance(r, o));
}

// 6' of play on both axes of the platform, and the loop's moves compensated as
// the plugin compensates them today (Full, the measured 6'): the compensated
// legs must not disturb the convergence. (Uncompensated, this platform also
// finishes, in 8 moves: the controller learns the response the play leaves.)
TEST(SC_SixArcminBacklash_CompensatedByTheLoop_Finishes) {
  auto o = scBase();
  o.sim.x.backlashArcmin = 6.0;
  o.sim.y.backlashArcmin = 6.0;
  o.settings.backlashX = {motion::BacklashMode::Full, 6.0f, 6.0f};
  o.settings.backlashY = {motion::BacklashMode::Full, 6.0f, 6.0f};
  auto r = runScenario(o);
  CHECK(scExpect(r, {Outcome::Finished}));
  CHECK(scNearTolerance(r, o));
}

// The bench case of 27/09/2026 (firmware 1.3.0, real motors, simulated sky): 120'/-60',
// factors told 12.5 against a true 15, move cap 120', a reading every 4 s showing the axes
// 3 s earlier and the first reading after a move discarded. It took 16 corrections, each one
// axis at half its error. Correcting 75% of the error on both axes per move, 134' falls below
// the 1' tolerance in 4 corrections after the 2 probes; 2 more allow for noise.
TEST(SC_BenchCase_LargeErrorClosesInAFewTwoAxisCorrections) {
  auto o = scBase();
  o.settings.factorX = 12.5;
  o.settings.factorY = 12.5;
  o.settings.userCapArcmin = 120;
  o.settings.readingsToSkipAfterMotion = 1;
  o.sim.x.trueStepsPerArcmin = 15;
  o.sim.y.trueStepsPerArcmin = 15;
  o.sim.initialAzArcmin = 120;
  o.sim.initialAltArcmin = -60;
  o.readoutLagMs = 3000;
  auto r = runScenario(o);
  CHECK(scExpect(r, {Outcome::Finished}));
  CHECK(scNearTolerance(r, o));
  CHECK(r.capViolations == 0);
  std::printf("  bench case: %d corrections, %d axis moves, %u ms\n", r.corrections, r.moves, r.elapsedMs);
  CHECK(r.corrections <= 8);
}
