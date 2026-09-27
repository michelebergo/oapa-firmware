// Block C spec 5: each factor within the fraction its mechanism allows.
#pragma once
#include <cstdio>
#include <cstring>
using paloop::CalState;

static CalOptions calBase() {
  CalOptions o;
  o.sim.x.trueStepsPerArcmin = 180;
  o.sim.y.trueStepsPerArcmin = 180;
  return o;
}

static bool calNear(double got, double want, double fraction) {
  bool ok = std::fabs(got - want) <= fraction * want;
  if (!ok) std::printf("  factor %.2f, want %.2f within %.0f %%\n", got, want, fraction * 100);
  return ok;
}

TEST(CAL_FindsBothFactors_AndReturnsToTheStart) {
  CalRun r = runCalibration(calBase());
  CHECK(r.state == CalState::Done);
  CHECK(r.x.valid && r.y.valid);
  CHECK(calNear(r.x.stepsPerArcmin, 180, 0.05));
  CHECK(calNear(r.y.stepsPerArcmin, 180, 0.05));
  CHECK(r.x.sign == 1 && r.y.sign == 1);
  CHECK(r.finalPos[0] == 0 && r.finalPos[1] == 0);
}

TEST(CAL_ReversedMotor_GivesTheMagnitudeAndSignMinusOne) {
  CalOptions o = calBase();
  o.sim.y.sign = -1;
  CalRun r = runCalibration(o);
  CHECK(r.state == CalState::Done);
  CHECK(calNear(r.y.stepsPerArcmin, 180, 0.05));
  CHECK(r.y.sign == -1);
}

TEST(CAL_TwoArcminBacklash_Within15Percent) {
  CalOptions o = calBase();
  o.sim.x.backlashArcmin = 2.0;
  o.sim.y.backlashArcmin = 2.0;
  CalRun r = runCalibration(o);
  CHECK(r.state == CalState::Done);
  CHECK(calNear(r.x.stepsPerArcmin, 180, 0.15));
  CHECK(calNear(r.y.stepsPerArcmin, 180, 0.15));
}

TEST(CAL_NightDriftWithThresholdFive_Within15Percent) {
  CalOptions o = calBase();
  o.sim.driftAzArcminPerMin = 0.6;
  o.sim.driftAltArcminPerMin = 0.6;
  o.settings.thresholdArcmin = 5.0;
  CalRun r = runCalibration(o);
  CHECK(r.state == CalState::Done);
  CHECK(calNear(r.x.stepsPerArcmin, 180, 0.15));
  CHECK(calNear(r.y.stepsPerArcmin, 180, 0.15));
}

// True factor 250: probes reach 0.8', 2.4', 5.6' after 200, 600, 1400 steps.
TEST(CAL_TheThresholdDecidesWhereProbingStops) {
  CalOptions o;
  o.sim.x.trueStepsPerArcmin = 250;
  o.sim.y.trueStepsPerArcmin = 250;
  o.settings.thresholdArcmin = 1.0;
  CHECK(runCalibration(o).x.steps == 600);
  o.settings.thresholdArcmin = 5.0;
  CHECK(runCalibration(o).x.steps == 1400);
}

TEST(CAL_DeadAxis_ReportsNoResponse_AndRestores) {
  CalOptions o = calBase();
  o.sim.x.trueStepsPerArcmin = 1e9;
  CalRun r = runCalibration(o);
  CHECK(r.state == CalState::Failed);
  CHECK(r.reason.find("axis AZ (X) does not respond") != std::string::npos);
  CHECK(!r.x.valid && !r.y.valid);
  CHECK(r.maxAbsPos[0] == 25400);  // probes 200..12800: the take-up is the probing itself
  CHECK(r.finalPos[0] == 0);
}

// Baseline readings at 0 s and 4 s, first probe (200 steps) moves 4.0-4.2 s.
TEST(CAL_StopByUser_StopsWithoutRestoring) {
  CalOptions o = calBase();
  o.stopAtMs = 4100;
  CalRun r = runCalibration(o);
  CHECK(r.state == CalState::Stopped);
  CHECK(r.stopIssued);
  CHECK(r.movesAfterEnd == 0);
  CHECK(r.finalPos[0] > 0 && r.finalPos[0] < 200);
}

TEST(CAL_Nina_StopsWithoutRestoring) {
  CalOptions o = calBase();
  o.ninaFromMs = 8100;
  CalRun r = runCalibration(o);
  CHECK(r.state == CalState::Stopped);
  CHECK_EQ_STR(r.reason, "A PC took control of OAPA");
  CHECK(r.movesAfterEnd == 0);
}

TEST(CAL_PaEnded_StopsWithoutRestoring) {
  CalOptions o = calBase();
  o.sourceEndAtMs = 8100;
  CalRun r = runCalibration(o);
  CHECK(r.state == CalState::Stopped);
  CHECK_EQ_STR(r.reason, "PA stopped on ASIAIR");
  CHECK(r.movesAfterEnd == 0);
}

TEST(CAL_NoReadings_Fails) {
  CalOptions o = calBase();
  o.sim.refreshMs = 600000;
  CalRun r = runCalibration(o);
  CHECK(r.state == CalState::Failed);
  CHECK_EQ_STR(r.reason, "No error measurement received");
}

TEST(CAL_StartIsRefusedWhileNinaIsActive) {
  paloop::Calibration c;
  CHECK(!c.start(paloop::CalibrationSettings(), 0, true));
  CHECK(!c.running());
}

// TPPA as the source: it exposes the next image right after publishing a
// reading, so each reading shows the axes one period earlier (5 s here); slow
// moves, no extra settle. The first reading after a move can come from an image
// taken mid-move, and discarding it keeps the probes measured on images taken
// after them: with 0 the same run reports 224.30 and 215.94 steps per arcmin
// for a true 180.
static CalOptions calTppa(int skip) {
  CalOptions o = calBase();
  o.sim.refreshMs = 5000;
  o.readoutLagMs = 5000;
  o.settings.feed = 200;
  o.settings.settleMs = 0;
  o.settings.readingsToSkipAfterMotion = skip;
  return o;
}

TEST(CAL_TppaStream_SkippingOneReadingAfterEachMove_FindsBothFactors) {
  CalRun r = runCalibration(calTppa(1));
  CHECK(r.state == CalState::Done);
  CHECK(calNear(r.x.stepsPerArcmin, 180, 0.05));
  CHECK(calNear(r.y.stepsPerArcmin, 180, 0.05));
  CHECK(r.finalPos[0] == 0 && r.finalPos[1] == 0);
}

// The play: after the factor, the axis reverses and comes back, each time until the
// error answers by the threshold; the travel the error did not show is the play of
// that direction. The simulator's band loses its whole width on every reversal.
TEST(CAL_SixArcminPlay_IsMeasuredInBothDirections_AndTheFactorStaysClean) {
  CalOptions o = calBase();
  o.sim.x.backlashArcmin = 6.0;
  o.sim.y.backlashArcmin = 6.0;
  CalRun r = runCalibration(o);
  CHECK(r.state == CalState::Done);
  CHECK(calNear(r.x.stepsPerArcmin, 180, 0.05));
  CHECK(calNear(r.y.stepsPerArcmin, 180, 0.05));
  CHECK(r.x.backlashValid && r.y.backlashValid);
  CHECK_NEAR(r.x.playEnteringNegative, 6.0, 0.6);
  CHECK_NEAR(r.x.playEnteringPositive, 6.0, 0.6);
  CHECK_NEAR(r.y.playEnteringNegative, 6.0, 0.6);
  CHECK_NEAR(r.y.playEnteringPositive, 6.0, 0.6);
  CHECK(r.finalPos[0] == 0 && r.finalPos[1] == 0);
}

TEST(CAL_NoPlay_MeasuresNone) {
  CalRun r = runCalibration(calBase());
  CHECK(r.state == CalState::Done);
  CHECK(r.x.backlashValid && r.y.backlashValid);
  CHECK_NEAR(r.x.playEnteringNegative, 0.0, 0.3);
  CHECK_NEAR(r.x.playEnteringPositive, 0.0, 0.3);
}

TEST(CAL_PlayOverTheTppaStream_WithTheSkip) {
  CalOptions o = calTppa(1);
  o.sim.x.backlashArcmin = 0;  // the lagged readout assumes no band (see the runner)
  CalRun r = runCalibration(o);
  CHECK(r.state == CalState::Done);
  CHECK(r.x.backlashValid);
  CHECK_NEAR(r.x.playEnteringNegative, 0.0, 0.3);
}
