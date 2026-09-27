// Port of nina.plugin.polaralignment ConvergenceMonitorTest.cs, case by case.
#pragma once
using paloop::ConvergenceAction;
using paloop::ConvergenceMonitor;

static ConvergenceMonitor cmNew() { return ConvergenceMonitor(0.5); }
static ConvergenceMonitor cmNew(double jitterArcmin) {
  return ConvergenceMonitor(0.5, [jitterArcmin]() { return std::optional<double>(jitterArcmin); });
}

TEST(CM_TwoConsecutiveBelowTolerance_Finishes) {
  auto m = cmNew();
  CHECK(m.observe(0.42, 0.3, true).action == ConvergenceAction::AwaitConfirmation);
  CHECK(m.observe(0.40, 0, false).action == ConvergenceAction::Finish);
}

TEST(CM_ConfirmationSurvivesReadingWithinAbsoluteMargin) {
  auto m = cmNew();
  CHECK(m.observe(0.42, 0.3, true).action == ConvergenceAction::AwaitConfirmation);
  CHECK(m.observe(0.58, 0, false).action == ConvergenceAction::AwaitConfirmation);
  CHECK(m.observe(0.45, 0, false).action == ConvergenceAction::Finish);
}

TEST(CM_ConfirmationResetsAboveMargin) {
  auto m = cmNew();
  CHECK(m.observe(0.42, 0.3, true).action == ConvergenceAction::AwaitConfirmation);
  CHECK(m.observe(0.64, 0, false).action == ConvergenceAction::Continue);
  CHECK(m.minimumAchievedArcmin().has_value() && *m.minimumAchievedArcmin() == 0.42);
}

TEST(CM_StationaryDrift_SetsDegraded) {
  auto m = cmNew();
  m.observe(3.0, 0, false, true);
  m.observe(3.4, 0, false);
  CHECK(m.estimateDegraded());
}

TEST(CM_MovementExplainsChange_NoDegradation) {
  auto m = cmNew();
  m.observe(3.0, 2.0, true, true);
  m.observe(1.2, 2.0, true);
  CHECK(!m.estimateDegraded());
}

TEST(CM_WorseningStreakWithLargeMoves_HaltsAsCalibrationSuspect) {
  auto m = cmNew();
  m.observe(2.0, 1.5, true, true);
  CHECK(m.observe(2.2, 1.5, true).action == ConvergenceAction::Continue);
  CHECK(m.observe(2.5, 1.5, true).action == ConvergenceAction::Continue);
  CHECK(m.observe(2.9, 1.5, true).action == ConvergenceAction::HaltCalibrationSuspect);
}

TEST(CM_WorseningStreakWithSubNoiseMoves_HaltsAsEstimateDrift) {
  auto m = cmNew();
  m.observe(0.62, 0.23, true, true);
  CHECK(m.observe(0.85, 0.23, true).action == ConvergenceAction::Continue);
  CHECK(m.observe(1.10, 0.9, true).action == ConvergenceAction::Continue);
  CHECK(m.observe(1.41, 0.9, true).action == ConvergenceAction::HaltEstimateDrift);
}

TEST(CM_ImprovementResetsWorseningStreak) {
  auto m = cmNew();
  m.observe(2.0, 1.5, true, true);
  m.observe(2.2, 1.5, true);
  m.observe(2.5, 1.5, true);
  CHECK(m.observe(1.8, 1.5, true).action == ConvergenceAction::Continue);
  CHECK(m.observe(2.0, 1.5, true).action == ConvergenceAction::Continue);
}

TEST(CM_OscillationAroundAchievedMinimum_FinishesBestEffort) {
  auto m = cmNew();
  CHECK(m.observe(0.45, 0.3, true).action == ConvergenceAction::AwaitConfirmation);
  m.observe(0.75, 0.2, true);
  m.observe(0.70, 0.2, true);
  m.observe(0.72, 0.2, true);
  auto last = m.observe(0.74, 0.2, true);
  CHECK(last.action == ConvergenceAction::FinishBestEffort);
  CHECK(*m.minimumAchievedArcmin() == 0.45);
}

TEST(CM_DegradedWithAchievedMinimum_FinishesBestEffortImmediately) {
  auto m = cmNew();
  CHECK(m.observe(0.45, 0.3, true).action == ConvergenceAction::AwaitConfirmation);
  CHECK(m.observe(0.80, 0, false).action == ConvergenceAction::FinishBestEffort);
}

TEST(CM_WithinMarginReadingsDoNotCountAsWorsenings) {
  auto m = cmNew();
  m.observe(0.42, 0.3, true);
  m.observe(0.55, 0, false);
  m.observe(0.58, 0, false);
  CHECK(m.observe(0.60, 0, false).action != ConvergenceAction::HaltEstimateDrift);
}

TEST(CM_DegradedWithMinimum_PrefersBestEffortOverHalt) {
  auto m = cmNew();
  CHECK(m.observe(0.42, 0.3, true).action == ConvergenceAction::AwaitConfirmation);
  CHECK(m.observe(0.68, 0, false).action == ConvergenceAction::FinishBestEffort);
}

TEST(CM_FirstObservationBelowTolerance_CountsAsConfirmation) {
  auto m = cmNew();
  CHECK(m.observe(0.35, 0, false, true).action == ConvergenceAction::AwaitConfirmation);
  CHECK(m.observe(0.40, 0, false).action == ConvergenceAction::Finish);
}

TEST(CM_BelowTolerance_WithResidualLargerThanTheNoise_CorrectsOnceMore) {
  auto m = cmNew(0.05);
  CHECK(m.observe(0.42, 0.3, true).action == ConvergenceAction::Continue);
}

TEST(CM_BelowTolerance_WithResidualInsideTheNoise_StopsAsBefore) {
  auto m = cmNew(0.05);
  CHECK(m.observe(0.10, 0.3, true).action == ConvergenceAction::AwaitConfirmation);
}

TEST(CM_WithoutAJitterMeasurement_TheOldBehaviourIsUnchanged) {
  auto m = cmNew();
  CHECK(m.observe(0.42, 0.3, true).action == ConvergenceAction::AwaitConfirmation);
}

TEST(CM_TheExtraCorrectionCannotDelayTheFinish) {
  auto m = cmNew(0.05);
  CHECK(m.observe(0.42, 0.3, true).action == ConvergenceAction::Continue);
  CHECK(m.observe(0.20, 0.3, true).action == ConvergenceAction::Finish);
}

TEST(CM_AnExtraCorrectionThatMakesThingsWorse_LeavesTheOscillationGuardIntact) {
  auto m = cmNew(0.05);
  CHECK(m.observe(0.42, 0.3, true).action == ConvergenceAction::Continue);
  for (int i = 0; i < ConvergenceMonitor::kBestEffortOscillations - 1; i++) {
    CHECK(m.observe(1.2, 0.3, true).action == ConvergenceAction::Continue);
  }
  CHECK(m.observe(1.2, 0.3, true).action == ConvergenceAction::FinishBestEffort);
}

TEST(CM_ADegradedEstimate_NeverEarnsAnExtraCorrection) {
  auto m = cmNew(0.05);
  CHECK(m.observe(0.42, 0.3, true).action == ConvergenceAction::Continue);
  m.observe(0.90, 0, false);
  CHECK(m.estimateDegraded());
  CHECK(m.observe(0.42, 0, true).action != ConvergenceAction::Continue);
}

TEST(CM_FirstObservationAboveTolerance_Continues) {
  auto m = cmNew();
  CHECK(m.observe(2.0, 0, false, true).action == ConvergenceAction::Continue);
}

TEST(CM_ReasonStrings_MatchThePlugin) {
  auto m = cmNew();
  CHECK_EQ_STR(m.observe(2.0, 0, false, true).reason, "First observation.");
  auto m2 = cmNew();
  CHECK_EQ_STR(m2.observe(0.35, 0, false, true).reason,
               "Below tolerance (0.35') on the first observation, awaiting confirmation solve (1/2).");
  CHECK_EQ_STR(m2.observe(0.40, 0, false).reason, "Total error 0.40' below tolerance for 2 consecutive solves.");
}
