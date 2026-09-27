// Port of FieldConvergenceReplayTest.cs: the total errors of one tester's last
// alignment on 19 August 2026, read from the N.I.N.A. log, replayed through the
// decision machine. See the C# file for what this replay is and is not.
#pragma once
#include <optional>

struct FrReplayResult { ConvergenceAction last; double lastError; int corrections; };

static const double kFrTheNight[] = {
    302.20, 301.75, 278.45, 296.27, 243.90, 201.85, 152.35, 110.53, 80.55, 60.37, 43.77, 32.40, 22.62,
    18.37,  12.77,  9.98,   7.30,   5.50,   3.98,   2.63,   1.93,   1.55,  1.00,  0.77,  0.58,  0.62,
    0.47,   0.47,   5.12,   5.12,   5.62,   5.28,   3.68,   3.68,   1.65,  0.58,  0.58,  0.55,  0.60,
    0.57,   1.30,   1.27,   1.18,   0.80,   0.55,   0.52,   0.52,   0.52,  0.50,  0.35,  0.35};

static FrReplayResult frReplay(std::optional<double> jitterArcmin) {
  ConvergenceMonitor monitor(0.5, [jitterArcmin]() { return jitterArcmin; });
  ConvergenceAction action = ConvergenceAction::Continue;
  int corrections = 0;
  double last = 0;
  const int n = static_cast<int>(sizeof kFrTheNight / sizeof kFrTheNight[0]);
  for (int i = 0; i < n; i++) {
    last = kFrTheNight[i];
    auto decision = monitor.observe(last, 1.0, action == ConvergenceAction::Continue, i == 0);
    action = decision.action;
    if (action == ConvergenceAction::Continue) corrections++;
    if (action == ConvergenceAction::Finish || action == ConvergenceAction::FinishBestEffort) break;
  }
  return {action, last, corrections};
}

TEST(FR_TheNightAsItHappened_AndTheSameNightWithAMeasuredFloor) {
  CHECK(frReplay(std::nullopt).last == ConvergenceAction::Finish);
  CHECK(frReplay(0.08).last == ConvergenceAction::Finish);
}

TEST(FR_AFloorBelowTheResidual_SpendsTheWaitCorrecting) {
  CHECK(frReplay(0.08).corrections - frReplay(std::nullopt).corrections == 1);
}

TEST(FR_AFloorAboveTheResidual_ChangesNothing) {
  CHECK(frReplay(0.20).corrections == frReplay(std::nullopt).corrections);
}
