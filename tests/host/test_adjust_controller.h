// Port of nina.plugin.polaralignment AutomatedAdjustmentControllerTest.cs: the
// hardware is a local linear response, the controller issues X/Y commands and
// the next "solve" feeds the resulting error back.
#pragma once
#include <cmath>
using paloop::AdjustController;
using paloop::AdjustmentPlan;

struct AcClosedLoopResult { double finalErrorDegrees; int iterations; };

static AcClosedLoopResult acRunClosedLoop(AdjustController &controller, const double plant[2][2],
                                          double initialAz, double initialAlt, int maxIterations) {
  double az = initialAz, alt = initialAlt;
  controller.updateObservation(az, alt);
  for (int iteration = 0; iteration < maxIterations; iteration++) {
    AdjustmentPlan plan = controller.createPlan();
    if (!plan.hasMovement()) break;
    az += plant[0][0] * plan.x + plant[0][1] * plan.y;
    alt += plant[1][0] * plan.x + plant[1][1] * plan.y;
    controller.noteSuccessfulExecution(plan);
    controller.updateObservation(az, alt);
    double total = std::sqrt(az * az + alt * alt);
    if (total < 0.02) return {total, iteration + 1};
  }
  return {std::sqrt(az * az + alt * alt), maxIterations};
}

TEST(AC_LearnsReversedAzimuthAxisAndConverges) {
  AdjustController controller;
  const double plant[2][2] = {{+0.08, 0.00}, {0.00, -0.07}};
  auto r = acRunClosedLoop(controller, plant, 0.4, -0.25, 12);
  CHECK(r.finalErrorDegrees < 0.03);
  CHECK(r.iterations < 12);
  CHECK(controller.hasResponseModel());
}

static void acPolarityCase(double azPerX, double altPerY) {
  AdjustController controller;
  const double plant[2][2] = {{azPerX, 0.00}, {0.00, altPerY}};
  auto r = acRunClosedLoop(controller, plant, 0.4, -0.25, 12);
  CHECK(r.finalErrorDegrees < 0.03);
  CHECK(r.iterations < 12);
  CHECK(controller.hasResponseModel());
}
TEST(AC_LearnsAxisPolarity_MinusMinus) { acPolarityCase(-0.08, -0.07); }
TEST(AC_LearnsAxisPolarity_MinusPlus) { acPolarityCase(-0.08, 0.07); }
TEST(AC_LearnsAxisPolarity_PlusMinus) { acPolarityCase(0.08, -0.07); }
TEST(AC_LearnsAxisPolarity_PlusPlus) { acPolarityCase(0.08, 0.07); }

TEST(AC_LearnsPoorCalibrationAndAxisCrossCoupling) {
  AdjustController controller;
  const double plant[2][2] = {{-0.18, -0.03}, {+0.04, -0.11}};
  auto r = acRunClosedLoop(controller, plant, 0.9, 0.6, 12);
  CHECK(r.finalErrorDegrees < 0.04);
  CHECK(r.iterations < 12);
  CHECK(controller.sampleCount() >= 3);
}

TEST(AC_DoesNotLearnFromFailedMove) {
  AdjustController controller;
  controller.updateObservation(0.5, -0.3);
  AdjustmentPlan first = controller.createPlan();
  CHECK(first.hasMovement() && first.isProbe && first.x != 0);
  controller.noteFailedExecution();
  controller.updateObservation(0.5, -0.3);
  CHECK(controller.sampleCount() == 0);
  AdjustmentPlan second = controller.createPlan();
  CHECK(second.hasMovement() && second.isProbe && second.x != 0);
}

TEST(AC_ThreeConsecutiveFailedMoves_MarkTheHardwareUnresponsive) {
  AdjustController controller;
  controller.noteFailedExecution();
  controller.noteFailedExecution();
  CHECK(!controller.executionUnresponsive());
  controller.noteFailedExecution();
  CHECK(controller.executionUnresponsive());
  controller.resetExecutionFailureStreak();
  CHECK(!controller.executionUnresponsive());
}

TEST(AC_ASuccessfulMove_ResetsTheFailureStreak) {
  AdjustController controller;
  controller.noteFailedExecution();
  controller.noteFailedExecution();
  controller.noteSuccessfulExecution(AdjustmentPlan{1.0, 0.0, false, "test"});
  controller.noteFailedExecution();
  CHECK(!controller.executionUnresponsive());
}

TEST(AC_AggressiveProbe_ScalesWithErrorAndNamesTheAxis) {
  // Not in the C# suite: pins the OAPA profile the loop uses. 36.06' of error,
  // cap 28.84' -> probe = max(1, min(0.15 * 36.06, 28.84 / 2)) = 5.41 on X first.
  AdjustController controller;
  controller.aggressiveCorrections = true;
  controller.setMaximumMoveMagnitude(28.84);
  controller.updateObservation(30.0 / 60.0, -20.0 / 60.0);
  AdjustmentPlan plan = controller.createPlan();
  CHECK(plan.isProbe);
  CHECK_NEAR(plan.x, 0.15 * std::sqrt(30.0 * 30.0 + 20.0 * 20.0), 1e-9);
  CHECK(plan.y == 0);
  CHECK_EQ_STR(plan.reason, "Probing azimuth response");
}
