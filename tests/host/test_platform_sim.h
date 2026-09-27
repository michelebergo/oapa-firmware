// The simulator is itself test equipment: each property the scenarios rely on is pinned here.
#pragma once
#include <cmath>
using paloop::PlatformSim;
using paloop::SimParams;

static SimParams simQuiet() {
  SimParams p;
  p.noiseSigmaArcmin = 0;
  return p;
}

TEST(SIM_NoMotion_ReadsTheInitialError) {
  PlatformSim sim(simQuiet());
  sim.track(0, 0);
  auto o = sim.read(0);
  CHECK_NEAR(o.azArcmin, 30.0, 1e-12);
  CHECK_NEAR(o.altArcmin, -20.0, 1e-12);
  CHECK(o.receivedMs == 0);
}

TEST(SIM_AxisMotion_UsesTrueFactorAndSign) {
  SimParams p = simQuiet();
  p.x.trueStepsPerArcmin = 60;
  p.y.sign = -1;
  PlatformSim sim(p);
  sim.track(0, 0);
  sim.track(120, 60);  // X +2', Y +1' commanded, Y wired reversed
  double az, alt;
  sim.trueError(az, alt);
  CHECK_NEAR(az, 32.0, 1e-12);
  CHECK_NEAR(alt, -21.0, 1e-12);
}

TEST(SIM_Coupling_MixesTheAxes) {
  SimParams p = simQuiet();
  p.coupling[0][1] = 0.3;  // Y motion also moves azimuth
  PlatformSim sim(p);
  sim.track(0, 0);
  sim.track(0, 600);  // Y +10'
  double az, alt;
  sim.trueError(az, alt);
  CHECK_NEAR(az, 33.0, 1e-12);
  CHECK_NEAR(alt, -10.0, 1e-12);
}

TEST(SIM_Backlash_IsADeadBandWithDirectionMemory) {
  SimParams p = simQuiet();
  p.x.backlashArcmin = 2.0;  // 1' of free play each way from the engaged centre
  PlatformSim sim(p);
  double az, alt;
  sim.track(0, 0);
  sim.track(60, 0);   // +1': inside the band, output does not move
  sim.trueError(az, alt);
  CHECK_NEAR(az, 30.0, 1e-12);
  sim.track(120, 0);  // +2': output follows at input - 1'
  sim.trueError(az, alt);
  CHECK_NEAR(az, 31.0, 1e-12);
  sim.track(0, 0);    // back to 0: still inside the band on the other side
  sim.trueError(az, alt);
  CHECK_NEAR(az, 31.0, 1e-12);
  sim.track(-60, 0);  // -1': output follows at input + 1'
  sim.trueError(az, alt);
  CHECK_NEAR(az, 30.0, 1e-12);
}

TEST(SIM_Refresh_IsDueImmediatelyThenEveryPeriod) {
  PlatformSim sim(simQuiet());
  sim.track(0, 0);
  CHECK(sim.due(0));
  sim.read(0);
  CHECK(!sim.due(3999));
  CHECK(sim.due(4000));
}

TEST(SIM_Drift_AccruesWithDeviceTime) {
  SimParams p = simQuiet();
  p.driftAzArcminPerMin = 0.6;
  p.driftAltArcminPerMin = -0.3;
  PlatformSim sim(p);
  sim.track(0, 0);
  auto o = sim.read(120000);
  CHECK_NEAR(o.azArcmin, 30 + 1.2, 1e-9);
  CHECK_NEAR(o.altArcmin, -20 - 0.6, 1e-9);
}

TEST(SIM_Noise_IsSeededAndHasTheRequestedSigma) {
  SimParams p;
  p.noiseSigmaArcmin = 0.5;
  p.seed = 42;
  PlatformSim a(p), b(p);
  a.track(0, 0);
  b.track(0, 0);
  double sum = 0, sumSq = 0;
  const int n = 4000;
  for (int i = 0; i < n; i++) {
    auto oa = a.read(i * 4000u);
    auto ob = b.read(i * 4000u);
    CHECK(oa.azArcmin == ob.azArcmin);
    double e = oa.azArcmin - 30.0;
    sum += e;
    sumSq += e * e;
  }
  double mean = sum / n;
  double sd = std::sqrt(sumSq / n - mean * mean);
  // Standard error of the sample sd is about sigma / sqrt(2n) = 1.1 %; 10 % is far outside chance.
  CHECK(std::fabs(sd - 0.5) < 0.05);
  CHECK(std::fabs(mean) < 0.05);
}
