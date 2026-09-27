// Closes the loop on fake time: motors advance at the commanded feed, the
// simulator tracks them every tick and publishes a readout every refresh period.
#pragma once
#include <cmath>
#include <deque>
#include <optional>
#include <string>
#include <utility>

struct ScenarioOptions {
  paloop::SimParams sim;
  paloop::LoopSettings settings;
  uint32_t maxMs = 30u * 60u * 1000u;
  uint32_t tickMs = 50;
  bool motorsDead = false;  // commanded moves never progress
  bool noReadout = false;   // the error source never delivers
  std::optional<uint32_t> ninaActiveFromMs;
  // Each readout shows where the axes were this long ago: TPPA reports an
  // image that was exposed and solved before the reading arrives. Assumes no
  // simulated backlash (the lagged track would disturb its direction memory).
  uint32_t readoutLagMs = 0;
};

struct ScenarioResult {
  paloop::Outcome outcome = paloop::Outcome::None;
  std::string reason;
  double finalTrueErrorArcmin = 0;
  int moves = 0;
  int capViolations = 0;
  paloop::ActionKind lastActionKind = paloop::ActionKind::None;
  uint32_t elapsedMs = 0;
};

inline ScenarioResult runScenario(const ScenarioOptions &opt) {
  paloop::PlatformSim sim(opt.sim);
  paloop::AlignmentLoop alignment(opt.settings);
  double pos[2] = {0, 0};
  long target[2] = {0, 0};
  paloop::Observation latest;
  bool hasLatest = false;
  ScenarioResult r;
  r.elapsedMs = opt.maxMs;
  std::deque<std::pair<uint32_t, std::pair<long, long>>> history;  // (time, axis steps)

  alignment.start(0, false);
  for (uint32_t now = 0; now <= opt.maxMs; now += opt.tickMs) {
    if (!opt.motorsDead) {
      double stride = opt.settings.feed * opt.tickMs / 1000.0;
      for (int i = 0; i < 2; i++) {
        double d = target[i] - pos[i];
        pos[i] = std::fabs(d) <= stride ? static_cast<double>(target[i]) : pos[i] + (d > 0 ? stride : -stride);
      }
    }
    long nowX = std::lround(pos[0]), nowY = std::lround(pos[1]);
    history.push_back({now, {nowX, nowY}});
    while (history.size() > 1 && now - history[1].first >= opt.readoutLagMs) history.pop_front();
    if (!opt.noReadout && sim.due(now)) {
      sim.track(history.front().second.first, history.front().second.second);
      latest = sim.read(now);
      hasLatest = true;
    }
    sim.track(nowX, nowY);

    paloop::TickInput in;
    in.nowMs = now;
    in.observation = hasLatest ? &latest : nullptr;
    in.axisIdle[0] = std::lround(pos[0]) == target[0];
    in.axisIdle[1] = std::lround(pos[1]) == target[1];
    in.ninaActive = opt.ninaActiveFromMs.has_value() && now >= *opt.ninaActiveFromMs;

    paloop::LoopAction a = alignment.tick(in);
    if (a.kind != paloop::ActionKind::None) r.lastActionKind = a.kind;
    if (a.kind == paloop::ActionKind::Move) {
      int i = a.axis == paloop::AxisId::X ? 0 : 1;
      target[i] += a.steps;
      double factor = i == 0 ? opt.settings.factorX : opt.settings.factorY;
      if (std::fabs(a.steps / factor) > alignment.lastCapArcmin() + 1.0 / factor) r.capViolations++;
      r.moves++;
    } else if (a.kind == paloop::ActionKind::Stop) {
      target[0] = std::lround(pos[0]);
      target[1] = std::lround(pos[1]);
    }

    if (!alignment.running()) {
      r.elapsedMs = now;
      break;
    }
  }

  double az = 0, alt = 0;
  sim.trueError(az, alt);
  r.finalTrueErrorArcmin = std::hypot(az, alt);
  r.outcome = alignment.outcome();
  r.reason = alignment.reason();
  return r;
}
