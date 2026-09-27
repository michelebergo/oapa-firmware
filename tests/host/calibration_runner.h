// Runs one calibration on fake time: motors advance at the feed, the simulator
// tracks them every tick and publishes a readout every refresh period. Ticks
// continue 5 s after the end so a move issued after it is counted.
#pragma once
#include <cmath>
#include <cstdlib>
#include <deque>
#include <optional>
#include <string>
#include <utility>

struct CalOptions {
  paloop::SimParams sim;
  paloop::CalibrationSettings settings;
  uint32_t maxMs = 20u * 60u * 1000u;
  uint32_t tickMs = 50;
  std::optional<uint32_t> stopAtMs;
  std::optional<uint32_t> ninaFromMs;
  std::optional<uint32_t> sourceEndAtMs;
  uint32_t readoutLagMs = 0;  // each readout shows the axes this long ago (TPPA); no simulated backlash
};

struct CalRun {
  paloop::CalState state = paloop::CalState::Idle;
  std::string reason;
  paloop::AxisCalibration x, y;
  long finalPos[2] = {0, 0};
  long maxAbsPos[2] = {0, 0};
  int movesAfterEnd = 0;
  bool stopIssued = false;
};

inline CalRun runCalibration(const CalOptions &opt) {
  paloop::PlatformSim sim(opt.sim);
  paloop::Calibration cal;
  double pos[2] = {0, 0};
  long target[2] = {0, 0};
  paloop::Observation latest;
  bool hasLatest = false;
  bool ended = false;
  uint32_t endedAt = 0;
  CalRun r;
  std::deque<std::pair<uint32_t, std::pair<long, long>>> history;  // (time, axis steps)

  cal.start(opt.settings, 0, false);
  for (uint32_t now = 0; now <= opt.maxMs; now += opt.tickMs) {
    double stride = opt.settings.feed * opt.tickMs / 1000.0;
    for (int i = 0; i < 2; i++) {
      double d = target[i] - pos[i];
      pos[i] = std::fabs(d) <= stride ? static_cast<double>(target[i]) : pos[i] + (d > 0 ? stride : -stride);
      long p = std::lround(pos[i]);
      if (std::labs(p) > r.maxAbsPos[i]) r.maxAbsPos[i] = std::labs(p);
    }
    long nowX = std::lround(pos[0]), nowY = std::lround(pos[1]);
    history.push_back({now, {nowX, nowY}});
    while (history.size() > 1 && now - history[1].first >= opt.readoutLagMs) history.pop_front();
    if (sim.due(now)) {
      sim.track(history.front().second.first, history.front().second.second);
      latest = sim.read(now);
      hasLatest = true;
    }
    sim.track(nowX, nowY);
    if (opt.stopAtMs && now == *opt.stopAtMs) cal.stopByUser();
    if (opt.sourceEndAtMs && now == *opt.sourceEndAtMs) cal.stopBySource("PA stopped on ASIAIR");

    paloop::TickInput in;
    in.nowMs = now;
    in.observation = hasLatest ? &latest : nullptr;
    in.axisIdle[0] = std::lround(pos[0]) == target[0];
    in.axisIdle[1] = std::lround(pos[1]) == target[1];
    in.ninaActive = opt.ninaFromMs.has_value() && now >= *opt.ninaFromMs;

    paloop::LoopAction a = cal.tick(in);
    if (a.kind == paloop::ActionKind::Move) {
      if (ended) r.movesAfterEnd++;
      target[a.axis == paloop::AxisId::X ? 0 : 1] += a.steps;
    } else if (a.kind == paloop::ActionKind::Stop) {
      r.stopIssued = true;
      target[0] = std::lround(pos[0]);
      target[1] = std::lround(pos[1]);
    }
    if (!cal.running() && !ended) {
      ended = true;
      endedAt = now;
    }
    if (ended && now >= endedAt + 5000) break;
  }

  r.state = cal.state();
  r.reason = cal.reason();
  r.x = cal.result(paloop::AxisId::X);
  r.y = cal.result(paloop::AxisId::Y);
  r.finalPos[0] = std::lround(pos[0]);
  r.finalPos[1] = std::lround(pos[1]);
  return r;
}
