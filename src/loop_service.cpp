#include "loop_service.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>

#include "asiair/pa_tracker.h"
#include "asiair_client.h"
#include "loop/alignment_loop.h"
#include "loop/calibration.h"
#include "loop/platform_sim.h"
#include "nina_bridge.h"
#include "device_config.h"
#include "device_state.h"
#include "shared_state.h"

namespace loopservice {
namespace {

constexpr uint32_t kSimSourceTimeoutMs = 300000;
constexpr uint32_t kAsiairSourceTimeoutMs = 30000;
constexpr uint32_t kNinaSourceTimeoutMs = 60000;

paloop::AlignmentLoop alignment;
paloop::Calibration calibration;
asiair::PaTracker tracker;
std::unique_ptr<paloop::PlatformSim> sim;
paloop::Observation latest;
bool hasLatest = false;
uint32_t passMaxUs = 0;
char command[48];
devset::LoopConfig pendingConfig;
bool configPending = false;
paloop::Outcome lastOutcome = paloop::Outcome::None;
int lastMoves = 0;
int movesAtLastSample = 0;
bool trackerEndHandled = false;
bool trackerReadingTaken = false;
uint32_t lastTrackerReadingMs = 0;
bool calibrationStarted = false;
long calibrationStartX = 0;
long calibrationStartY = 0;
paloop::CalState lastCalState = paloop::CalState::Idle;
ninabridge::NinaSource nina;
bool ninaStartPending = false;
bool ninaRun = false;  // the current (or last) run is fed by N.I.N.A.
double ninaFactorX = 0;  // 0 until the plugin pushes its calibrated factors
double ninaFactorY = 0;
paloop::AxisBacklash ninaBacklashX;  // Off until the plugin pushes its values
paloop::AxisBacklash ninaBacklashY;
double ninaToleranceArcmin = 0;  // 0 until the plugin pushes TPPA's tolerance
bool ninaFactorsPushed = false;    // the plugin sends $F= only for calibrated factors
bool ninaCalibrateRequested = false;
bool ninaAlignAfterCalibration = true;  // false: $C=2, the readings are not a polar error
bool ninaCalibrating = false;      // the current (or last) calibration is fed by N.I.N.A.
bool ninaCalibrationHandled = true;
long lastStepsX = 0;
long lastStepsY = 0;
bool stepsSeen = false;
uint32_t lastNinaReadingTakenMs = 0;
bool ninaReadingTaken = false;

bool ninaDriving() { return ninaRun && alignment.running(); }

// The play the loop uses after a calibration: the mean of the two directions.
float meanPlay(const paloop::AxisCalibration &r) {
  return static_cast<float>((r.playEnteringPositive + r.playEnteringNegative) / 2.0);
}

motion::BacklashMode recommendedMode(const paloop::AxisCalibration &r) {
  return motion::BacklashPlanner::recommend(meanPlay(r), 0.0f);
}

char modeLetter(motion::BacklashMode mode) {
  switch (mode) {
    case motion::BacklashMode::Soft: return 'S';
    case motion::BacklashMode::Full: return 'F';
    case motion::BacklashMode::Unidirectional: return 'U';
    default: return 'O';
  }
}
bool ninaCalibratingNow() { return ninaCalibrating && calibration.running(); }
bool ninaFeeding() { return ninaDriving() || ninaCalibratingNow(); }

void applyConfig(paloop::LoopSettings &s, const devset::LoopConfig &c) {
  s.factorX = c.factorX;
  s.factorY = c.factorY;
  s.toleranceArcmin = c.toleranceArcmin;
  s.userCapArcmin = c.userCapArcmin;
  s.settleMs = c.settleMs;
  s.feed = c.feed;
}

void copyText(char *dst, size_t size, const char *src) {
  std::strncpy(dst, src, size - 1);
  dst[size - 1] = 0;
}

bool sourceReady() { return sim != nullptr || (tracker.adjusting() && tracker.hasReading()); }

void addSample() {
  device::addHistory(static_cast<float>(latest.azArcmin), static_cast<float>(latest.altArcmin),
                     alignment.movesCommanded() != movesAtLastSample);
  movesAtLastSample = alignment.movesCommanded();
}

void serviceAsiair(uint32_t nowMs) {
  asiair::AsiairEvent event;
  while (asiairclient::popEvent(event)) tracker.onEvent(event, nowMs);
  tracker.tick(nowMs);

  if (tracker.ended() && !trackerEndHandled) {
    trackerEndHandled = true;
    device::logEvent("asiair", "%s", tracker.endReason());
    if (!sim) {
      if (alignment.running() && !ninaDriving()) alignment.stopBySource(tracker.endReason());
      if (calibration.running() && !ninaCalibratingNow()) calibration.stopBySource(tracker.endReason());
    }
  }
  if (!tracker.ended()) trackerEndHandled = false;

  if (sim || ninaFeeding()) return;
  if (!tracker.hasReading()) {
    hasLatest = false;
    return;
  }
  const paloop::Observation &reading = tracker.reading();
  if (trackerReadingTaken && reading.receivedMs == lastTrackerReadingMs) return;
  trackerReadingTaken = true;
  lastTrackerReadingMs = reading.receivedMs;
  latest = reading;
  hasLatest = true;
  addSample();
}

void publish(uint32_t nowMs, long stepsX, long stepsY) {
  LoopStatusSnapshot s;
  copyText(s.phase, sizeof s.phase, paloop::phaseName(alignment.phase()));
  copyText(s.outcome, sizeof s.outcome, paloop::outcomeName(alignment.outcome()));
  copyText(s.reason, sizeof s.reason, alignment.reason().c_str());
  s.simulation = sim != nullptr;
  copyText(s.source, sizeof s.source, sim ? "sim" : ninaFeeding() ? "nina" : (tracker.adjusting() ? "asiair" : "none"));
  s.sourceReady = sourceReady();
  s.hasReading = hasLatest;
  if (hasLatest) {
    s.azArcmin = latest.azArcmin;
    s.altArcmin = latest.altArcmin;
    s.totalArcmin = std::hypot(latest.azArcmin, latest.altArcmin);
  }
  s.moves = alignment.movesCommanded();
  s.planX = alignment.lastPlan().x;
  s.planY = alignment.lastPlan().y;
  s.planProbe = alignment.lastPlan().isProbe;
  s.capArcmin = alignment.lastCapArcmin();
  s.passMaxUs = passMaxUs;
  shared::publishLoop(s);

  CalibrationSnapshot c;
  copyText(c.state, sizeof c.state, paloop::calStateName(calibration.state()));
  c.axis = calibration.axis() == paloop::AxisId::X ? 'X' : 'Y';
  c.probe = calibration.probe();
  c.steps = calibration.probeSteps();
  c.responseArcmin = calibration.responseArcmin();
  copyText(c.reason, sizeof c.reason, calibration.reason().c_str());
  const paloop::AxisCalibration &rx = calibration.result(paloop::AxisId::X);
  const paloop::AxisCalibration &ry = calibration.result(paloop::AxisId::Y);
  c.xValid = rx.valid;
  c.xFactor = rx.stepsPerArcmin;
  c.xSign = rx.sign;
  c.yValid = ry.valid;
  c.yFactor = ry.stepsPerArcmin;
  c.ySign = ry.sign;
  if (calibrationStarted) {
    c.offsetX = stepsX - calibrationStartX;
    c.offsetY = stepsY - calibrationStartY;
  }
  shared::publishCalibration(c);

  AsiairSnapshot a;
  copyText(a.phase, sizeof a.phase, tracker.phase());
  a.adjusting = tracker.adjusting();
  a.ended = tracker.ended();
  copyText(a.endReason, sizeof a.endReason, tracker.endReason());
  a.hasReading = tracker.hasReading();
  if (a.hasReading) {
    a.readingAgeMs = nowMs - tracker.readingArrivedMs();
    a.azArcmin = tracker.reading().azArcmin;
    a.altArcmin = tracker.reading().altArcmin;
  }
  a.rejected = tracker.rejected();
  shared::publishAsiair(a);
}

void logChanges() {
  if (alignment.movesCommanded() > lastMoves) {
    lastMoves = alignment.movesCommanded();
    const paloop::AdjustmentPlan &plan = alignment.lastPlan();
    device::logEvent("move", "%s AZ %.2f' ALT %.2f'", plan.isProbe ? "probe" : "correction", plan.x, plan.y);
  }
  if (alignment.outcome() != lastOutcome) {
    lastOutcome = alignment.outcome();
    if (lastOutcome != paloop::Outcome::None) {
      device::logEvent("loop", "%s: %.56s", paloop::outcomeName(lastOutcome), alignment.reason().c_str());
    }
  }
  paloop::CalState state = calibration.state();
  if (state == lastCalState) return;
  lastCalState = state;
  if (state == paloop::CalState::Done) {
    device::logEvent("calibration", "AZ (X) %.1f (%+d), ALT (Y) %.1f (%+d) steps per arcmin",
                     calibration.result(paloop::AxisId::X).stepsPerArcmin, calibration.result(paloop::AxisId::X).sign,
                     calibration.result(paloop::AxisId::Y).stepsPerArcmin, calibration.result(paloop::AxisId::Y).sign);
  } else if (state == paloop::CalState::Failed || state == paloop::CalState::Stopped) {
    device::logEvent("calibration", "%s: %.60s", paloop::calStateName(state), calibration.reason().c_str());
  }
}

}  // namespace

void ninaReading(double azArcmin, double altArcmin, uint32_t nowMs) {
  if (nina.onReading(azArcmin, altArcmin, nowMs, alignment.running() || calibration.running())) ninaStartPending = true;
}

void ninaFactors(double factorX, double factorY) {
  ninaFactorX = factorX;
  ninaFactorY = factorY;
  ninaFactorsPushed = true;
}

void ninaCalibrateStart(uint32_t nowMs, bool alignAfter) {
  ninaCalibrateRequested = true;
  ninaAlignAfterCalibration = alignAfter;
  // Readings already coming and nothing running: calibrate now, then align.
  if (nina.streaming(nowMs) && !alignment.running() && !calibration.running()) ninaStartPending = true;
}

void ninaCalibrateStop() {
  ninaCalibrateRequested = false;
  if (ninaCalibratingNow()) calibration.stopByUser();
}

size_t ninaCalibrationStatus(char *out, size_t len) {
  ninabridge::CalibrationStatus s;
  s.state = paloop::calStateName(calibration.state());
  const paloop::AxisCalibration &x = calibration.result(paloop::AxisId::X);
  const paloop::AxisCalibration &y = calibration.result(paloop::AxisId::Y);
  s.xValid = x.valid;
  s.xFactor = x.stepsPerArcmin;
  s.xSign = x.sign;
  s.yValid = y.valid;
  s.yFactor = y.stepsPerArcmin;
  s.ySign = y.sign;
  s.xPlayValid = x.backlashValid;
  s.xPlayPositive = x.playEnteringPositive;
  s.xPlayNegative = x.playEnteringNegative;
  s.yPlayValid = y.backlashValid;
  s.yPlayPositive = y.playEnteringPositive;
  s.yPlayNegative = y.playEnteringNegative;
  s.xMode = modeLetter(recommendedMode(x));
  s.yMode = modeLetter(recommendedMode(y));
  s.reason = calibration.reason().c_str();
  return ninabridge::formatCalibration(s, out, len);
}

void ninaBacklash(char axis, char mode, double plusArcmin, double minusArcmin) {
  paloop::AxisBacklash play;
  play.mode = mode == 'S' ? motion::BacklashMode::Soft
              : mode == 'F' ? motion::BacklashMode::Full
              : mode == 'U' ? motion::BacklashMode::Unidirectional
                            : motion::BacklashMode::Off;
  play.plusArcmin = static_cast<float>(plusArcmin);
  play.minusArcmin = static_cast<float>(minusArcmin);
  (axis == 'X' ? ninaBacklashX : ninaBacklashY) = play;
}

void ninaTolerance(double arcmin) { ninaToleranceArcmin = arcmin; }

void ninaStart() { ninaStartPending = true; }

void ninaStop() {
  ninaStartPending = false;
  if (ninaDriving()) alignment.stopByUser();
  if (ninaCalibratingNow()) calibration.stopByUser();
}

void ninaUserMotion() {
  if (!ninaFeeding()) return;
  if (ninaDriving()) alignment.stopByUser();
  if (ninaCalibratingNow()) calibration.stopByUser();
  device::logEvent("loop", "the PC moved the axes: run stopped");
}

size_t ninaStatus(char *out, size_t len) {
  ninabridge::Status s;
  if (ninaCalibratingNow()) {
    s.phase = "calibrating";
    s.source = "nina";
    s.moves = calibration.probe();
    s.hasReading = hasLatest;
    s.azArcmin = latest.azArcmin;
    s.altArcmin = latest.altArcmin;
    s.reason = calibration.reason().c_str();
    return ninabridge::formatStatus(s, out, len);
  }
  s.phase = paloop::phaseName(alignment.phase());
  s.outcome = paloop::outcomeName(alignment.outcome());
  s.source = sim ? "sim" : ninaDriving() || (ninaRun && !alignment.running()) ? "nina" : (tracker.adjusting() ? "asiair" : "none");
  s.moves = alignment.movesCommanded();
  s.hasReading = hasLatest;
  s.azArcmin = latest.azArcmin;
  s.altArcmin = latest.altArcmin;
  s.planX = alignment.lastPlan().x;
  s.planY = alignment.lastPlan().y;
  s.reason = alignment.reason().c_str();
  return ninabridge::formatStatus(s, out, len);
}

void notePassMicros(uint32_t micros) {
  if (micros > passMaxUs) passMaxUs = micros;
}

const char *tick(uint32_t nowMs, bool ninaActive, long stepsX, long stepsY, bool runningX, bool runningY) {
  // Whoever moved an axis (this loop, a jog from N.I.N.A. or from the page), the
  // side of its play it now rests on is the direction it last travelled.
  if (stepsSeen) {
    long dx = stepsX - lastStepsX, dy = stepsY - lastStepsY;
    double fx = alignment.settings().factorX, fy = alignment.settings().factorY;
    if (dx != 0) alignment.setEngagement(paloop::AxisId::X, dx * fx > 0 ? motion::Direction::Positive : motion::Direction::Negative);
    if (dy != 0) alignment.setEngagement(paloop::AxisId::Y, dy * fy > 0 ? motion::Direction::Positive : motion::Direction::Negative);
  }
  lastStepsX = stepsX;
  lastStepsY = stepsY;
  stepsSeen = true;

  devset::LoopConfig changed;
  if (device::takeLoopConfigChange(changed)) {
    pendingConfig = changed;
    configPending = true;
  }
  if (configPending && !sim && !alignment.running()) {
    paloop::LoopSettings s = alignment.settings();
    applyConfig(s, pendingConfig);
    configPending = !alignment.configure(s);
  }

  SimRequest request;
  if (shared::takeSimRequest(request)) {
    alignment.stopByUser();  // reconfiguring the source ends any run and stops motion
    calibration.stopByUser();
    if (request.enable) {
      sim.reset(new paloop::PlatformSim(request.params));
      paloop::LoopSettings settings = alignment.settings();
      applyConfig(settings, device::loopConfig());
      settings.factorX = request.factorX;  // the simulator's factor overrides while it is enabled
      settings.factorY = request.factorY;
      alignment.configure(settings);
      device::logEvent("sim", "simulation enabled, factor %.1f", request.factorX);
    } else {
      sim.reset();
      pendingConfig = device::loopConfig();
      configPending = true;
      device::logEvent("sim", "simulation disabled");
    }
    hasLatest = false;
    trackerReadingTaken = false;
  }
  if (shared::takeLoopStopRequest()) {
    alignment.stopByUser();
    calibration.stopByUser();
  }

  serviceAsiair(nowMs);

  if (shared::takeLoopStartRequest() && sourceReady() && !calibration.running()) {
    paloop::LoopSettings settings = alignment.settings();
    settings.sourceTimeoutMs = sim ? kSimSourceTimeoutMs : kAsiairSourceTimeoutMs;
    alignment.configure(settings);
    passMaxUs = 0;
    if (alignment.start(nowMs, ninaActive)) {
      ninaRun = false;
      lastOutcome = paloop::Outcome::None;
      lastMoves = 0;
      movesAtLastSample = 0;
      device::logEvent("loop", "run started (%s)", sim ? "simulation" : "ASIAIR");
    }
  }
  if (shared::takeCalibrationStartRequest() && sourceReady() && !alignment.running()) {
    devset::LoopConfig config = device::loopConfig();
    paloop::CalibrationSettings settings;
    settings.thresholdArcmin = config.calThresholdArcmin;
    settings.settleMs = config.settleMs;
    settings.feed = config.feed;
    if (calibration.start(settings, nowMs, ninaActive)) {
      ninaCalibrating = false;
      calibrationStarted = true;
      calibrationStartX = stepsX;
      calibrationStartY = stepsY;
      device::logEvent("calibration", "started (%s), threshold %.1f'", sim ? "simulation" : "ASIAIR",
                       settings.thresholdArcmin);
    }
  }

  if (!sim && ninaStartPending && !alignment.running() && !calibration.running() &&
      (!ninaFactorsPushed || ninaCalibrateRequested)) {
    ninaStartPending = false;
    devset::LoopConfig config = device::loopConfig();
    paloop::CalibrationSettings settings;
    settings.thresholdArcmin = config.calThresholdArcmin;
    settings.settleMs = config.settleMs;
    settings.feed = config.feed;
    settings.sourceTimeoutMs = kNinaSourceTimeoutMs;
    settings.readingsToSkipAfterMotion = 1;
    if (calibration.start(settings, nowMs, false)) {
      ninaCalibrating = true;
      ninaCalibrationHandled = false;
      ninaRun = false;
      calibrationStarted = true;
      calibrationStartX = stepsX;
      calibrationStartY = stepsY;
      ninaReadingTaken = false;
      device::logEvent("calibration", "started (PC), threshold %.1f'", settings.thresholdArcmin);
    }
  }
  if (!sim && ninaStartPending) {
    ninaStartPending = false;
    if (!alignment.running() && !calibration.running()) {
      paloop::LoopSettings settings = alignment.settings();
      applyConfig(settings, device::loopConfig());
      if (ninaFactorX != 0 && ninaFactorY != 0) {
        settings.factorX = ninaFactorX;
        settings.factorY = ninaFactorY;
      }
      settings.sourceTimeoutMs = kNinaSourceTimeoutMs;
      settings.readingsToSkipAfterMotion = 1;
      settings.backlashX = ninaBacklashX;
      settings.backlashY = ninaBacklashY;
      if (ninaToleranceArcmin > 0) settings.toleranceArcmin = ninaToleranceArcmin;
      alignment.configure(settings);
      passMaxUs = 0;
      if (alignment.start(nowMs, false)) {
        ninaRun = true;
        lastOutcome = paloop::Outcome::None;
        lastMoves = 0;
        movesAtLastSample = 0;
        ninaReadingTaken = false;
        device::logEvent("loop", "run started (PC), factors %.2f / %.2f, tolerance %.2f'", settings.factorX,
                         settings.factorY, settings.toleranceArcmin);
      }
    }
  }
  if (!sim && ninaFeeding() && nina.hasReading() &&
      (!ninaReadingTaken || nina.latest().receivedMs != lastNinaReadingTakenMs)) {
    ninaReadingTaken = true;
    lastNinaReadingTakenMs = nina.latest().receivedMs;
    latest = nina.latest();
    hasLatest = true;
    addSample();
  }

  if (sim) {
    sim->track(stepsX, stepsY);
    if (sim->due(nowMs)) {
      latest = sim->read(nowMs);
      hasLatest = true;
      addSample();
    }
  }

  paloop::TickInput in;
  in.nowMs = nowMs;
  in.observation = hasLatest ? &latest : nullptr;
  in.axisIdle[0] = !runningX;
  in.axisIdle[1] = !runningY;
  // N.I.N.A. is the source of a run it feeds, not a competitor for the motors.
  in.ninaActive = ninaFeeding() ? false : ninaActive;
  paloop::LoopAction calibrationAction = calibration.tick(in);
  paloop::LoopAction action = alignment.tick(in);
  if (calibrationAction.kind != paloop::ActionKind::None) action = calibrationAction;
  if (ninaCalibrating && !ninaCalibrationHandled && !calibration.running()) {
    ninaCalibrationHandled = true;
    bool alignAfter = ninaAlignAfterCalibration;
    ninaAlignAfterCalibration = true;  // whatever the outcome, the next request says again
    if (calibration.state() == paloop::CalState::Done) {
      ninaFactorX = calibration.result(paloop::AxisId::X).stepsPerArcmin;
      ninaFactorY = calibration.result(paloop::AxisId::Y).stepsPerArcmin;
      ninaFactorsPushed = true;
      ninaCalibrateRequested = false;
      // The play goes to the loop as the mean of the two directions: a split the
      // mechanism does not have biases every reversal (valo_20260919: the split
      // pair planned a -0.29' request to arrive at +4.27'; the mean converged).
      for (paloop::AxisId axis : {paloop::AxisId::X, paloop::AxisId::Y}) {
        const paloop::AxisCalibration &r = calibration.result(axis);
        if (!r.backlashValid) continue;
        float mean = meanPlay(r);
        paloop::AxisBacklash play;
        play.mode = recommendedMode(r);
        play.plusArcmin = mean;
        play.minusArcmin = mean;
        (axis == paloop::AxisId::X ? ninaBacklashX : ninaBacklashY) = play;
      }
      // Align with the factors just measured, on the next pass - unless the readings were
      // the plugin's own field displacements, which no alignment can use.
      if (alignAfter) ninaStartPending = true;
    }
  }
  publish(nowMs, stepsX, stepsY);
  logChanges();

  if (action.kind == paloop::ActionKind::Move) {
    std::snprintf(command, sizeof command, "$J=G91G21%c%ldF%d", action.axis == paloop::AxisId::X ? 'X' : 'Y',
                  action.steps, action.feed);
    return command;
  }
  if (action.kind == paloop::ActionKind::Stop) return "!";
  return nullptr;
}

}  // namespace loopservice
