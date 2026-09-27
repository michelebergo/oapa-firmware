#include "pa_tracker.h"

#include <cstring>

namespace asiair {
namespace {

void copyText(char *dst, size_t size, const char *src) {
  std::strncpy(dst, src, size - 1);
  dst[size - 1] = 0;
}

}  // namespace

void PaTracker::onEvent(const AsiairEvent &e, uint32_t nowMs) {
  if (e.kind == EventKind::PaExposure) {
    if (std::strcmp(e.state, "start") == 0) {
      exposureStartMs_ = nowMs;
      haveExposure_ = true;
    }
    return;
  }
  if (e.kind == EventKind::PaSolve) {
    // An aborted solve (code 253) also happens in the middle of a live PA: on
    // 16/09/2026 ASIAIR sent two of them and was back in calc4 nine seconds later.
    // Only silence tells us the PA is over.
    return;
  }
  if (e.kind != EventKind::Pa3ppa) return;

  last3ppaMs_ = nowMs;
  copyText(phase_, sizeof phase_, e.state);
  bool calc = std::strcmp(e.state, "calc3") == 0 || std::strcmp(e.state, "calc4") == 0;
  if (!calc || !e.hasError) {
    if (std::strcmp(e.state, "start") == 0) {
      ended_ = false;
      endReason_[0] = 0;
      hasReading_ = false;
      haveExposure_ = false;
    }
    adjusting_ = false;
    return;
  }

  adjusting_ = true;
  ended_ = false;
  endReason_[0] = 0;
  if (e.retryCnt > 1) {  // the frame follows a failed solve: seen jumping 0.3 deg untouched
    rejected_++;
    return;
  }
  reading_.azArcmin = e.xDeg * 60.0;
  reading_.altArcmin = e.yDeg * 60.0;
  reading_.receivedMs = haveExposure_ ? exposureStartMs_ : nowMs;
  readingArrivedMs_ = nowMs;
  hasReading_ = true;
}

void PaTracker::tick(uint32_t nowMs) {
  if (adjusting_ && static_cast<uint32_t>(nowMs - last3ppaMs_) >= kSilenceEndMs) {
    end("no PA result from ASIAIR for 30 s");
  }
}

void PaTracker::end(const char *why) {
  if (ended_) return;
  ended_ = true;
  adjusting_ = false;
  hasReading_ = false;
  copyText(endReason_, sizeof endReason_, why);
}

bool linkStale(uint32_t lastDataMs, uint32_t nowMs) {
  return static_cast<uint32_t>(nowMs - lastDataMs) >= kLinkSilenceMs;
}

}  // namespace asiair
