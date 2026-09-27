// ASIAIR's polar alignment as seen from its event stream (block C spec 3.1):
// which readings the alignment may use, when each frame was exposed, and when
// the PA ended. Pure, fed with the device clock.
#pragma once
#include <cstdint>

#include "../loop/observation.h"
#include "asiair_parser.h"

namespace asiair {

// ASIAIR can stop feeding an open socket without closing it (field 16/09/2026: the
// board sat on a dead connection for 23 minutes). No byte for this long means reconnect.
constexpr uint32_t kLinkSilenceMs = 30000;
bool linkStale(uint32_t lastDataMs, uint32_t nowMs);

class PaTracker {
 public:
  static constexpr uint32_t kSilenceEndMs = 30000;

  void onEvent(const AsiairEvent &e, uint32_t nowMs);
  void tick(uint32_t nowMs);

  const char *phase() const { return phase_; }
  bool adjusting() const { return adjusting_; }
  bool hasReading() const { return hasReading_; }
  const paloop::Observation &reading() const { return reading_; }  // receivedMs = exposure start
  uint32_t readingArrivedMs() const { return readingArrivedMs_; }
  bool ended() const { return ended_; }
  const char *endReason() const { return endReason_; }
  uint32_t rejected() const { return rejected_; }

 private:
  void end(const char *why);

  char phase_[16] = "";
  char endReason_[48] = "";
  bool adjusting_ = false;
  bool hasReading_ = false;
  bool ended_ = false;
  bool haveExposure_ = false;
  paloop::Observation reading_;
  uint32_t readingArrivedMs_ = 0;
  uint32_t exposureStartMs_ = 0;
  uint32_t last3ppaMs_ = 0;
  uint32_t rejected_ = 0;
};

}  // namespace asiair
