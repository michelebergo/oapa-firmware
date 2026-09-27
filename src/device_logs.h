// What the page's event log and error chart are made of. Pure, host-tested.
#pragma once
#include <cstdint>
#include <cstring>

#include "ring_buffer.h"

struct EventEntry {
  uint32_t uptimeMs = 0;
  char code[16] = "";
  char text[81] = "";
};

struct HistorySample {
  uint32_t uptimeMs = 0;
  float azArcmin = 0;
  float altArcmin = 0;
  bool moved = false;  // a move was commanded since the previous sample
};

using EventLog = SeqRing<EventEntry, 100>;
using HistoryLog = SeqRing<HistorySample, 300>;

inline EventEntry makeEvent(uint32_t uptimeMs, const char *code, const char *text) {
  EventEntry entry;
  entry.uptimeMs = uptimeMs;
  std::strncpy(entry.code, code, sizeof entry.code - 1);
  std::strncpy(entry.text, text, sizeof entry.text - 1);
  return entry;
}
