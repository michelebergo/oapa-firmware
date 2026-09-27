// What /api/asiair reports: link state (core 0) merged with the PA tracker (core 1).
#pragma once
#include <cstddef>
#include <cstdint>

struct AsiairSnapshot {
  bool connected = false;
  char host[16] = "";
  char phase[16] = "";
  bool adjusting = false;
  bool ended = false;
  char endReason[48] = "";
  bool hasReading = false;
  uint32_t readingAgeMs = 0;
  double azArcmin = 0;
  double altArcmin = 0;
  uint32_t lines = 0;
  uint32_t badLines = 0;
  uint32_t rejected = 0;
};

int formatAsiairJson(const AsiairSnapshot &s, char *buf, size_t len);
