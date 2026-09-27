// What the loop service publishes for /api/loop, and its JSON form. Fixed-size
// fields so it can be copied between cores under a spinlock.
#pragma once
#include <cstddef>
#include <cstdint>

struct LoopStatusSnapshot {
  char phase[16] = "idle";
  char outcome[32] = "none";
  char reason[240] = "";
  bool simulation = false;
  char source[8] = "none";  // none, sim, asiair
  bool sourceReady = false;
  bool hasReading = false;
  double azArcmin = 0;
  double altArcmin = 0;
  double totalArcmin = 0;
  int moves = 0;
  double planX = 0;
  double planY = 0;
  bool planProbe = false;
  double capArcmin = 0;
  uint32_t passMaxUs = 0;
};

int formatLoopJson(const LoopStatusSnapshot &s, char *buf, size_t len);
