// What /api/calibrate reports. Fixed-size so it can be copied between cores.
#pragma once
#include <cstddef>
#include <cstdint>

struct CalibrationSnapshot {
  char state[12] = "idle";
  char axis = 'X';
  int probe = 0;
  long steps = 0;
  double responseArcmin = 0;
  char reason[96] = "";
  bool xValid = false;
  double xFactor = 0;
  int xSign = 0;
  bool yValid = false;
  double yFactor = 0;
  int ySign = 0;
  long offsetX = 0;  // steps away from where the last calibration started
  long offsetY = 0;
};

int formatCalibrationJson(const CalibrationSnapshot &s, char *buf, size_t len);
