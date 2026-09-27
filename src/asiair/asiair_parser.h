// One line of ASIAIR's event stream (port 4700) reduced to the fields the
// alignment needs. No JSON library: each line is one object and only a handful
// of keys matter. Pure, host-tested against lines from the night capture.
#pragma once

namespace asiair {

enum class EventKind { Other, Pa3ppa, PaExposure, PaSolve };

struct AsiairEvent {
  EventKind kind = EventKind::Other;
  char state[16] = "";
  int stateCode = -1;
  bool hasError = false;
  double xDeg = 0;  // azimuth error, degrees
  double yDeg = 0;  // altitude error, degrees
  int retryCnt = 0;
  int code = 0;  // PlateSolve failure code (251 solve failed, 253 aborted), 0 when absent
};

// false when the line is not a complete object with an "Event" name.
bool parseLine(const char *line, AsiairEvent &out);

}  // namespace asiair
