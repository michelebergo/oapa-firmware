// Connection to ASIAIR's event port 4700 on core 0 (AsyncTCP). Lines are parsed
// where they arrive and the PA events are queued for the loop service on core 1.
// Host: the saved address, or the WiFi gateway (ASIAIR itself on its hotspot).
#pragma once
#include <cstdint>

#include "asiair/asiair_parser.h"

namespace asiairclient {

struct Status {
  bool connected = false;
  char host[16] = "";
  uint32_t lines = 0;
  uint32_t badLines = 0;
};

void begin();                             // setup(), after webserver::begin()
bool popEvent(asiair::AsiairEvent &out);  // core 1
Status status();                          // any core
void reconnect();                         // after the saved address changes

}  // namespace asiairclient
