// HTTP routes on core 0. Handlers never touch a stepper: jogs and STOP go
// through shared_state, status comes from the published snapshot.
#pragma once

namespace webserver {
void begin(const char *fwVersion);
}
