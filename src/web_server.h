// HTTP routes on core 0. Handlers never touch a stepper: jogs and STOP go
// through shared_state, status comes from the published snapshot.
#pragma once

namespace webserver {
// board: which build this is (fysetc_e4 / esp32_stepdir), so the page fetches the right update.
void begin(const char *fwVersion, const char *board);
}
