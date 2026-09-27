// The alignment loop on core 1. main.cpp ticks it from serviceWebSide() with
// the steppers' state and executes the returned command through
// runWebCommand(), the same path as a web jog. Simulation mode is off at boot.
#pragma once
#include <cstddef>
#include <cstdint>

namespace loopservice {

const char *tick(uint32_t nowMs, bool ninaActive, long stepsX, long stepsY, bool runningX, bool runningY);
void notePassMicros(uint32_t micros);

// N.I.N.A. bridge (see nina_bridge.h). Called from the serial handler, which
// runs on the same loop() pass as tick(), so no locking is needed.
void ninaReading(double azArcmin, double altArcmin, uint32_t nowMs);
void ninaFactors(double factorX, double factorY);
void ninaBacklash(char axis, char mode, double plusArcmin, double minusArcmin);
void ninaTolerance(double arcmin);
void ninaMoveCap(double arcmin);
// Calibrate before the next alignment N.I.N.A. feeds (or now, if its readings are
// already coming). Without factors from the plugin the board calibrates anyway.
void ninaCalibrateStart(uint32_t nowMs, bool alignAfter = true);
void ninaCalibrateStop();
size_t ninaCalibrationStatus(char *out, size_t len);
void ninaStart();
void ninaStop();
void ninaUserMotion();  // a jog or STOP from N.I.N.A. ends a run N.I.N.A. is feeding
size_t ninaStatus(char *out, size_t len);

}  // namespace loopservice
