// State shared between core 0 (WiFi, web) and core 1 (loop: motors, serial).
// Core 0 never touches a stepper: it queues commands and reads snapshots.
#pragma once
#include "status_json.h"
#include "loop/loop_status_json.h"
#include "loop/platform_sim.h"
#include "loop/calibration_status.h"
#include "asiair/asiair_status.h"

// A request from the web to (re)configure or disable the simulated error source.
struct SimRequest {
  bool enable = false;
  paloop::SimParams params;
  double factorX = 60;
  double factorY = 60;
};

namespace shared {

void begin();                          // call in setup() before any web task
bool enqueueCommand(const char *cmd);  // core 0; false when the queue is full
bool popCommand(char *out);            // core 1; out has web::kCommandLen bytes
void requestStop();                    // core 0; STOP never waits in the queue
bool takeStopRequest();                // core 1
void publish(const StatusSnapshot &s); // core 1
StatusSnapshot read();                 // core 0

void requestLoopStart();                        // core 0
bool takeLoopStartRequest();                    // core 1
void requestLoopStop();                         // core 0
bool takeLoopStopRequest();                     // core 1
void requestSim(const SimRequest &request);     // core 0
bool takeSimRequest(SimRequest &out);           // core 1
void publishLoop(const LoopStatusSnapshot &s);  // core 1
LoopStatusSnapshot readLoop();                  // core 0

void requestCalibrationStart();                         // core 0
bool takeCalibrationStartRequest();                     // core 1
void publishCalibration(const CalibrationSnapshot &s);  // core 1
CalibrationSnapshot readCalibration();                  // core 0
void publishAsiair(const AsiairSnapshot &s);            // core 1
AsiairSnapshot readAsiair();                            // core 0

}  // namespace shared
