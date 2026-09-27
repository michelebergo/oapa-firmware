#include "shared_state.h"

#include <Arduino.h>
#include <atomic>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "web_bridge.h"

namespace shared {
namespace {

constexpr UBaseType_t kQueueDepth = 8;
QueueHandle_t commandQueue = nullptr;
std::atomic<bool> stopRequested{false};
portMUX_TYPE snapshotMux = portMUX_INITIALIZER_UNLOCKED;
StatusSnapshot snapshot;
std::atomic<bool> loopStartRequested{false};
std::atomic<bool> loopStopRequested{false};
portMUX_TYPE simMux = portMUX_INITIALIZER_UNLOCKED;
SimRequest pendingSim;
bool simPending = false;
LoopStatusSnapshot loopStatus;
std::atomic<bool> calibrationStartRequested{false};
CalibrationSnapshot calibrationStatus;
AsiairSnapshot asiairStatus;

}  // namespace

void begin() { commandQueue = xQueueCreate(kQueueDepth, web::kCommandLen); }

bool enqueueCommand(const char *cmd) {
  char entry[web::kCommandLen] = {0};
  std::strncpy(entry, cmd, sizeof entry - 1);
  return xQueueSend(commandQueue, entry, 0) == pdTRUE;
}

bool popCommand(char *out) { return xQueueReceive(commandQueue, out, 0) == pdTRUE; }

void requestStop() { stopRequested.store(true); }

bool takeStopRequest() { return stopRequested.exchange(false); }

void publish(const StatusSnapshot &s) {
  portENTER_CRITICAL(&snapshotMux);
  snapshot = s;
  portEXIT_CRITICAL(&snapshotMux);
}

StatusSnapshot read() {
  portENTER_CRITICAL(&snapshotMux);
  StatusSnapshot copy = snapshot;
  portEXIT_CRITICAL(&snapshotMux);
  return copy;
}

void requestLoopStart() { loopStartRequested.store(true); }
bool takeLoopStartRequest() { return loopStartRequested.exchange(false); }
void requestLoopStop() { loopStopRequested.store(true); }
bool takeLoopStopRequest() { return loopStopRequested.exchange(false); }

void requestSim(const SimRequest &request) {
  portENTER_CRITICAL(&simMux);
  pendingSim = request;
  simPending = true;
  portEXIT_CRITICAL(&simMux);
}

bool takeSimRequest(SimRequest &out) {
  portENTER_CRITICAL(&simMux);
  bool had = simPending;
  if (had) {
    out = pendingSim;
    simPending = false;
  }
  portEXIT_CRITICAL(&simMux);
  return had;
}

void publishLoop(const LoopStatusSnapshot &s) {
  portENTER_CRITICAL(&snapshotMux);
  loopStatus = s;
  portEXIT_CRITICAL(&snapshotMux);
}

LoopStatusSnapshot readLoop() {
  portENTER_CRITICAL(&snapshotMux);
  LoopStatusSnapshot copy = loopStatus;
  portEXIT_CRITICAL(&snapshotMux);
  return copy;
}

void requestCalibrationStart() { calibrationStartRequested.store(true); }
bool takeCalibrationStartRequest() { return calibrationStartRequested.exchange(false); }

void publishCalibration(const CalibrationSnapshot &s) {
  portENTER_CRITICAL(&snapshotMux);
  calibrationStatus = s;
  portEXIT_CRITICAL(&snapshotMux);
}

CalibrationSnapshot readCalibration() {
  portENTER_CRITICAL(&snapshotMux);
  CalibrationSnapshot copy = calibrationStatus;
  portEXIT_CRITICAL(&snapshotMux);
  return copy;
}

void publishAsiair(const AsiairSnapshot &s) {
  portENTER_CRITICAL(&snapshotMux);
  asiairStatus = s;
  portEXIT_CRITICAL(&snapshotMux);
}

AsiairSnapshot readAsiair() {
  portENTER_CRITICAL(&snapshotMux);
  AsiairSnapshot copy = asiairStatus;
  portEXIT_CRITICAL(&snapshotMux);
  return copy;
}

}  // namespace shared
