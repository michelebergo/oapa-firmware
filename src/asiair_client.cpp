#include "asiair_client.h"

#include <Arduino.h>
#include <AsyncTCP.h>
#include <WiFi.h>
#include <atomic>
#include <cstdio>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "asiair/pa_tracker.h"
#include "device_state.h"
#include "net.h"

namespace asiairclient {
namespace {

constexpr uint16_t kPort = 4700;
constexpr size_t kLineMax = 2048;
constexpr UBaseType_t kQueueDepth = 16;
constexpr uint32_t kRetryMs = 2000;
enum : int { kIdle, kConnecting, kConnected };

std::atomic<int> linkState{kIdle};
QueueHandle_t events = nullptr;
AsyncClient *client = nullptr;
char line[kLineMax];
size_t lineLen = 0;
bool overflow = false;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
Status current;
uint32_t lastDataMs = 0;

void countLine(bool bad) {
  portENTER_CRITICAL(&mux);
  current.lines++;
  if (bad) current.badLines++;
  portEXIT_CRITICAL(&mux);
}

void onData(void *, AsyncClient *, void *data, size_t len) {
  lastDataMs = millis();
  const char *bytes = static_cast<const char *>(data);
  for (size_t i = 0; i < len; i++) {
    char c = bytes[i];
    if (c != '\n') {
      if (overflow) continue;
      if (lineLen < kLineMax - 1) line[lineLen++] = c;
      else overflow = true;  // longer than any ASIAIR line seen: dropped whole
      continue;
    }
    if (overflow) {
      countLine(true);
    } else if (lineLen) {
      line[lineLen] = 0;
      asiair::AsiairEvent e;
      bool ok = asiair::parseLine(line, e);
      countLine(!ok);
      if (ok && e.kind != asiair::EventKind::Other) xQueueSend(events, &e, 0);
    }
    lineLen = 0;
    overflow = false;
  }
}

void setConnected(bool connected) {
  int previous = linkState.exchange(connected ? kConnected : kIdle);
  char host[16];
  portENTER_CRITICAL(&mux);
  current.connected = connected;
  std::snprintf(host, sizeof host, "%s", current.host);
  portEXIT_CRITICAL(&mux);
  if (connected && previous != kConnected) device::logEvent("asiair", "connected to %s", host);
  if (!connected && previous == kConnected) device::logEvent("asiair", "connection to %s lost", host);
}

bool resolveHost(IPAddress &ip, char *text, size_t len) {
  device::asiairHost(text, len);
  if (text[0]) return ip.fromString(text);
  ip = WiFi.gatewayIP();
  if (ip == IPAddress(0, 0, 0, 0)) return false;
  std::snprintf(text, len, "%s", ip.toString().c_str());
  return true;
}

void connectTask(void *) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(kRetryMs));
    if (linkState.load() == kConnected && asiair::linkStale(lastDataMs, millis())) {
      device::logEvent("asiair", "no data for %lu s, reconnecting",
                       static_cast<unsigned long>(asiair::kLinkSilenceMs / 1000));
      client->close();  // ASIAIR stops feeding an open socket without closing it
      continue;
    }
    if (linkState.load() != kIdle || !net::stationConnected()) continue;
    IPAddress ip;
    char text[16];
    if (!resolveHost(ip, text, sizeof text)) continue;
    portENTER_CRITICAL(&mux);
    std::snprintf(current.host, sizeof current.host, "%s", text);
    portEXIT_CRITICAL(&mux);
    linkState.store(kConnecting);
    if (!client->connect(ip, kPort)) linkState.store(kIdle);
  }
}

}  // namespace

void begin() {
  events = xQueueCreate(kQueueDepth, sizeof(asiair::AsiairEvent));
  client = new AsyncClient();
  client->onConnect(
      [](void *, AsyncClient *) {
        lineLen = 0;
        overflow = false;
        lastDataMs = millis();
        setConnected(true);
      },
      nullptr);
  client->onDisconnect([](void *, AsyncClient *) { setConnected(false); }, nullptr);
  client->onError([](void *, AsyncClient *, int8_t) { setConnected(false); }, nullptr);
  client->onData(onData, nullptr);
  xTaskCreatePinnedToCore(connectTask, "asiair", 4096, nullptr, 1, nullptr, 0);
}

bool popEvent(asiair::AsiairEvent &out) { return events && xQueueReceive(events, &out, 0) == pdTRUE; }

Status status() {
  portENTER_CRITICAL(&mux);
  Status copy = current;
  portEXIT_CRITICAL(&mux);
  return copy;
}

void reconnect() {
  if (client && linkState.load() == kConnected) client->close();
}

}  // namespace asiairclient
