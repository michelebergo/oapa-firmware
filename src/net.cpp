#include "net.h"

#include <Arduino.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WiFi.h>
#include <atomic>
#include <esp_system.h>

#include "device_state.h"
#include "net_rules.h"

namespace net {
namespace {

const char *const kSetupSsid = "OAPA-setup";
const char *const kHostname = "oapa";
constexpr uint32_t kTaskPeriodMs = 50;
constexpr uint32_t kRestartDelayMs = 1500;

Preferences prefs;
DNSServer dns;
String ssid, password;
std::atomic<netrules::Mode> mode{netrules::Mode::Connecting};
uint32_t modeSinceMs = 0, lastRetryMs = 0, restartAtMs = 0;
std::atomic<bool> restartPending{false};
bool mdnsStarted = false;
uint32_t boots = 0;

// Modem sleep wakes the radio only every DTIM beacon: measured 328 ms median
// and up to 4.5 s per request, which a STOP button cannot afford.
void keepRadioAwake() { WiFi.setSleep(false); }

void enterSetup(uint32_t now) {
  WiFi.mode(WIFI_AP_STA);
  keepRadioAwake();
  WiFi.softAP(kSetupSsid);
  dns.start(53, "*", WiFi.softAPIP());
  mode = netrules::Mode::Setup;
  modeSinceMs = lastRetryMs = now;
  device::logEvent("wifi", "setup mode: OAPA-setup");
}

void enterStation(uint32_t now) {
  dns.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  if (!mdnsStarted && MDNS.begin(kHostname)) {
    MDNS.addService("http", "tcp", 80);
    mdnsStarted = true;
  }
  mode = netrules::Mode::Station;
  modeSinceMs = now;
  device::logEvent("wifi", "joined %s as %s", ssid.c_str(), WiFi.localIP().toString().c_str());
}

void apply(netrules::Action action, uint32_t now) {
  switch (action) {
    case netrules::Action::EnterSetup: enterSetup(now); break;
    case netrules::Action::EnterStation: enterStation(now); break;
    case netrules::Action::RetryStation:
      WiFi.begin(ssid.c_str(), password.c_str());
      lastRetryMs = now;
      break;
    case netrules::Action::BackToConnecting:
      mode = netrules::Mode::Connecting;  // WiFi auto-reconnect keeps trying
      device::logEvent("wifi", "link lost, reconnecting");
      modeSinceMs = now;
      break;
    case netrules::Action::None: break;
  }
}

void netTask(void *) {
  for (;;) {
    uint32_t now = millis();
    if (mode == netrules::Mode::Setup) dns.processNextRequest();
    netrules::Inputs in{ssid.length() > 0, WiFi.status() == WL_CONNECTED, now,
                        modeSinceMs, lastRetryMs, WiFi.softAPgetStationNum()};
    apply(netrules::decide(mode, in), now);
    if (restartPending && (uint32_t)(now - restartAtMs) >= kRestartDelayMs) ESP.restart();
    vTaskDelay(pdMS_TO_TICKS(kTaskPeriodMs));
  }
}

}  // namespace

void begin() {
  prefs.begin("oapa", false);
  boots = prefs.getUInt("boots", 0) + 1;
  prefs.putUInt("boots", boots);
  ssid = prefs.getString("ssid", "");
  password = prefs.getString("pass", "");

  WiFi.persistent(false);  // credentials live in our own NVS namespace
  WiFi.setAutoReconnect(true);
  // The first WiFi mode is chosen here, synchronously, for two reasons:
  // - the web server opens its socket right after begin() returns; on a TCP/IP
  //   stack that was never started lwIP asserts (tcpip_api_call: Invalid mbox)
  //   and the board reboot-loops;
  // - switching STA -> AP_STA from the net task while the STA start was still
  //   in flight left softAP() reporting success with no beacon on the air.
  uint32_t now = millis();
  if (ssid.length() > 0) {
    WiFi.mode(WIFI_STA);
    keepRadioAwake();
    WiFi.begin(ssid.c_str(), password.c_str());
    modeSinceMs = now;
  } else {
    enterSetup(now);
  }
  // Core 0 with the WiFi stack; loop() and the motors stay alone on core 1.
  xTaskCreatePinnedToCore(netTask, "net", 4096, nullptr, 1, nullptr, 0);
}

bool stationConnected() { return WiFi.status() == WL_CONNECTED; }
bool inSetupMode() { return mode == netrules::Mode::Setup; }
int rssi() { return stationConnected() ? WiFi.RSSI() : 0; }
uint32_t bootCount() { return boots; }

const char *resetReasonName() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_SW: return "SW";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_EXT: return "EXT";
    default: return "OTHER";
  }
}

bool saveCredentials(const char *newSsid, const char *newPassword) {
  if (!netrules::validCredentials(newSsid, newPassword)) return false;
  prefs.putString("ssid", newSsid);
  prefs.putString("pass", newPassword);
  return true;
}

void requestRestart() {
  restartAtMs = millis();
  restartPending = true;
}

void clearCredentials() {
  prefs.remove("ssid");
  prefs.remove("pass");
}

}  // namespace net
