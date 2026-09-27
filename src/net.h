// WiFi on core 0: saved station network with an OAPA-setup access point
// fallback, captive DNS in setup mode, mDNS "oapa" in station mode.
#pragma once
#include <cstdint>

namespace net {

void begin();  // setup(): loads NVS, counts the boot, starts the net task
bool stationConnected();
bool inSetupMode();
int rssi();
uint32_t bootCount();
const char *resetReasonName();
bool saveCredentials(const char *ssid, const char *password);  // validates, writes NVS
void requestRestart();  // performed by the net task ~1.5 s later
void clearCredentials();  // next boot starts OAPA-setup

}  // namespace net
