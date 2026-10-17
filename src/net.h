// WiFi on core 0: saved station network with an OAPA-setup access point
// fallback, captive DNS in setup mode, and mDNS in station mode under a name
// taken from the MAC, "oapa-a1b2c3.local", unique per board.
#pragma once
#include <cstdint>

namespace net {

void begin();  // setup(): loads NVS, counts the boot, starts the net task
bool stationConnected();
const char *hostname();  // "oapa-a1b2c3", also the mDNS name with .local
bool inSetupMode();
int rssi();
uint32_t bootCount();
const char *resetReasonName();
bool saveCredentials(const char *ssid, const char *password);  // validates, writes NVS
void requestRestart();  // performed by the net task ~1.5 s later
void clearCredentials();  // next boot starts OAPA-setup

}  // namespace net
