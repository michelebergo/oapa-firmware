// Pure decisions for the WiFi state machine, host-tested. The ESP side in
// net.cpp only applies the returned action.
#pragma once
#include <cstdint>

namespace netrules {

constexpr uint32_t kConnectTimeoutMs = 30000;
constexpr uint32_t kRetryEveryMs = 60000;

enum class Mode { Connecting, Station, Setup };
enum class Action { None, EnterStation, EnterSetup, RetryStation, BackToConnecting };

struct Inputs {
  bool hasCredentials;
  bool connected;       // station link up
  uint32_t nowMs;
  uint32_t modeSinceMs; // when the current mode was entered
  uint32_t lastRetryMs; // last station retry while in Setup
  int apClients;        // phones attached to OAPA-setup
};

Action decide(Mode mode, const Inputs &in);

// SSID 1..32 bytes; password empty (open network) or 8..63 bytes (WPA2).
bool validCredentials(const char *ssid, const char *password);

}  // namespace netrules
