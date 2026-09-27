#include "net_rules.h"

#include <cstring>

namespace netrules {

Action decide(Mode mode, const Inputs &in) {
  if (!in.hasCredentials) return mode == Mode::Setup ? Action::None : Action::EnterSetup;
  switch (mode) {
    case Mode::Connecting:
      if (in.connected) return Action::EnterStation;
      return (uint32_t)(in.nowMs - in.modeSinceMs) >= kConnectTimeoutMs ? Action::EnterSetup : Action::None;
    case Mode::Station:
      return in.connected ? Action::None : Action::BackToConnecting;
    case Mode::Setup:
      if (in.connected) return Action::EnterStation;
      // A retry hops the radio channel and drops a phone using the setup page.
      if (in.apClients == 0 && (uint32_t)(in.nowMs - in.lastRetryMs) >= kRetryEveryMs) return Action::RetryStation;
      return Action::None;
  }
  return Action::None;
}

bool validCredentials(const char *ssid, const char *password) {
  if (ssid == nullptr || password == nullptr) return false;
  size_t s = std::strlen(ssid), p = std::strlen(password);
  return s >= 1 && s <= 32 && (p == 0 || (p >= 8 && p <= 63));
}

}  // namespace netrules
