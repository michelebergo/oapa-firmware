#include "web_bridge.h"

#include <cctype>
#include <cstdio>
#include <cstring>

namespace web {

bool isAllowedCommand(const char *cmd) {
  if (cmd == nullptr) return false;
  size_t len = std::strlen(cmd);
  if (len == 0 || len >= kCommandLen) return false;
  for (size_t i = 0; i < len; ++i) {
    if ((unsigned char)cmd[i] < 0x20) return false;
  }
  if (std::strncmp(cmd, "$J=", 3) == 0) return true;
  if (std::strcmp(cmd, "!") == 0) return true;
  char first = cmd[0];
  bool axis = first == 'X' || first == 'x' || first == 'Y' || first == 'y';
  return axis && (std::isdigit((unsigned char)cmd[1]) || cmd[1] == '-');
}

bool isDriverCommand(const char *cmd) {
  if (cmd == nullptr) return false;
  size_t len = std::strlen(cmd);
  if (len < 3 || len > 7) return false;
  if (cmd[0] != 'C' && cmd[0] != 'H' && cmd[0] != 'S') return false;
  if (cmd[1] != 'X' && cmd[1] != 'Y') return false;
  for (size_t i = 2; i < len; ++i) {
    if (!std::isdigit(static_cast<unsigned char>(cmd[i]))) return false;
  }
  return true;
}

bool buildJogCommand(char axis, long steps, int feed, char *out, size_t outLen) {
  if (axis != 'X' && axis != 'Y') return false;
  if (steps == 0 || steps > kMaxJogSteps || steps < -kMaxJogSteps) return false;
  if (feed < kMinFeed || feed > kMaxFeed) return false;
  int n = std::snprintf(out, outLen, "$J=G91G21%c%ldF%d", axis, steps, feed);
  return n > 0 && (size_t)n < outLen;
}

bool ninaActive(bool anySerialByte, uint32_t nowMs, uint32_t lastSerialMs) {
  return anySerialByte && (uint32_t)(nowMs - lastSerialMs) < kNinaWindowMs;
}

}  // namespace web
