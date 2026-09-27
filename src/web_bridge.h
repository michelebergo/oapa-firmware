// Rules for commands that arrive from the web page. Arduino-free so the host
// tests exercise exactly this code.
#pragma once
#include <cstddef>
#include <cstdint>

namespace web {

constexpr long kMaxJogSteps = 200000;
constexpr int kMinFeed = 50;     // firmware JOG_SPEED_MIN
constexpr int kMaxFeed = 3000;   // firmware JOG_SPEED_MAX
constexpr int kDefaultFeed = 1000;
constexpr uint32_t kNinaWindowMs = 10000;
constexpr size_t kCommandLen = 48;

// True only for commands whose handlers never write to Serial: $J= jogs,
// bare X<n>/Y<n> moves and "!". "?" and "$H" print directly and would put
// unexpected lines in front of N.I.N.A.
bool isAllowedCommand(const char *cmd);

// Type-first driver configuration ("CX600", "HY40", "SX16"). Its handler writes
// nothing to Serial; callers must also check that N.I.N.A. is not active.
bool isDriverCommand(const char *cmd);

// Relative jog in the plugin's grammar, e.g. "$J=G91G21X800F1000".
bool buildJogCommand(char axis, long steps, int feed, char *out, size_t outLen);

// N.I.N.A. owns the motors while it has sent a byte within kNinaWindowMs.
// Unsigned subtraction keeps this correct across millis() wrap.
bool ninaActive(bool anySerialByte, uint32_t nowMs, uint32_t lastSerialMs);

}  // namespace web
