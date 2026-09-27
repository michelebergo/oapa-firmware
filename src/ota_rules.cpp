#include "ota_rules.h"

#include <cstring>

namespace ota {

const char *validatePassword(const char *password) {
  static const char *kMessage = "password must be 8-64 printable ASCII characters";
  if (!password) return kMessage;
  size_t len = std::strlen(password);
  if (len < 8 || len > 64) return kMessage;
  for (size_t i = 0; i < len; i++) {
    unsigned char c = static_cast<unsigned char>(password[i]);
    if (c < 0x20 || c > 0x7E) return kMessage;
  }
  return nullptr;
}

bool Lockout::locked(uint32_t nowMs) const {
  return lockActive_ && static_cast<uint32_t>(nowMs - lockStartMs_) < kLockMs;
}

uint32_t Lockout::remainingMs(uint32_t nowMs) const {
  return locked(nowMs) ? kLockMs - static_cast<uint32_t>(nowMs - lockStartMs_) : 0;
}

void Lockout::fail(uint32_t nowMs) {
  if (lockActive_ && !locked(nowMs)) {
    lockActive_ = false;
    failures_ = 0;
  }
  if (lockActive_) return;
  if (++failures_ >= kMaxFailures) {
    lockActive_ = true;
    lockStartMs_ = nowMs;
  }
}

void Lockout::succeed() {
  failures_ = 0;
  lockActive_ = false;
}

bool UploadGate::claim(uint32_t id, uint32_t nowMs) {
  if (owner_ != 0 && owner_ != id && static_cast<uint32_t>(nowMs - lastMs_) < kStaleMs) return false;
  owner_ = id;
  lastMs_ = nowMs;
  return true;
}

bool UploadGate::owns(uint32_t id, uint32_t nowMs) {
  if (id == 0 || owner_ != id) return false;
  lastMs_ = nowMs;
  return true;
}

void UploadGate::release(uint32_t id) {
  if (owner_ == id) owner_ = 0;
}

bool looksLikeEsp32Image(const uint8_t *data, size_t len) { return data != nullptr && len > 0 && data[0] == 0xE9; }

bool constantTimeEqual(const uint8_t *a, const uint8_t *b, size_t len) {
  uint8_t diff = 0;
  for (size_t i = 0; i < len; i++) diff |= a[i] ^ b[i];
  return diff == 0;
}

}  // namespace ota
