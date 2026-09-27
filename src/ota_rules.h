// Firmware update over WiFi: the pure rules (password shape, lockout after
// wrong passwords, image check). Hashing and flashing live in the ESP32 code.
#pragma once
#include <cstddef>
#include <cstdint>

namespace ota {

// nullptr when the password is 8-64 printable ASCII characters, otherwise a message.
const char *validatePassword(const char *password);

// Three wrong passwords lock updates for 60 s; the count restarts when the lock expires.
class Lockout {
 public:
  static constexpr int kMaxFailures = 3;
  static constexpr uint32_t kLockMs = 60000;

  bool locked(uint32_t nowMs) const;
  uint32_t remainingMs(uint32_t nowMs) const;
  void fail(uint32_t nowMs);
  void succeed();

 private:
  int failures_ = 0;
  bool lockActive_ = false;
  uint32_t lockStartMs_ = 0;
};

// One upload at a time. The first upload owns the update while its chunks keep
// arriving; another upload is refused instead of aborting it (a double tap mixed two
// uploads' chunks on 14/09/2026). After 15 s without a chunk the owner is abandoned.
// Ids are non-zero.
class UploadGate {
 public:
  static constexpr uint32_t kStaleMs = 15000;

  bool claim(uint32_t id, uint32_t nowMs);
  bool owns(uint32_t id, uint32_t nowMs);  // refreshes the owner's activity
  void release(uint32_t id);

 private:
  uint32_t owner_ = 0;
  uint32_t lastMs_ = 0;
};

// An ESP32 application image starts with the magic byte 0xE9.
bool looksLikeEsp32Image(const uint8_t *data, size_t len);

bool constantTimeEqual(const uint8_t *a, const uint8_t *b, size_t len);

}  // namespace ota
