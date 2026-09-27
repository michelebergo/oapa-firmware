// Firmware update over WiFi: password rules, lockout after failures, image check.
#pragma once
#include <cstdint>
#include <string>

TEST(OTA_Password_8To64PrintableAscii) {
  CHECK(ota::validatePassword("12345678") == nullptr);
  CHECK(ota::validatePassword(std::string(64, 'a').c_str()) == nullptr);
  CHECK(ota::validatePassword("with space ok") == nullptr);
  CHECK(ota::validatePassword("1234567") != nullptr);
  CHECK(ota::validatePassword(std::string(65, 'a').c_str()) != nullptr);
  CHECK(ota::validatePassword("tab\there1") != nullptr);
  CHECK(ota::validatePassword("caf\xc3\xa9-1234") != nullptr);
  CHECK(ota::validatePassword("") != nullptr);
  CHECK(ota::validatePassword(nullptr) != nullptr);
}

TEST(OTA_Lockout_ThreeFailuresLockForSixtySeconds) {
  ota::Lockout l;
  CHECK(!l.locked(0));
  l.fail(1000);
  l.fail(2000);
  CHECK(!l.locked(2000));
  l.fail(3000);
  CHECK(l.locked(3000));
  CHECK(l.remainingMs(3000) == 60000);
  CHECK(l.locked(62999));
  CHECK(l.remainingMs(62999) == 1);
  CHECK(!l.locked(63000));
  CHECK(l.remainingMs(63000) == 0);
  l.fail(64000);  // the count restarted when the lock expired
  CHECK(!l.locked(64000));
}

TEST(OTA_Lockout_SuccessClearsTheCount_AndSurvivesMillisWrap) {
  ota::Lockout l;
  l.fail(10);
  l.fail(20);
  l.succeed();
  l.fail(30);
  l.fail(40);
  CHECK(!l.locked(40));
  ota::Lockout w;
  for (int i = 0; i < 3; i++) w.fail(0xFFFFF000u);
  CHECK(w.locked(0x00001000u));  // 8 s after the lock, across the wrap
  CHECK(!w.locked(0xFFFFF000u + 60000u));
}

TEST(OTA_ImageCheck_Esp32MagicByte) {
  const uint8_t good[] = {0xE9, 0x05, 0x02};
  const uint8_t bad[] = {0x7F, 'E', 'L', 'F'};
  CHECK(ota::looksLikeEsp32Image(good, sizeof good));
  CHECK(!ota::looksLikeEsp32Image(bad, sizeof bad));
  CHECK(!ota::looksLikeEsp32Image(good, 0));
  CHECK(!ota::looksLikeEsp32Image(nullptr, 3));
}

TEST(OTA_ConstantTimeEqual) {
  const uint8_t a[] = {1, 2, 3, 4};
  const uint8_t b[] = {1, 2, 3, 4};
  const uint8_t c[] = {1, 2, 3, 5};
  CHECK(ota::constantTimeEqual(a, b, 4));
  CHECK(!ota::constantTimeEqual(a, c, 4));
  CHECK(ota::constantTimeEqual(a, c, 3));
}
