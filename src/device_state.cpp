#include "device_state.h"

#include <Arduino.h>
#include <Preferences.h>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <esp_system.h>
#include <mbedtls/sha256.h>

#include "ota_rules.h"

namespace device {
namespace {

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
EventLog events;
HistoryLog history;
Preferences prefs;
devset::LoopConfig loopCfg;
bool loopChangePending = false;
devset::DriverConfig storedX, storedY;
bool storedDrivers = false;
devset::DriverConfig activeX, activeY;
char savedHost[16] = "";
uint8_t otaSalt[16];
uint8_t otaHash[32];
bool otaSet = false;
bool invertedX = false;
bool invertedY = false;

void otaDigest(const uint8_t *salt, const char *password, uint8_t out[32]) {
  uint8_t buf[16 + 64];
  size_t len = std::strlen(password);
  if (len > 64) len = 64;
  std::memcpy(buf, salt, 16);
  std::memcpy(buf + 16, password, len);
  mbedtls_sha256_ret(buf, 16 + len, out, 0);
}

}  // namespace

void begin() {
  prefs.begin("oapaset", false);
  devset::LoopConfig c;
  c.factorX = prefs.getDouble("fx", c.factorX);
  c.factorY = prefs.getDouble("fy", c.factorY);
  c.toleranceArcmin = prefs.getDouble("tol", c.toleranceArcmin);
  c.userCapArcmin = prefs.getDouble("cap", c.userCapArcmin);
  c.settleMs = prefs.getUInt("settle", c.settleMs);
  c.feed = prefs.getInt("feed", c.feed);
  c.calThresholdArcmin = prefs.getDouble("calth", c.calThresholdArcmin);
  String host = prefs.getString("ahost", "");
  devset::DriverConfig x, y;
  x.runMa = prefs.getInt("xr", x.runMa);
  x.holdPct = prefs.getInt("xh", x.holdPct);
  x.microsteps = prefs.getInt("xm", x.microsteps);
  y.runMa = prefs.getInt("yr", y.runMa);
  y.holdPct = prefs.getInt("yh", y.holdPct);
  y.microsteps = prefs.getInt("ym", y.microsteps);
  bool invX = prefs.getBool("invx", false);
  bool invY = prefs.getBool("invy", false);
  bool drivers = prefs.getBool("drv", false) && !devset::validateDriver(x) && !devset::validateDriver(y);
  bool ota = prefs.getBytesLength("otasalt") == sizeof otaSalt && prefs.getBytesLength("otahash") == sizeof otaHash &&
             prefs.getBytes("otasalt", otaSalt, sizeof otaSalt) == sizeof otaSalt &&
             prefs.getBytes("otahash", otaHash, sizeof otaHash) == sizeof otaHash;

  portENTER_CRITICAL(&mux);
  if (devset::validateLoop(c) == nullptr) loopCfg = c;
  if (devset::validateHost(host.c_str()) == nullptr) std::snprintf(savedHost, sizeof savedHost, "%s", host.c_str());
  loopChangePending = true;
  otaSet = ota;
  invertedX = invX;
  invertedY = invY;
  storedDrivers = drivers;
  if (drivers) {
    storedX = x;
    storedY = y;
  }
  portEXIT_CRITICAL(&mux);
}

void logEvent(const char *code, const char *fmt, ...) {
  char text[81];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(text, sizeof text, fmt, args);
  va_end(args);
  EventEntry entry = makeEvent(millis(), code, text);
  portENTER_CRITICAL(&mux);
  events.push(entry);
  portEXIT_CRITICAL(&mux);
}

void addHistory(float azArcmin, float altArcmin, bool moved) {
  HistorySample sample;
  sample.uptimeMs = millis();
  sample.azArcmin = azArcmin;
  sample.altArcmin = altArcmin;
  sample.moved = moved;
  portENTER_CRITICAL(&mux);
  history.push(sample);
  portEXIT_CRITICAL(&mux);
}

size_t readEvents(uint32_t afterSeq, EventLog::Item *out, size_t max, uint32_t &lastSeq) {
  portENTER_CRITICAL(&mux);
  size_t n = events.since(afterSeq, out, max);
  lastSeq = events.lastSeq();
  portEXIT_CRITICAL(&mux);
  return n;
}

size_t readHistory(uint32_t afterSeq, HistoryLog::Item *out, size_t max, uint32_t &lastSeq) {
  portENTER_CRITICAL(&mux);
  size_t n = history.since(afterSeq, out, max);
  lastSeq = history.lastSeq();
  portEXIT_CRITICAL(&mux);
  return n;
}

devset::LoopConfig loopConfig() {
  portENTER_CRITICAL(&mux);
  devset::LoopConfig copy = loopCfg;
  portEXIT_CRITICAL(&mux);
  return copy;
}

bool saveLoopConfig(const devset::LoopConfig &c) {
  if (devset::validateLoop(c) != nullptr) return false;
  prefs.putDouble("fx", c.factorX);
  prefs.putDouble("fy", c.factorY);
  prefs.putDouble("tol", c.toleranceArcmin);
  prefs.putDouble("cap", c.userCapArcmin);
  prefs.putUInt("settle", c.settleMs);
  prefs.putInt("feed", c.feed);
  prefs.putDouble("calth", c.calThresholdArcmin);
  portENTER_CRITICAL(&mux);
  loopCfg = c;
  loopChangePending = true;
  portEXIT_CRITICAL(&mux);
  return true;
}

bool takeLoopConfigChange(devset::LoopConfig &out) {
  portENTER_CRITICAL(&mux);
  bool pending = loopChangePending;
  if (pending) {
    out = loopCfg;
    loopChangePending = false;
  }
  portEXIT_CRITICAL(&mux);
  return pending;
}

void asiairHost(char *out, size_t len) {
  portENTER_CRITICAL(&mux);
  std::snprintf(out, len, "%s", savedHost);
  portEXIT_CRITICAL(&mux);
}

bool saveAsiairHost(const char *host) {
  if (devset::validateHost(host) != nullptr) return false;
  prefs.putString("ahost", host);
  portENTER_CRITICAL(&mux);
  std::snprintf(savedHost, sizeof savedHost, "%s", host);
  portEXIT_CRITICAL(&mux);
  return true;
}

bool motorInverted(char axis) {
  portENTER_CRITICAL(&mux);
  bool inverted = axis == 'Y' ? invertedY : invertedX;
  portEXIT_CRITICAL(&mux);
  return inverted;
}

void saveMotorInverted(char axis, bool inverted) {
  prefs.putBool(axis == 'Y' ? "invy" : "invx", inverted);
  portENTER_CRITICAL(&mux);
  (axis == 'Y' ? invertedY : invertedX) = inverted;
  portEXIT_CRITICAL(&mux);
}

bool otaPasswordSet() {
  portENTER_CRITICAL(&mux);
  bool set = otaSet;
  portEXIT_CRITICAL(&mux);
  return set;
}

bool checkOtaPassword(const char *password) {
  if (!password) return false;
  uint8_t salt[16], hash[32], digest[32];
  portENTER_CRITICAL(&mux);
  bool set = otaSet;
  std::memcpy(salt, otaSalt, sizeof salt);
  std::memcpy(hash, otaHash, sizeof hash);
  portEXIT_CRITICAL(&mux);
  if (!set) return false;
  otaDigest(salt, password, digest);
  return ota::constantTimeEqual(digest, hash, sizeof hash);
}

bool setOtaPassword(const char *password) {
  if (ota::validatePassword(password) != nullptr) return false;
  uint8_t salt[16], hash[32];
  esp_fill_random(salt, sizeof salt);
  otaDigest(salt, password, hash);
  prefs.putBytes("otasalt", salt, sizeof salt);
  prefs.putBytes("otahash", hash, sizeof hash);
  portENTER_CRITICAL(&mux);
  std::memcpy(otaSalt, salt, sizeof salt);
  std::memcpy(otaHash, hash, sizeof hash);
  otaSet = true;
  portEXIT_CRITICAL(&mux);
  return true;
}

void clearOtaPassword() {
  prefs.remove("otasalt");
  prefs.remove("otahash");
  portENTER_CRITICAL(&mux);
  otaSet = false;
  portEXIT_CRITICAL(&mux);
}

bool driversStored() {
  portENTER_CRITICAL(&mux);
  bool stored = storedDrivers;
  portEXIT_CRITICAL(&mux);
  return stored;
}

devset::DriverConfig driverConfig(char axis) {
  portENTER_CRITICAL(&mux);
  devset::DriverConfig copy = axis == 'Y' ? storedY : storedX;
  portEXIT_CRITICAL(&mux);
  return copy;
}

bool saveDriverConfig(char axis, const devset::DriverConfig &c) {
  if ((axis != 'X' && axis != 'Y') || devset::validateDriver(c) != nullptr) return false;
  // The first save seeds the other axis with its active values, so a pair is always complete.
  devset::DriverConfig seedX, seedY;
  readActiveDrivers(seedX, seedY);
  bool firstSave = !driversStored();
  devset::DriverConfig newX = firstSave ? seedX : driverConfig('X');
  devset::DriverConfig newY = firstSave ? seedY : driverConfig('Y');
  (axis == 'X' ? newX : newY) = c;

  prefs.putInt("xr", newX.runMa);
  prefs.putInt("xh", newX.holdPct);
  prefs.putInt("xm", newX.microsteps);
  prefs.putInt("yr", newY.runMa);
  prefs.putInt("yh", newY.holdPct);
  prefs.putInt("ym", newY.microsteps);
  prefs.putBool("drv", true);

  portENTER_CRITICAL(&mux);
  storedX = newX;
  storedY = newY;
  storedDrivers = true;
  portEXIT_CRITICAL(&mux);
  return true;
}

void publishActiveDrivers(const devset::DriverConfig &x, const devset::DriverConfig &y) {
  portENTER_CRITICAL(&mux);
  activeX = x;
  activeY = y;
  portEXIT_CRITICAL(&mux);
}

void readActiveDrivers(devset::DriverConfig &x, devset::DriverConfig &y) {
  portENTER_CRITICAL(&mux);
  x = activeX;
  y = activeY;
  portEXIT_CRITICAL(&mux);
}

}  // namespace device
