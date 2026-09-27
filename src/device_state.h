// Saved settings, event log, error history and the drivers' active values,
// shared between core 0 (web) and core 1 (motors) behind one spinlock.
#pragma once
#include <cstddef>
#include <cstdint>

#include "device_config.h"
#include "device_logs.h"

namespace device {

void begin();  // setup(), after net::begin(): loads NVS
void logEvent(const char *code, const char *fmt, ...);
void addHistory(float azArcmin, float altArcmin, bool moved);
size_t readEvents(uint32_t afterSeq, EventLog::Item *out, size_t max, uint32_t &lastSeq);
size_t readHistory(uint32_t afterSeq, HistoryLog::Item *out, size_t max, uint32_t &lastSeq);

devset::LoopConfig loopConfig();
bool saveLoopConfig(const devset::LoopConfig &config);  // validates, writes NVS, marks it for the loop
bool takeLoopConfigChange(devset::LoopConfig &out);     // core 1

void asiairHost(char *out, size_t len);   // "" = automatic (the WiFi gateway)
bool saveAsiairHost(const char *host);    // validates, writes NVS

// Firmware update password: only a salted SHA-256 is stored.
bool otaPasswordSet();
bool checkOtaPassword(const char *password);
bool setOtaPassword(const char *password);  // validates, writes NVS
void clearOtaPassword();

bool driversStored();
devset::DriverConfig driverConfig(char axis);
bool saveDriverConfig(char axis, const devset::DriverConfig &config);
void publishActiveDrivers(const devset::DriverConfig &x, const devset::DriverConfig &y);  // core 1
void readActiveDrivers(devset::DriverConfig &x, devset::DriverConfig &y);               // core 0

// Motor direction, like swapping two wires of the motor: every move of that axis,
// from the page, N.I.N.A. or the alignment, turns the other way.
bool motorInverted(char axis);
void saveMotorInverted(char axis, bool inverted);  // writes NVS

}  // namespace device
