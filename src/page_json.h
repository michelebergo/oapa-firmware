// JSON bodies for the page's endpoints; each returns the length, or -1 when buf is too small.
#pragma once
#include <cstddef>
#include <cstdint>

#include "device_config.h"
#include "device_logs.h"

int formatEventsJson(const EventLog::Item *items, size_t count, uint32_t lastSeq, char *buf, size_t len);
int formatHistoryJson(const HistoryLog::Item *items, size_t count, uint32_t lastSeq, char *buf, size_t len);
int formatSettingsJson(const devset::LoopConfig &config, const char *asiairHost, char *buf, size_t len);
int formatDriversJson(const devset::DriverConfig &x, const devset::DriverConfig &y, char *buf, size_t len);
