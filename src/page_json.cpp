#include "page_json.h"

#include <cstdarg>
#include <cstdio>

#include "json_util.h"

namespace {

struct Appender {
  char *buf;
  size_t len;
  size_t pos = 0;
  bool ok = true;

  void add(const char *fmt, ...) {
    if (!ok) return;
    va_list args;
    va_start(args, fmt);
    int n = std::vsnprintf(buf + pos, len - pos, fmt, args);
    va_end(args);
    if (n < 0 || static_cast<size_t>(n) >= len - pos) {
      ok = false;
      return;
    }
    pos += static_cast<size_t>(n);
  }

  int result() const { return ok ? static_cast<int>(pos) : -1; }
};

}  // namespace

int formatEventsJson(const EventLog::Item *items, size_t count, uint32_t lastSeq, char *buf, size_t len) {
  Appender a{buf, len};
  a.add("{\"last\":%lu,\"events\":[", static_cast<unsigned long>(lastSeq));
  for (size_t i = 0; i < count; i++) {
    char code[40], text[200];
    if (!jsonEscape(items[i].value.code, code, sizeof code) || !jsonEscape(items[i].value.text, text, sizeof text)) {
      return -1;
    }
    a.add("%s{\"seq\":%lu,\"t\":%lu,\"code\":\"%s\",\"text\":\"%s\"}", i ? "," : "",
          static_cast<unsigned long>(items[i].seq), static_cast<unsigned long>(items[i].value.uptimeMs), code, text);
  }
  a.add("]}");
  return a.result();
}

int formatHistoryJson(const HistoryLog::Item *items, size_t count, uint32_t lastSeq, char *buf, size_t len) {
  Appender a{buf, len};
  a.add("{\"last\":%lu,\"samples\":[", static_cast<unsigned long>(lastSeq));
  for (size_t i = 0; i < count; i++) {
    const HistorySample &s = items[i].value;
    a.add("%s[%lu,%lu,%.2f,%.2f,%d]", i ? "," : "", static_cast<unsigned long>(items[i].seq),
          static_cast<unsigned long>(s.uptimeMs), s.azArcmin, s.altArcmin, s.moved ? 1 : 0);
  }
  a.add("]}");
  return a.result();
}

int formatSettingsJson(const devset::LoopConfig &c, const char *asiairHost, char *buf, size_t len) {
  char host[40];
  if (!jsonEscape(asiairHost ? asiairHost : "", host, sizeof host)) return -1;
  Appender a{buf, len};
  a.add("{\"factorX\":%.3f,\"factorY\":%.3f,\"tolerance\":%.2f,\"cap\":%.2f,\"settleMs\":%lu,\"feed\":%d,"
        "\"calThresholdArcmin\":%.2f,\"asiairHost\":\"%s\"}",
        c.factorX, c.factorY, c.toleranceArcmin, c.userCapArcmin, static_cast<unsigned long>(c.settleMs), c.feed,
        c.calThresholdArcmin, host);
  return a.result();
}

int formatDriversJson(const devset::DriverConfig &x, const devset::DriverConfig &y, char *buf, size_t len) {
  Appender a{buf, len};
  a.add("{\"x\":{\"run\":%d,\"hold\":%d,\"micro\":%d},\"y\":{\"run\":%d,\"hold\":%d,\"micro\":%d}}", x.runMa, x.holdPct,
        x.microsteps, y.runMa, y.holdPct, y.microsteps);
  return a.result();
}
